"""Merlin BLE provisioning protocol, transport-free (no ROS, no D-Bus). Spec: README.md.

Each GATT write is one UTF-8 JSON object ending in a newline. pair_start/pair_verify run
SRP-6a against the robot PIN; after that every request and reply is sealed with AES-GCM.
"""

import base64
import json
import time

from cryptography.exceptions import InvalidTag
from cryptography.hazmat.primitives.ciphers.aead import AESGCM

from merlin_bluetooth import srp

PROTO = 2
MAX_MESSAGE = 512  # bytes: one GATT attribute value
LOCKOUT_FAILURES = 5  # wrong PINs before a lockout
LOCKOUT_S = 60.0  # first lockout; doubles with each one after it
LOCKOUT_MAX_S = 3600.0


def b64(data):
    return base64.b64encode(data).decode()


def unb64(text):
    return base64.b64decode(text, validate=True)


def nonce(counter):
    return counter.to_bytes(12, 'big')


def encode(msg):
    return json.dumps(msg, separators=(',', ':')).encode() + b'\n'


class Reassembler:
    """Collects written bytes; returns each complete line's message (None if not a JSON object)."""

    def __init__(self):
        self._buf = b''

    def feed(self, data):
        self._buf += bytes(data)
        *lines, self._buf = self._buf.split(b'\n')
        if len(self._buf) > MAX_MESSAGE:
            self._buf = b''
            lines.append(b'')  # reported as a bad message
        out = []
        for line in lines:
            try:
                msg = json.loads(line)
            except ValueError:
                msg = None
            out.append(msg if isinstance(msg, dict) else None)
        return out


class Lockout:
    """Wrong-PIN counter shared by every connection: reconnecting doesn't reset it."""

    def __init__(self, now=time.monotonic):
        self._now = now
        self._failures = 0
        self._until = 0.0
        self._next = LOCKOUT_S

    def remaining(self):
        return max(0.0, self._until - self._now())

    def record(self, ok):
        if ok:
            self._failures, self._next = 0, LOCKOUT_S
            return
        self._failures += 1
        if self._failures >= LOCKOUT_FAILURES:
            self._failures = 0
            self._until = self._now() + self._next
            self._next = min(self._next * 2, LOCKOUT_MAX_S)


def error(req_id, code, text=''):
    return {'id': req_id, 'ok': False, 'error': code, 'msg': text}


class Session:
    """One BLE connection. `robot` does the work (see node.Robot); handle() returns the reply."""

    def __init__(self, name, pin, robot, lockout, make_server=srp.Server):
        self._name = name
        self._pin = pin
        self._robot = robot
        self._lockout = lockout
        self._make_server = make_server
        self._srp = None
        self._rx = self._tx = None  # AESGCM, set once the PIN is proven
        self._rx_count = 0  # highest phone counter seen: replays and reorders are refused
        self._tx_count = 0

    @property
    def authed(self):
        return self._rx is not None

    async def handle(self, msg):
        if msg is None:
            return error(None, 'bad_request', 'not a JSON object')
        op = msg.get('op')
        if op == 'pair_start':
            return self._pair_start(msg.get('id'))
        if op == 'pair_verify':
            return self._pair_verify(msg)
        if 'c' in msg and self.authed:
            return await self._sealed(msg)
        return error(msg.get('id'), 'unauthorized', 'pair first (pair_start, pair_verify)')

    def _pair_start(self, req_id):
        if wait := self._lockout.remaining():
            return error(req_id, 'locked', f'too many wrong PINs; try again in {wait:.0f} s')
        self._rx = self._tx = None
        self._srp = self._make_server(self._pin)
        return {'id': req_id, 'ok': True, 'proto': PROTO, 'salt': b64(self._srp.salt),
                'B': b64(srp.pad(self._srp.B, srp.N_2048))}

    def _pair_verify(self, msg):
        req_id, server, self._srp = msg.get('id'), self._srp, None  # one proof per pair_start
        if server is None:
            return error(req_id, 'bad_request', 'send pair_start first')
        if self._lockout.remaining():
            return error(req_id, 'locked', 'too many wrong PINs')
        try:
            A, m1 = srp.to_int(unb64(msg['A'])), unb64(msg['M1'])
        except (KeyError, TypeError, ValueError):
            return error(req_id, 'bad_request', 'A and M1 (base64)')
        result = server.verify(A, m1)
        self._lockout.record(result is not None)
        if result is None:
            return error(req_id, 'bad_pin', 'wrong PIN')
        K, m2 = result
        rx_key, tx_key = srp.session_keys(K)
        self._rx, self._tx = AESGCM(rx_key), AESGCM(tx_key)
        self._rx_count = self._tx_count = 0
        return {'id': req_id, 'ok': True, 'M2': b64(m2), 'name': self._name}

    async def _sealed(self, envelope):
        try:
            counter = int(envelope['n'])
            if counter <= self._rx_count:
                raise ValueError('replayed counter')
            inner = json.loads(self._rx.decrypt(nonce(counter), unb64(envelope['c']), None))
            if not isinstance(inner, dict):
                raise ValueError('not an object')
        except (KeyError, TypeError, ValueError, InvalidTag):
            return error(None, 'bad_request', 'undecryptable message')
        self._rx_count = counter
        self._tx_count += 1
        reply = await self._dispatch(inner)
        sealed = self._tx.encrypt(nonce(self._tx_count), encode(reply)[:-1], None)
        return {'n': self._tx_count, 'c': b64(sealed)}

    async def _dispatch(self, msg):
        req_id, op = msg.get('id'), msg.get('op')
        if op == 'status':
            return {'id': req_id, 'ok': True, **await self._robot.status()}
        if op == 'join_wifi':
            ssid, password = msg.get('ssid'), msg.get('password', '')
            if not isinstance(ssid, str) or not ssid or not isinstance(password, str):
                return error(req_id, 'bad_request', 'ssid (and optional password) strings')
            return await self._result(req_id, self._robot.join_wifi(ssid, password))
        if op == 'start_hotspot':
            return await self._result(req_id, self._robot.start_hotspot())
        if op == 'shutdown':
            await self._robot.shutdown()
            return {'id': req_id, 'ok': True}
        return error(req_id, 'unknown_op', str(op))

    @staticmethod
    async def _result(req_id, work):
        """Wi-Fi results are (fields, None) on success or (None, (code, text)) on failure."""
        fields, err = await work
        if err:
            return error(req_id, *err)
        return {'id': req_id, 'ok': True, **fields}
