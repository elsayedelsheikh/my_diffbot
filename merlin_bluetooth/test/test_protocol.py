import asyncio
import json

from cryptography.hazmat.primitives.ciphers.aead import AESGCM

from merlin_bluetooth import srp
from merlin_bluetooth.protocol import (
    LOCKOUT_FAILURES, LOCKOUT_S, MAX_MESSAGE, Lockout, Reassembler, Session, b64, encode, nonce, unb64,
)
from merlin_bluetooth.wifi import failure

PIN = '12345678'


def h(text):
    return int(''.join(text.split()), 16)


# RFC 5054 Appendix A (1024-bit group) and Appendix B (SHA-1 test vectors).
N1024 = h('''EEAF0AB9 ADB38DD6 9C33F80A FA8FC5E8 60726187 75FF3C0B 9EA2314C 9C256576 D674DF74 96EA81D3
    383B4813 D692C6E0 E0D5D8E2 50B98BE4 8E495C1D 6089DAD1 5DC7D7B4 6154D6B6 CE8EF4AD 69B15D49
    82559B29 7BCF1885 C529F566 660E57EC 68EDBC3C 05726CC0 2FD4CBF4 976EAA9A FD5138FE 8376435B
    9FC61D2F C0EB06E3''')
RFC = dict(
    s=bytes.fromhex('BEB25379D1A8581EB5A727673A2441EE'),
    k=h('7556AA04 5AEF2CDD 07ABAF0F 665C3E81 8913186F'),
    x=h('94B7555A ABE9127C C58CCF49 93DB6CF8 4D16C124'),
    v=h('''7E273DE8 696FFC4F 4E337D05 B4B375BE B0DDE156 9E8FA00A 9886D812 9BADA1F1 822223CA 1A605B53
        0E379BA4 729FDC59 F105B478 7E5186F5 C671085A 1447B52A 48CF1970 B4FB6F84 00BBF4CE BFBB1681
        52E08AB5 EA53D15C 1AFF87B2 B9DA6E04 E058AD51 CC72BFC9 033B564E 26480D78 E955A5E2 9E7AB245
        DB2BE315 E2099AFB'''),
    a=h('60975527 035CF2AD 1989806F 0407210B C81EDC04 E2762A56 AFD529DD DA2D4393'),
    b=h('E487CB59 D31AC550 471E81F0 0F6928E0 1DDA08E9 74A004F4 9E61F5D1 05284D20'),
    A=h('''61D5E490 F6F1B795 47B0704C 436F523D D0E560F0 C64115BB 72557EC4 4352E890 3211C046 92272D8B
        2D1A5358 A2CF1B6E 0BFCF99F 921530EC 8E393561 79EAE45E 42BA92AE ACED8251 71E1E8B9 AF6D9C03
        E1327F44 BE087EF0 6530E69F 66615261 EEF54073 CA11CF58 58F0EDFD FE15EFEA B349EF5D 76988A36
        72FAC47B 0769447B'''),
    B=h('''BD0C6151 2C692C0C B6D041FA 01BB152D 4916A1E7 7AF46AE1 05393011 BAF38964 DC46A067 0DD125B9
        5A981652 236F99D9 B681CBF8 7837EC99 6C6DA044 53728610 D0C6DDB5 8B318885 D7D82C7F 8DEB75CE
        7BD4FBAA 37089E6F 9C6059F3 88838E7A 00030B33 1EB76840 910440B1 B27AAEAE EB4012B7 D7665238
        A8E3FB00 4B117B58'''),
    u=h('CE38B959 3487DA98 554ED47D 70A7AE5F 462EF019'),
    S=h('''B0DC82BA BCF30674 AE450C02 87745E79 90A3381F 63B387AA F271A10D 233861E3 59B48220 F7C4693C
        9AE12B0A 6F67809F 0876E2D0 13800D6C 41BB59B6 D5979B5C 00A172B4 A2A5903A 0BDCAF8A 709585EB
        2AFAFA8F 3499B200 210DCC1F 10EB3394 3CD67FC8 8A2F39A4 BE5BEC4E C0A3212D C346D7E4 74B29EDE
        8A469FFE CA686E5A'''),
)


def test_srp_rfc5054_vectors():
    sha1, g, r = 'sha1', 2, RFC
    assert srp.multiplier(sha1, N1024, g) == r['k']
    assert srp.private_key(sha1, r['s'], b'alice', b'password123') == r['x']
    assert srp.verifier(sha1, N1024, g, r['s'], b'alice', b'password123') == r['v']
    assert pow(g, r['a'], N1024) == r['A']
    assert srp.server_public(sha1, N1024, g, r['v'], r['b']) == r['B']
    assert srp.scramble(sha1, N1024, r['A'], r['B']) == r['u']
    assert srp.server_secret(N1024, r['A'], r['v'], r['u'], r['b']) == r['S']
    assert srp.client_secret(sha1, N1024, g, r['x'], r['a'], r['u'], r['B']) == r['S']


class Phone:
    """The app's side (merlin_android Srp.kt), for driving a Session end to end."""

    def __init__(self, pin, a=None):
        self.pin, self.a, self.n = pin.encode(), a or 0x1234567890ABCDEF, 0

    def verify_msg(self, start):
        N, g, sha = srp.N_2048, srp.G, 'sha256'
        salt, B = unb64(start['salt']), srp.to_int(unb64(start['B']))
        A = pow(g, self.a, N)
        u = srp.scramble(sha, N, A, B)
        x = srp.private_key(sha, salt, srp.IDENTITY, self.pin)
        K, m1, self.m2 = srp.proofs(sha, N, A, B, srp.client_secret(sha, N, g, x, self.a, u, B))
        self.tx, self.rx = (AESGCM(k) for k in srp.session_keys(K))
        return {'id': 2, 'op': 'pair_verify', 'A': b64(srp.pad(A, N)), 'M1': b64(m1)}

    def seal(self, msg):
        self.n += 1
        return {'n': self.n, 'c': b64(self.tx.encrypt(nonce(self.n), json.dumps(msg, separators=(',', ':')).encode(), None))}

    def open(self, reply):
        return json.loads(self.rx.decrypt(nonce(reply['n']), unb64(reply['c']), None))


class FakeRobot:
    async def status(self):
        return {'name': 'Merlin', 'wifi': {'mode': 'station', 'ssid': 'x' * 32, 'ip': '192.168.100.100'},
                'camera': {'user': 'merlin', 'password': 'p' * 24, 'port': 8889}, 'bridge_port': 8766}

    async def join_wifi(self, ssid, password):
        return (None, ('not_found', 'x')) if ssid == 'nope' else ({'ip': '1.2.3.4'}, None)

    async def start_hotspot(self):
        return {'mode': 'hotspot', 'ssid': 'Merlin-hotspot', 'password': 'p' * 12, 'ip': '10.42.0.1'}, None


def run(session, msg):
    reply = asyncio.run(session.handle(msg))
    assert len(encode(reply)) <= MAX_MESSAGE, 'every reply must fit one GATT value'
    return reply


def paired(lockout=None):
    s, phone = Session('Merlin', PIN, FakeRobot(), lockout or Lockout()), Phone(PIN)
    verify = phone.verify_msg(run(s, {'id': 1, 'op': 'pair_start'}))
    assert len(encode(verify)) <= MAX_MESSAGE
    reply = run(s, verify)
    assert reply['ok'] and unb64(reply['M2']) == phone.m2, 'the robot proves the PIN back'
    return s, phone


def test_pairing_then_sealed_requests():
    s, phone = paired()
    status = phone.open(run(s, phone.seal({'id': 3, 'op': 'status'})))
    assert status['ok'] and status['camera']['password'] == 'p' * 24
    assert phone.open(run(s, phone.seal({'id': 4, 'op': 'join_wifi', 'ssid': 'nope'})))['error'] == 'not_found'
    assert phone.open(run(s, phone.seal({'id': 5, 'op': 'start_hotspot'})))['ssid'] == 'Merlin-hotspot'
    assert phone.open(run(s, phone.seal({'id': 6, 'op': 'fly'})))['error'] == 'unknown_op'


def test_nothing_without_the_pin():
    s = Session('Merlin', PIN, FakeRobot(), Lockout())
    assert run(s, {'id': 1, 'op': 'status'})['error'] == 'unauthorized'  # plaintext requests are refused
    phone = Phone('87654321')
    assert run(s, phone.verify_msg(run(s, {'id': 1, 'op': 'pair_start'})))['error'] == 'bad_pin'
    assert run(s, phone.seal({'id': 3, 'op': 'status'}))['error'] == 'unauthorized'
    assert run(s, None)['error'] == 'bad_request'


def test_replay_and_tamper_refused():
    s, phone = paired()
    sealed = phone.seal({'id': 3, 'op': 'status'})
    assert 'c' in run(s, sealed)
    assert run(s, sealed)['error'] == 'bad_request'  # same counter again
    forged = phone.seal({'id': 4, 'op': 'status'})
    forged['c'] = b64(unb64(forged['c'])[:-1] + b'\0')
    assert run(s, forged)['error'] == 'bad_request'


def test_lockout_doubles_and_survives_reconnects():
    t = [0.0]
    lockout = Lockout(now=lambda: t[0])

    def attempt(pin):
        s = Session('M', PIN, FakeRobot(), lockout)  # a fresh connection each time
        start = run(s, {'id': 1, 'op': 'pair_start'})
        return start if not start['ok'] else run(s, Phone(pin).verify_msg(start))

    for wait in (LOCKOUT_S, 2 * LOCKOUT_S):
        for _ in range(LOCKOUT_FAILURES):
            assert attempt('00000000')['error'] == 'bad_pin'
        assert attempt(PIN)['error'] == 'locked'
        t[0] += wait + 1
    assert attempt(PIN)['ok']


def test_framing():
    r = Reassembler()
    msg = {'op': 'pair_verify', 'A': 'x' * 400}
    data = encode(msg)
    assert r.feed(data[:100]) == [] and r.feed(data[100:]) == [msg]
    assert r.feed(b'not json\n[1]\n') == [None, None]
    assert r.feed(b'x' * 600) == [None]  # oversized line is dropped, not buffered forever


def test_nmcli_failures():
    assert failure('Error: No network with SSID "x" found.')[0] == 'not_found'
    assert failure('Error: Connection activation failed: Secrets were required, but not provided.')[0] == 'auth_failed'
    assert failure('Error: something else')[0] == 'failed'


def test_cross_language_vector():
    """Same fixed inputs as merlin_android SrpTest.kt: both sides must derive identical bytes."""
    server = srp.Server(PIN, salt=bytes(range(16)), b=0xB0B)
    phone = Phone(PIN, a=0xA11CE)
    start = {'salt': b64(server.salt), 'B': b64(srp.pad(server.B, srp.N_2048))}
    verify = phone.verify_msg(start)
    K, m2 = server.verify(srp.to_int(unb64(verify['A'])), unb64(verify['M1']))
    assert unb64(verify['M1']).hex() == VECTOR['M1'] and m2.hex() == VECTOR['M2']
    assert srp.session_keys(K)[0].hex() == VECTOR['key_phone_to_robot']
    assert phone.seal({'op': 'status'})['c'] == VECTOR['sealed_status']


VECTOR = {'M1': '87c7ef7a3c047ef4d93a1e7f504c38e13b6e2b0067ada0c3df0f0371d15ae916', 'M2': '4cb4b53e291346546feb9247fad37522c132f3559c2d035e09e614bfe6eaea8b', 'key_phone_to_robot': '7d47a18953b8a606bc8a31324b68b644730802fc376556c688d70c9b111a014b', 'sealed_status': 'leu41egdgn37KLtWNsW3C0ayDD8ryNiAmnQgoFW9pw=='}  # mirrored in SrpTest.kt
