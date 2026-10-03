#!/usr/bin/env python3
"""merlin_bluetooth: BLE provisioning link between the Merlin app and the robot.

The phone proves the robot PIN with SRP-6a (the PIN never crosses the air), then over an
AES-GCM sealed session reads the robot's IP, camera password and ports, or moves wlan0 to another network or the robot's own
hotspot. Protocol: README.md. BlueZ, nmcli and the sessions run on an asyncio loop in
its own thread; ROS only provides parameters, logging and the process lifecycle.
"""

import asyncio
import os
import re
from threading import Thread

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import UInt32

from merlin_bluetooth import wifi
from merlin_bluetooth.gatt import GattServer
from merlin_bluetooth.protocol import MAX_MESSAGE, Lockout, Reassembler, Session, encode

def uptime_s():
    """Seconds since the Jetson booted (the container shares the host kernel's clock)."""
    with open('/proc/uptime') as f:
        return int(float(f.read().split()[0]))


def camera_password(path):
    """The viewer password install.sh wrote into the MediaMTX config, '' if unreadable."""
    try:
        with open(path) as f:
            m = re.search(r'^\s*pass:\s*(\S+)', f.read(), re.MULTILINE)
    except OSError:
        return ''
    return m.group(1) if m else ''


class Robot:
    """What a session can ask for. Wi-Fi results are (fields, None) | (None, (code, text))."""

    def __init__(self, node):
        self._node = node
        self._watchdog = None

    async def status(self):
        p = self._node.params
        return {
            'name': p['name'],
            'uptime_s': uptime_s(),
            'wifi': await wifi.status(),
            'camera': {'user': 'merlin', 'password': camera_password(p['camera_config']),
                       'port': p['whep_port']},
            'bridge_port': p['bridge_port'],
        }

    async def join_wifi(self, ssid, password):
        self._cancel_watchdog()
        return await wifi.join(ssid, password)

    async def start_hotspot(self):
        previous, mode, _ = await wifi.active()
        fields, err = await wifi.start_hotspot(self._node.params['name'])
        if not err and mode == 'station':
            self._cancel_watchdog()
            self._watchdog = asyncio.ensure_future(self._revert_unless_joined(previous))
        return fields, err

    async def shutdown(self):
        # Powers the Jetson off (host systemd, through PID 1's mount namespace). Delayed so the
        # reply reaches the phone first; docker stops the containers, so the wheels stop too.
        self._node.get_logger().warning('shutdown requested over Bluetooth: powering off in 2 s')
        asyncio.get_running_loop().call_later(2.0, lambda: asyncio.ensure_future(
            asyncio.create_subprocess_exec('nsenter', '-t', '1', '-m', '--', 'systemctl', 'poweroff')))

    def _cancel_watchdog(self):
        if self._watchdog is not None:
            self._watchdog.cancel()
            self._watchdog = None

    async def _revert_unless_joined(self, previous):
        # Nobody joined the hotspot: go back to the network the robot came from, so a
        # failed hand-over can't strand it off every LAN.
        timeout = self._node.params['hotspot_timeout']
        log = self._node.get_logger()
        for _ in range(int(timeout // 5)):
            await asyncio.sleep(5)
            if await wifi.station_count():
                log.info('hotspot: a client joined')
                return
        log.warning(f'hotspot: nobody joined in {timeout:.0f} s; back to {previous!r}')
        await wifi.nmcli('connection', 'up', previous)


class BluetoothNode(Node):
    def __init__(self):
        super().__init__('merlin_bluetooth')
        self.params = {
            'name': self.declare_parameter('name', 'Merlin').value,
            'camera_config': self.declare_parameter('camera_config', '/etc/merlin/mediamtx.yml').value,
            'whep_port': self.declare_parameter('whep_port', 8889).value,
            'bridge_port': self.declare_parameter('bridge_port', 8766).value,
            'hotspot_timeout': float(self.declare_parameter('hotspot_timeout', 300.0).value),
        }
        self._pin = os.environ.get('MERLIN_BT_PIN', '')
        if not re.fullmatch(r'[0-9]{6,12}', self._pin):  # 8+ recommended: lockouts only slow guessing
            raise RuntimeError('MERLIN_BT_PIN must be 6-12 digits (set it in the robot .env)')
        self._robot = Robot(self)
        self._lockout = Lockout()
        self._sessions = {}  # device path -> (Session, Reassembler)
        # For the app's status bar, over the lean foxglove bridge (merlin_bringup foxglove_app.yaml).
        self._uptime_pub = self.create_publisher(UInt32, '/merlin/uptime', 1)
        self.create_timer(5.0, lambda: self._uptime_pub.publish(UInt32(data=uptime_s())))
        self.server = GattServer(self.params['name'], self._on_message, self._on_disconnect,
                                 self.get_logger())

    async def _on_message(self, device, data):
        if device not in self._sessions:
            session = Session(self.params['name'], self._pin, self._robot, self._lockout)
            self._sessions[device] = (session, Reassembler())
        session, reassembler = self._sessions[device]
        reply = None
        for msg in reassembler.feed(data):
            reply = await session.handle(msg)
            # Sealed traffic stays opaque in the log; pairing shows only its outcome.
            what = (msg or {}).get('op') or ('sealed' if msg and 'c' in msg else 'bad')
            self.get_logger().info(f'{device}: {what} -> {reply.get("error") or "ok"}')
        if reply is None:
            return None
        data = encode(reply)
        if len(data) > MAX_MESSAGE:  # one GATT value; a longer reply would arrive truncated
            self.get_logger().error(f'{device}: reply is {len(data)} bytes, over {MAX_MESSAGE}')
        return data

    def _on_disconnect(self, device):
        if self._sessions.pop(device, None):
            self.get_logger().info(f'{device} disconnected')


def _run_loop(loop, coro):
    try:
        loop.run_until_complete(coro)
    finally:
        rclpy.try_shutdown()  # a dead loop leaves the robot unreachable: let launch respawn


def main(args=None):
    rclpy.init(args=args)
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    node = BluetoothNode()
    Thread(target=_run_loop, args=(loop, node.server.run()), daemon=True).start()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
