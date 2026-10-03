"""BlueZ glue: the Merlin GATT service and its LE advertisement.

Talks to the host bluetoothd over the system D-Bus; no btmgmt, no raw HCI, so the
adapter keeps BlueZ's defaults. Nothing bonds: the protocol secures the session.
"""

import asyncio

from dbus_next import BusType, Message, MessageType, Variant
from dbus_next.aio import MessageBus
from dbus_next.constants import PropertyAccess
from dbus_next.service import ServiceInterface, dbus_property, method

SERVICE_UUID = '7f1d0000-5c2a-4b8e-9d3f-6a1e2b4c8d01'
REQUEST_UUID = '7f1d0001-5c2a-4b8e-9d3f-6a1e2b4c8d01'
RESPONSE_UUID = '7f1d0002-5c2a-4b8e-9d3f-6a1e2b4c8d01'
APP = '/merlin'
SERVICE = APP + '/service0'
REQUEST = SERVICE + '/char0'
RESPONSE = SERVICE + '/char1'
ADVERT = APP + '/advert0'
RETRY_S = 5.0
MAX_VALUE = 512  # ATT attribute limit
# No link-layer security: the Nano's L4T kernel only does LE legacy pairing (sniffable) and
# BlueZ 5.53 segfaults when a bonded phone reconnects. The session is secured end to end by
# the protocol instead (SRP-6a + AES-GCM), on any kernel; no bonding means no pairing dialog.
# Requests are write-without-response: BlueZ 5.53 loses our WriteValue reply and fails every
# acknowledged write after 5 s. The link layer still acknowledges each packet.
REQUEST_FLAGS = ['write-without-response']
RESPONSE_FLAGS = ['read', 'notify']


class _ObjectManager(ServiceInterface):
    def __init__(self, objects):
        super().__init__('org.freedesktop.DBus.ObjectManager')
        self._objects = objects

    @method()
    def GetManagedObjects(self) -> 'a{oa{sa{sv}}}':  # noqa: F821, N802
        return self._objects


class _Service(ServiceInterface):
    def __init__(self):
        super().__init__('org.bluez.GattService1')

    @dbus_property(access=PropertyAccess.READ)
    def UUID(self) -> 's':  # noqa: F821, N802
        return SERVICE_UUID

    @dbus_property(access=PropertyAccess.READ)
    def Primary(self) -> 'b':  # noqa: F821, N802
        return True


class _Characteristic(ServiceInterface):
    """Request: the phone writes one JSON line. Response: read your reply; notify = reply ready."""

    def __init__(self, uuid, flags, server):
        super().__init__('org.bluez.GattCharacteristic1')
        self._uuid = uuid
        self._flags = flags
        self._server = server
        self._value = b''

    @dbus_property(access=PropertyAccess.READ)
    def UUID(self) -> 's':  # noqa: F821, N802
        return self._uuid

    @dbus_property(access=PropertyAccess.READ)
    def Service(self) -> 'o':  # noqa: F821, N802
        return SERVICE

    @dbus_property(access=PropertyAccess.READ)
    def Flags(self) -> 'as':  # noqa: F821, N802
        return self._flags

    @dbus_property(access=PropertyAccess.READ)
    def Value(self) -> 'ay':  # noqa: F821, N802
        return self._value

    @method()
    def ReadValue(self, options: 'a{sv}') -> 'ay':  # noqa: F821, F722, N802
        offset = options.get('offset', Variant('q', 0)).value
        return self._server.read(_device(options))[offset:offset + MAX_VALUE]

    @method()
    def WriteValue(self, value: 'ay', options: 'a{sv}'):  # noqa: F821, F722, N802
        self._server.write(_device(options), bytes(value))

    @method()
    def StartNotify(self):  # noqa: N802
        pass

    @method()
    def StopNotify(self):  # noqa: N802
        pass

    def notify(self, value):
        # Every subscribed device gets this: only a sequence number, never a reply.
        self._value = value
        self.emit_properties_changed({'Value': value})


class _Advertisement(ServiceInterface):
    def __init__(self, name):
        super().__init__('org.bluez.LEAdvertisement1')
        self._name = name

    @method()
    def Release(self):  # noqa: N802
        pass

    @dbus_property(access=PropertyAccess.READ)
    def Type(self) -> 's':  # noqa: F821, N802
        return 'peripheral'

    @dbus_property(access=PropertyAccess.READ)
    def ServiceUUIDs(self) -> 'as':  # noqa: F821, N802
        return [SERVICE_UUID]

    @dbus_property(access=PropertyAccess.READ)
    def LocalName(self) -> 's':  # noqa: F821, N802
        return self._name


def _device(options):
    return options['device'].value if 'device' in options else ''


def _chrc(uuid, flags):
    return {'org.bluez.GattCharacteristic1': {
        'UUID': Variant('s', uuid), 'Service': Variant('o', SERVICE), 'Flags': Variant('as', flags)}}


class GattServer:
    """Keeps the service, advertisement and agent registered; survives bluetoothd restarts.

    on_message(device, data) -> awaitable reply bytes; on_disconnect(device) drops its session.
    """

    def __init__(self, name, on_message, on_disconnect, log):
        self._name = name
        self._on_message = on_message
        self._on_disconnect = on_disconnect
        self._log = log
        self._bus = None
        self._replies = {}  # device path -> last reply bytes
        self._seq = 0
        self._tasks = set()
        self._request = _Characteristic(REQUEST_UUID, REQUEST_FLAGS, self)
        self._response = _Characteristic(RESPONSE_UUID, RESPONSE_FLAGS, self)

    def read(self, device):
        return self._replies.get(device, b'')

    def write(self, device, data):
        task = asyncio.ensure_future(self._serve(device, data))
        self._tasks.add(task)
        task.add_done_callback(self._tasks.discard)

    async def _serve(self, device, data):
        try:
            reply = await self._on_message(device, data)
        except Exception as e:  # noqa: BLE001 - one bad request must not kill the server
            self._log.error(f'{device}: request failed: {e!r}')
            return
        if reply is None:
            return
        self._replies[device] = reply
        self._seq = (self._seq + 1) % 65536
        self._response.notify(self._seq.to_bytes(2, 'little'))

    async def run(self):
        owner = None
        while True:
            try:
                if self._bus is None:
                    self._bus = await MessageBus(bus_type=BusType.SYSTEM).connect()
                    self._export()
                    self._bus.add_message_handler(self._on_signal)
                    await self._call('org.freedesktop.DBus', '/org/freedesktop/DBus',
                                     'org.freedesktop.DBus', 'AddMatch', 's',
                                     ["type='signal',sender='org.bluez',"
                                      "interface='org.freedesktop.DBus.Properties',"
                                      "member='PropertiesChanged',arg0='org.bluez.Device1'"])
                    owner = None
                current = (await self._call('org.freedesktop.DBus', '/org/freedesktop/DBus',
                                            'org.freedesktop.DBus', 'GetNameOwner', 's',
                                            ['org.bluez']))[0]
                if current != owner:
                    await self._register()
                    owner = current
                    self._log.info(f"advertising as '{self._name}' (BLE GATT)")
            except Exception as e:  # noqa: BLE001 - no adapter/bluetoothd yet: retry
                self._log.warning(f'bluetooth not ready: {e}', throttle_duration_sec=60.0)
                owner = None
                if self._bus is not None and not self._bus.connected:
                    self._bus = None
            await asyncio.sleep(RETRY_S)

    def _export(self):
        service = _Service()
        objects = {
            SERVICE: {'org.bluez.GattService1': {
                'UUID': Variant('s', SERVICE_UUID), 'Primary': Variant('b', True)}},
            REQUEST: _chrc(REQUEST_UUID, REQUEST_FLAGS),
            RESPONSE: _chrc(RESPONSE_UUID, RESPONSE_FLAGS),
        }
        self._bus.export(APP, _ObjectManager(objects))
        self._bus.export(SERVICE, service)
        self._bus.export(REQUEST, self._request)
        self._bus.export(RESPONSE, self._response)
        self._bus.export(ADVERT, _Advertisement(self._name))

    async def _register(self):
        adapter = await self._adapter()
        for key, value in (('Powered', Variant('b', True)), ('Alias', Variant('s', self._name)),
                           ('Pairable', Variant('b', False))):  # nobody bonds
            await self._bluez(adapter, 'org.freedesktop.DBus.Properties', 'Set', 'ssv',
                              ['org.bluez.Adapter1', key, value])
        await self._bluez(adapter, 'org.bluez.GattManager1', 'RegisterApplication', 'oa{sv}', [APP, {}])
        await self._bluez(adapter, 'org.bluez.LEAdvertisingManager1', 'RegisterAdvertisement',
                          'oa{sv}', [ADVERT, {}])

    async def _adapter(self):
        objects = (await self._bluez('/', 'org.freedesktop.DBus.ObjectManager',
                                     'GetManagedObjects', '', []))[0]
        path = next((p for p in sorted(objects) if 'org.bluez.GattManager1' in objects[p]), None)
        if path is None:
            raise RuntimeError('no BLE adapter')
        return path

    def _on_signal(self, msg):
        if msg.member != 'PropertiesChanged' or msg.body[0] != 'org.bluez.Device1':
            return
        connected = msg.body[1].get('Connected')
        if connected is not None and not connected.value:
            self._replies.pop(msg.path, None)
            self._on_disconnect(msg.path)

    async def _bluez(self, path, interface, member, signature, body):
        return await self._call('org.bluez', path, interface, member, signature, body)

    async def _call(self, destination, path, interface, member, signature, body):
        reply = await self._bus.call(Message(destination=destination, path=path, interface=interface,
                                             member=member, signature=signature, body=body))
        if reply.message_type == MessageType.ERROR:
            if reply.error_name == 'org.bluez.Error.AlreadyExists':
                return []  # still registered from before a transient failure
            raise RuntimeError(f'{member}: {reply.error_name} {reply.body}')
        return reply.body
