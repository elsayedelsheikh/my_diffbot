"""wlan0 control through the HOST's nmcli (nsenter into PID 1's mount namespace).

The container's own nmcli (1.46) is newer than the Jetson's NetworkManager (1.22);
the host binary always matches its daemon. Needs `pid: host` + privileged.
Every function returns (fields, None) on success or (None, (code, text)) on failure.
"""

import asyncio
import re
import secrets
import string

IFACE = 'wlan0'
HOTSPOT = 'merlin-hotspot'  # NM profile; it stores the AP's SSID and password itself


async def nmcli(*args, timeout=45):
    """(returncode, combined output) of the host nmcli."""
    proc = await asyncio.create_subprocess_exec(
        'nsenter', '-t', '1', '-m', '--', 'nmcli', *args,
        stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT,
    )
    try:
        out, _ = await asyncio.wait_for(proc.communicate(), timeout)
    except asyncio.TimeoutError:
        proc.kill()
        return 1, 'nmcli timed out'
    return proc.returncode, out.decode(errors='replace').strip()


async def _get(*args):
    rc, out = await nmcli('-t', '-g', *args)
    return out if rc == 0 else ''


async def active():
    """(connection name, mode 'station'|'hotspot'|'', ip) of wlan0."""
    name = await _get('GENERAL.CONNECTION', 'device', 'show', IFACE)
    if not name:
        return '', '', ''
    ip = (await _get('IP4.ADDRESS', 'device', 'show', IFACE)).split('/')[0].split('|')[0]
    mode = await _get('802-11-wireless.mode', 'connection', 'show', name)
    return name, 'hotspot' if mode == 'ap' else 'station', ip


async def status():
    name, mode, ip = await active()
    ssid = await _get('802-11-wireless.ssid', 'connection', 'show', name) if name else ''
    return {'mode': mode or 'disconnected', 'ssid': ssid, 'ip': ip}


def failure(out):
    """Map nmcli's failure text to a protocol error."""
    if re.search(r'No network with SSID', out):
        return 'not_found', 'network not in range'
    if re.search(r'[Ss]ecrets were required|802-1X supplicant|wrong password', out):
        return 'auth_failed', 'wrong Wi-Fi password'
    return 'failed', out.splitlines()[-1] if out else 'nmcli failed'


async def _saved(ssid):
    """Name of a saved station profile for `ssid`, or ''."""
    rc, out = await nmcli('-t', '-f', 'NAME,TYPE', 'connection', 'show')
    for line in out.splitlines() if rc == 0 else ():
        name, _, kind = line.rpartition(':')
        name = name.replace('\\:', ':')
        if kind == '802-11-wireless' and name != HOTSPOT:
            if await _get('802-11-wireless.ssid', 'connection', 'show', name) == ssid:
                return name
    return ''


async def join(ssid, password):
    """Join `ssid` (saved profile when no password); on failure go back to the old network."""
    previous, mode, _ = await active()
    if previous and mode == 'station' and await _saved(ssid) == previous:
        return await status(), None
    if mode == 'hotspot':
        await nmcli('connection', 'down', previous)
        await asyncio.sleep(3)  # an AP radio can't scan; let it settle as a station
    profile = await _saved(ssid)
    if profile and not password:
        rc, out = await nmcli('connection', 'up', profile)
    elif password:
        await nmcli('device', 'wifi', 'rescan', 'ifname', IFACE)
        await asyncio.sleep(3)
        rc, out = await nmcli('device', 'wifi', 'connect', ssid, 'password', password, 'ifname', IFACE)
        if rc and not profile:
            await nmcli('connection', 'delete', ssid)  # don't keep a profile with a bad password
    else:
        rc, out = 1, 'No saved profile; a password is required'
    if rc:
        if previous:
            await nmcli('connection', 'up', previous)
        return None, ('auth_failed', 'password required') if 'password is required' in out else failure(out)
    return await status(), None


async def start_hotspot(name):
    """Bring up the robot's own AP; created once with a random password, reused after."""
    name_, mode, _ = await active()
    if mode != 'hotspot':
        if not await _get('connection.id', 'connection', 'show', HOTSPOT):
            password = ''.join(secrets.choice(string.ascii_letters + string.digits) for _ in range(12))
            rc, out = await nmcli(
                'connection', 'add', 'type', 'wifi', 'ifname', IFACE, 'con-name', HOTSPOT,
                'autoconnect', 'no',  # a reboot always comes back on the home network
                'ssid', f'{name}-hotspot', '802-11-wireless.mode', 'ap', '802-11-wireless.band', 'bg',
                'ipv4.method', 'shared', 'wifi-sec.key-mgmt', 'wpa-psk', 'wifi-sec.proto', 'rsn',
                'wifi-sec.pairwise', 'ccmp', 'wifi-sec.group', 'ccmp', 'wifi-sec.psk', password,
            )
            if rc:
                return None, failure(out)
        rc, out = await nmcli('connection', 'up', HOTSPOT)
        if rc:
            if name_:
                await nmcli('connection', 'up', name_)
            return None, failure(out)
    rc, out = await nmcli('-s', '-t', '-g', '802-11-wireless.ssid,802-11-wireless-security.psk',
                          'connection', 'show', HOTSPOT)
    ssid, _, password = out.partition('\n')
    return {**await status(), 'ssid': ssid, 'password': password}, None


async def station_count():
    proc = await asyncio.create_subprocess_exec(
        'nsenter', '-t', '1', '-m', '-n', '--', 'iw', 'dev', IFACE, 'station', 'dump',
        stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.DEVNULL,
    )
    out, _ = await proc.communicate()
    return out.decode().count('Station ')
