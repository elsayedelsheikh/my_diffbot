# merlin_bluetooth

Bluetooth (BLE) provisioning link between the Merlin Android app and the robot. Over it the phone

- finds the robot with nothing configured (no IP, no shared network),
- reads the robot's IP, camera viewer password and ports,
- moves the robot's Wi-Fi: onto the robot's own hotspot, the phone's hotspot or any Wi-Fi.

Video and teleop still run over Wi-Fi; Bluetooth only gets the two onto the same network.

## Run

```bash
echo MERLIN_BT_PIN=12345678 >> .env      # 6-12 digits, 8+ recommended; the app asks for it once
docker compose up -d --build bluetooth
```

The container talks to the **host's** bluetoothd (system D-Bus) and NetworkManager (the host's own
`nmcli`/`iw` through `nsenter`, so client and daemon versions always match). It needs `privileged`,
`pid: host` and `/var/run/dbus`. Parameters: `name` (BLE name, hotspot SSID prefix; default `Merlin`),
`camera_config` (`/etc/merlin/mediamtx.yml`), `whep_port` (8889), `bridge_port` (8766),
`hotspot_timeout` (300 s). It also publishes `/merlin/uptime` (`std_msgs/UInt32`, seconds, every 5 s)
for the app's status bar.

## Design

| Layer | Choice | Why |
|---|---|---|
| Transport | BLE GATT, one service, **no bonding** | No Android pairing dialog, nothing stored in the OS, works on any BlueZ. |
| Authentication | SRP-6a (RFC 5054, 2048-bit group, SHA-256) on the robot PIN | The PIN never crosses the air; a sniffer or a fake robot gets nothing to brute-force offline, only live guesses. Same scheme as HomeKit setup codes and ESP-IDF provisioning "Security 2". |
| Guess limit | 5 wrong PINs, then a lockout that doubles (60 s ... 1 h), shared by all connections | Online guessing is the only attack SRP leaves. |
| Confidentiality | AES-256-GCM, one key per direction (HKDF-SHA256 of the SRP key), message counter as nonce | Replayed, reordered or altered messages fail to decrypt. |

Link-layer security (LE Secure Connections) is deliberately not relied on: the Jetson Nano's L4T 4.9
kernel only pairs with LE *legacy* just-works (recoverable by a passive sniffer), and BlueZ 5.53
segfaults when a bonded phone reconnects. The protocol is secure without it, on any kernel.

Two more BlueZ 5.53 workarounds, harmless elsewhere:

- Requests are written **without response**: 5.53 loses the D-Bus reply to an acknowledged write and
  fails it after 5 s. The link layer still acknowledges every packet.
- Replies are **read**, not notified: notifications reach every subscribed phone, a read is answered
  per device. The notification only says "your reply is ready".

## GATT

| | UUID | Properties |
|---|---|---|
| Service | `7f1d0000-5c2a-4b8e-9d3f-6a1e2b4c8d01` | advertised, with the robot name |
| Request | `7f1d0001-...` | write without response |
| Response | `7f1d0002-...` | read, notify |

One exchange: the phone writes one request (UTF-8 JSON object + `\n`, at most 512 bytes; ask for an
ATT MTU of 517 first), waits for a notification on Response (a 2-byte counter, no content), then
reads Response. The robot keeps one reply per connected phone, so requests are strictly one at a time.

## Messages (protocol 2)

Every reply has `ok`; a failure is `{"id":…, "ok":false, "error":"<code>", "msg":"<text>"}`.

**1. Pair** (plaintext; base64 for binary):

```
-> {"id":1,"op":"pair_start"}
<- {"id":1,"ok":true,"proto":2,"salt":"<16 B>","B":"<256 B>"}
-> {"id":2,"op":"pair_verify","A":"<256 B>","M1":"<32 B>"}
<- {"id":2,"ok":true,"M2":"<32 B>","name":"Merlin"}        # or error bad_pin / locked
```

SRP-6a with `I = "merlin"`, `P = PIN`, `N`,`g` = RFC 5054 2048-bit group, `H` = SHA-256, all integers
padded to 256 bytes: `k = H(N | PAD(g))`, `x = H(salt | H(I ":" P))`, `u = H(PAD(A) | PAD(B))`,
`K = H(PAD(S))`, `M1 = H(PAD(A) | PAD(B) | K)`, `M2 = H(PAD(A) | M1 | K)`. The phone must check `M2`
(the robot proves the PIN too). Session keys: `HKDF-SHA256(K, salt="merlin-ble-2", info)` with
`info = "phone->robot"` and `"robot->phone"`, 32 bytes each.

**2. Sealed requests.** Everything after pairing travels as

```
{"n": <counter>, "c": "<base64 AES-256-GCM(key, nonce = n as 12-byte big-endian, JSON)>"}
```

with `n` starting at 1 and strictly increasing per direction. Inside:

| Request | Reply fields |
|---|---|
| `{"id","op":"status"}` | `name`, `uptime_s`, `wifi:{mode: station\|hotspot\|disconnected, ssid, ip}`, `camera:{user, password, port}`, `bridge_port` |
| `{"id","op":"join_wifi","ssid","password"}` | `mode`, `ssid`, `ip`. Empty `password` uses the profile saved on the robot. Errors: `not_found`, `auth_failed`, `failed` (the robot returns to its previous network) |
| `{"id","op":"start_hotspot"}` | `mode`, `ssid`, `password`, `ip`. WPA2, created once with a random password and kept in the NetworkManager profile `merlin-hotspot` |

Other errors: `unauthorized` (not paired), `bad_request`, `unknown_op`.

The hotspot never strands the robot: if no client joins within `hotspot_timeout` it returns to the
network it came from, and the profile has `autoconnect=no`, so a reboot always lands on the home Wi-Fi.

## Tests

```bash
python3 -m pytest merlin_bluetooth/test     # RFC 5054 vectors, handshake, replay, lockout, sizes
(cd merlin_android && ./gradlew testDebugUnitTest)   # same vectors + a cross-language vector
```

`test_cross_language_vector` and `SrpTest.matchesTheRobot` pin identical bytes on both sides.
