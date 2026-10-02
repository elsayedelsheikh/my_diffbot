#!/usr/bin/env bash
# Re-activate the WiFi profile when NetworkManager has given up on it. A single 4-way handshake
# timeout on the congested 2.4 GHz channel fails the profile with 'no-secrets' (the headless
# Jetson has no secret agent to re-ask for the PSK), and NM then blocks autoconnect for it until
# it is brought up by hand. Run as root from wifi-watchdog.timer.
set -euo pipefail
IFACE=${WIFI_IFACE:-wlan0}
CON=${WIFI_CON:-Shadow Protocol}

state=$(nmcli -g GENERAL.STATE device show "$IFACE")
# Leave anything but a settled 'disconnected' alone: NM may be mid-activation or on a fallback.
[[ $state == 30\ * ]] || exit 0

ssid=$(nmcli -g 802-11-wireless.ssid connection show "$CON")
if ! nmcli -g SSID device wifi list ifname "$IFACE" --rescan auto | grep -Fxq "$ssid"; then
  echo "$IFACE disconnected, '$ssid' not in range"
  exit 0
fi

echo "$IFACE disconnected with '$ssid' in range, activating '$CON'"
nmcli -w 45 connection up "$CON" ifname "$IFACE"
