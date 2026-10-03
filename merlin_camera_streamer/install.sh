#!/bin/bash
# Install the camera streamer on the Jetson HOST (not in the ros:jazzy container:
# Argus and NVENC are L4T GStreamer plugins that only exist on the host's JetPack).
#   sudo ./install.sh
set -euo pipefail
cd "$(dirname "$0")"

MEDIAMTX_VERSION=v1.21.1

apt-get update && apt-get install -y --no-install-recommends gstreamer1.0-rtsp  # rtspclientsink

curl -fsSL "https://github.com/bluenviron/mediamtx/releases/download/${MEDIAMTX_VERSION}/mediamtx_${MEDIAMTX_VERSION}_linux_arm64.tar.gz" \
  | tar -xz -C /usr/local/bin mediamtx

# Viewer password (user "merlin"): MERLIN_CAMERA_PASS, else the installed one, else a new random one.
#   sudo MERLIN_CAMERA_PASS=<pass> ./install.sh
CONF=/etc/merlin/mediamtx.yml
PASS=${MERLIN_CAMERA_PASS:-$(sed -n 's/^    pass: //p' "$CONF" 2>/dev/null | head -1)}
PASS=${PASS:-$(head -c 12 /dev/urandom | base64 | tr -dc 'A-Za-z0-9')}
if [[ ! $PASS =~ ^[A-Za-z0-9._-]{8,}$ ]]; then
  echo "MERLIN_CAMERA_PASS: 8+ characters from A-Z a-z 0-9 . _ -" >&2
  exit 1
fi
install -d /etc/merlin
sed "s/MERLIN_CAMERA_PASS/$PASS/" config/mediamtx.yml > "$CONF"
chmod 600 "$CONF"
echo "Camera viewer: user merlin, password $PASS"
install -Dm644 systemd/merlin-camera.service /etc/systemd/system/merlin-camera.service
# Build root's GStreamer plugin cache now; otherwise the first camera start spends ~12 s
# scanning plugins, close to runOnDemandStartTimeout.
gst-inspect-1.0 >/dev/null 2>&1 || true

systemctl daemon-reload
systemctl enable --now merlin-camera.service
systemctl restart merlin-camera.service  # pick up a changed config on re-install
