#!/usr/bin/env bash
# PlantPi installer for Raspberry Pi OS (Bookworm or newer) on a Raspberry Pi 4.
# Run from the unzipped project folder:   sudo ./deploy/setup.sh
# Safe to run again; it only adds what is missing.
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then echo "Please run with sudo:  sudo ./deploy/setup.sh"; exit 1; fi
SRC="$(cd "$(dirname "$0")/.." && pwd)"
MODEL="$(tr -d '\0' </proc/device-tree/model 2>/dev/null || echo unknown)"
echo "==> Device: $MODEL"
case "$MODEL" in *"Raspberry Pi 4"*) ;; *) echo "    WARNING: this is not a Raspberry Pi 4 - pins may differ.";; esac

echo "==> 1/6 Installing packages"
apt-get update -q
apt-get install -y -q python3-serial python3-gpiozero python3-lgpio sqlite3

# v2 needs no changes in config.txt: no SPI, no DHT22 overlay. The Pico sensor node is a USB serial device (/dev/ttyACM0).

echo "==> 2/6 Service user 'plantpi'"
getent group plantpi >/dev/null || groupadd --system plantpi
id plantpi >/dev/null 2>&1 || useradd --system --gid plantpi --home-dir /var/lib/plantpi \
    --shell /usr/sbin/nologin plantpi
usermod -aG gpio,dialout plantpi
if [ -n "${SUDO_USER:-}" ] && [ "$SUDO_USER" != "root" ]; then
  usermod -aG plantpi,gpio,dialout "$SUDO_USER"
  echo "    added $SUDO_USER to groups plantpi, gpio, dialout (log out and in again)"
fi

echo "==> 3/6 Program files -> /opt/plantpi"
rm -rf /opt/plantpi/plantpi
install -d /opt/plantpi
cp -r "$SRC/plantpi" /opt/plantpi/plantpi
cp "$SRC/config.example.toml" /opt/plantpi/
find /opt/plantpi -name '__pycache__' -prune -exec rm -rf {} +

echo "==> 4/6 Config and data folders"
install -d -m 755 /etc/plantpi
if [ ! -f /etc/plantpi/config.toml ]; then
  install -m 644 "$SRC/config.example.toml" /etc/plantpi/config.toml
  echo "    created /etc/plantpi/config.toml"
elif grep -q '^spi_bus' /etc/plantpi/config.toml; then
  mv /etc/plantpi/config.toml /etc/plantpi/config.v1.toml
  install -m 644 "$SRC/config.example.toml" /etc/plantpi/config.toml
  echo "    old v1 config moved to config.v1.toml, new v2 config created"
else
  echo "    kept existing /etc/plantpi/config.toml"
fi
install -d -o plantpi -g plantpi -m 2775 /var/lib/plantpi
if [ -f /var/lib/plantpi/plantpi.db ] && ! sqlite3 /var/lib/plantpi/plantpi.db "PRAGMA table_info(samples)" | grep -q node_ok; then
  mv /var/lib/plantpi/plantpi.db "/var/lib/plantpi/plantpi.v1.db"
  rm -f /var/lib/plantpi/calibration.json
  echo "    v1 log moved to plantpi.v1.db; old MCP3008 calibration removed (recalibrate)"
fi

echo "==> 5/6 Command 'plantpi'"
cat > /usr/local/bin/plantpi <<'EOF'
#!/bin/sh
PYTHONPATH=/opt/plantpi exec /usr/bin/python3 -m plantpi "$@"
EOF
chmod 755 /usr/local/bin/plantpi

echo "==> 6/6 systemd service + hardware watchdog"
install -m 644 "$SRC/deploy/plantpi.service" /etc/systemd/system/plantpi.service
install -d /etc/systemd/system.conf.d
install -m 644 "$SRC/deploy/watchdog.conf" /etc/systemd/system.conf.d/plantpi-watchdog.conf
systemctl daemon-reload
echo "    installed, NOT enabled yet - enable it after wiring, tests and calibration (README, step 4)"

echo
echo "Done."
echo "Next:  plug the Pico (MicroPython + main.py) into a Pi USB port, then:   ls /dev/ttyACM*   and   plantpi test node"
