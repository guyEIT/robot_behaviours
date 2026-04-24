#!/usr/bin/env bash
# Installs the Hamilton STAR udev rule. Run once, requires sudo.
set -euo pipefail
SRC="$(cd "$(dirname "$0")" && pwd)/99-hamilton.rules"
DEST="/etc/udev/rules.d/99-hamilton.rules"

sudo install -m 0644 "$SRC" "$DEST"
sudo udevadm control --reload-rules
sudo udevadm trigger

if ! id -nG "$USER" | tr ' ' '\n' | grep -qx plugdev; then
  echo "Adding $USER to the plugdev group (log out/in to take effect)."
  sudo usermod -aG plugdev "$USER"
fi

echo "Done. Replug the STAR; it should appear as /dev/hamilton_star."
