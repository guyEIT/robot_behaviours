#!/usr/bin/env bash
# Installs a udev rule for the USB-serial adapter wired to the Liconic
# STX44 PLC. Gives members of the `plugdev` group read/write access and
# exposes the port at a stable path (default: /dev/liconic_stx44), so
# scripts don't have to chase /dev/ttyUSB0 vs /dev/ttyUSB1.
#
# By default the script auto-detects a single Prolific PL2303 adapter
# (VID 067b, PID 23a3) on the system. If multiple matching adapters are
# plugged in, pass --serial to pick one. Requires sudo.
set -euo pipefail

SYMLINK="liconic_stx44"
SERIAL=""
VID="067b"
PID="23a3"

usage() {
  cat <<EOF
Usage: $0 [--serial <SERIAL>] [--symlink <NAME>] [--vid <VID>] [--pid <PID>]

  --serial    USB device serial number (default: auto-detect; required if
              multiple matching adapters are attached)
  --symlink   /dev/ symlink to create (default: liconic_stx44)
  --vid       USB vendor id  (default: 067b — Prolific Technology)
  --pid       USB product id (default: 23a3 — PL2303)
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --serial)  SERIAL="$2"; shift 2 ;;
    --symlink) SYMLINK="$2"; shift 2 ;;
    --vid)     VID="$2"; shift 2 ;;
    --pid)     PID="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown arg: $1" >&2; usage; exit 2 ;;
  esac
done

if [[ -z "$SERIAL" ]]; then
  mapfile -t CANDIDATES < <(
    shopt -s nullglob
    for dev in /dev/ttyUSB* /dev/ttyACM*; do
      info=$(udevadm info -q property -n "$dev" 2>/dev/null || true)
      v=$(grep -m1 '^ID_VENDOR_ID='   <<<"$info" | cut -d= -f2)
      p=$(grep -m1 '^ID_MODEL_ID='    <<<"$info" | cut -d= -f2)
      s=$(grep -m1 '^ID_SERIAL_SHORT=' <<<"$info" | cut -d= -f2)
      if [[ "$v" == "$VID" && "$p" == "$PID" && -n "$s" ]]; then
        echo "$dev $s"
      fi
    done
  )
  case ${#CANDIDATES[@]} in
    0)
      echo "no matching USB-serial adapter ($VID:$PID) found." >&2
      echo "Plug in the Liconic cable, or override with --vid / --pid / --serial." >&2
      exit 1
      ;;
    1)
      DEV=${CANDIDATES[0]% *}
      SERIAL=${CANDIDATES[0]#* }
      echo "Detected $DEV (serial $SERIAL)"
      ;;
    *)
      echo "multiple matching adapters — pass --serial to disambiguate:" >&2
      printf '  %s\n' "${CANDIDATES[@]}" >&2
      exit 1
      ;;
  esac
fi

DEST="/etc/udev/rules.d/99-liconic.rules"
echo "Installing $DEST (symlink /dev/${SYMLINK}, serial ${SERIAL})"

sudo tee "$DEST" >/dev/null <<EOF
# Liconic STX44 — USB-serial adapter ${VID}:${PID} serial ${SERIAL}.
# Gives members of plugdev read/write access and exposes the port as
# /dev/${SYMLINK}. Regenerate via install-liconic-udev.sh if the adapter
# is replaced.
SUBSYSTEM=="tty", ATTRS{idVendor}=="${VID}", ATTRS{idProduct}=="${PID}", ATTRS{serial}=="${SERIAL}", \\
  MODE="0660", GROUP="plugdev", SYMLINK+="${SYMLINK}"
EOF
sudo chmod 0644 "$DEST"

sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty

if ! id -nG "$USER" | tr ' ' '\n' | grep -qx plugdev; then
  echo "Adding $USER to the plugdev group (log out/in to take effect)."
  sudo usermod -aG plugdev "$USER"
fi

echo "Done. Replug the USB-serial cable; it should appear as /dev/${SYMLINK}."
