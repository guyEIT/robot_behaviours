#!/usr/bin/env bash
# Install Intel RealSense udev rules so non-root users (in the plugdev group)
# can open the camera over libusb, and create stable /dev/realsense_video*
# symlinks for the V4L2 streams.
#
# Fixes the librealsense error you see in `pixi run meca500-real-up` logs:
#   ERROR (handle-libusb.h): failed to open usb interface: 0,
#                            error: RS2_USB_STATUS_ACCESS
#
# Idempotent: re-running just overwrites /etc/udev/rules.d/99-realsense.rules
# and reloads udev. Safe to run as many times as you like.
#
# Usage:
#   ./scripts/install-realsense-udev.sh           # uses sudo
#   sudo ./scripts/install-realsense-udev.sh      # already root
#
# After it finishes, restart `pixi run meca500-real-up` — no replug needed.

set -euo pipefail

RULES_PATH="/etc/udev/rules.d/99-realsense.rules"

if [[ $EUID -ne 0 ]]; then
  exec sudo -E "$0" "$@"
fi

if ! getent group plugdev >/dev/null; then
  echo "error: 'plugdev' group does not exist on this system." >&2
  echo "create it (or edit this script to use the 'video' group) and re-run." >&2
  exit 1
fi

INVOKING_USER="${SUDO_USER:-$USER}"
if ! id -nG "$INVOKING_USER" | tr ' ' '\n' | grep -qx plugdev; then
  echo "warning: user '$INVOKING_USER' is not in the plugdev group."
  echo "         the rules will install, but '$INVOKING_USER' won't benefit"
  echo "         until they're added: sudo usermod -aG plugdev $INVOKING_USER"
  echo "         (then log out and back in)."
fi

echo "writing $RULES_PATH ..."
cat > "$RULES_PATH" <<'EOF'
# Intel RealSense — grant libusb access to plugdev group + symlink V4L2 video nodes.
# Installed by scripts/install-realsense-udev.sh (robot_behaviours repo).
# Covers D4xx / D5xx series. Add product IDs here if you swap cameras.

SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad1", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad2", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad3", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad4", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad5", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0af2", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0af6", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0afe", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0aff", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b00", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b01", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b03", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b07", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b0c", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b0d", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b3a", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b3d", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b48", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b49", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b4b", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b4d", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b5b", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b5c", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b64", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b68", MODE:="0666", GROUP:="plugdev"

# V4L2 video device symlinks for D435 (extend ATTRS{idProduct} for other models).
# Stream order from librealsense is consistent: depth, depth IR, depth IR2,
# colour, plus 2 metadata nodes — symlinks /dev/realsense_video0..5.
KERNEL=="video[0-9]*", SUBSYSTEM=="video4linux", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b07", SYMLINK+="realsense_video%n", MODE:="0666", GROUP:="plugdev"

# IIO nodes (IMU on D435i, etc.)
KERNEL=="iio*", SUBSYSTEM=="iio", ATTRS{idVendor}=="8086", MODE:="0666", GROUP:="plugdev"
EOF

chmod 0644 "$RULES_PATH"

echo "reloading udev rules ..."
udevadm control --reload-rules
udevadm trigger

echo
echo "done. checking results:"
echo
if ls -l /dev/realsense_video* 2>/dev/null; then
  :
else
  echo "  (no /dev/realsense_video* symlinks yet — try unplugging and"
  echo "   replugging the camera once.)"
fi
echo
RS_USB_LINE=$(lsusb 2>/dev/null | awk 'tolower($0) ~ /intel.*realsense/ {print; exit}' || true)
if [[ -n "$RS_USB_LINE" ]]; then
  RS_BUS=$(echo "$RS_USB_LINE" | awk '{print $2}')
  RS_DEV=$(echo "$RS_USB_LINE" | awk '{gsub(":","",$4); print $4}')
  RS_NODE="/dev/bus/usb/${RS_BUS}/${RS_DEV}"
  if [[ -e "$RS_NODE" ]]; then
    ls -l "$RS_NODE"
  fi
fi
echo
echo "now restart \`pixi run meca500-real-up\` — RS2_USB_STATUS_ACCESS should be gone."
