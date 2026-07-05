#!/usr/bin/env bash
# Install the wired-joystick udev rule so the controller always appears as
# /dev/input/joystick_wired (the path joy_linux and joystick_gui expect).
#
# Matches by USB vendor/product only -- there is a single joystick on the robot,
# so VID:PID is a sufficient (and port-independent) key. Unlike the IMU rule,
# no serial / USB port path is needed for uniqueness.
#
# Usage:
#   ./install_udev.sh                 # auto-detect VID:PID from the connected /dev/input/jsN
#   ./install_udev.sh 045e 0b12       # force a specific vendor:product
set -euo pipefail

RULE_NAME="99-joystick-wired.rules"
DEST="/etc/udev/rules.d/${RULE_NAME}"

VENDOR="${1:-}"
PRODUCT="${2:-}"

# --- Auto-detect VID:PID from the connected joystick device ------------------
if [[ -z "${VENDOR}" || -z "${PRODUCT}" ]]; then
  for dev in /dev/input/js*; do
    [[ -e "${dev}" ]] || continue
    info="$(udevadm info -a -n "${dev}" 2>/dev/null || true)"
    v="$(grep -m1 -oP 'ATTRS\{idVendor\}=="\K[^"]+' <<<"${info}" || true)"
    p="$(grep -m1 -oP 'ATTRS\{idProduct\}=="\K[^"]+' <<<"${info}" || true)"
    name="$(grep -m1 -oP 'ATTRS\{name\}=="\K[^"]+' <<<"${info}" || true)"
    if [[ -n "${v}" && -n "${p}" ]]; then
      VENDOR="${v}"; PRODUCT="${p}"
      echo "Detected joystick on ${dev}: ${VENDOR}:${PRODUCT} (${name:-unknown})"
      break
    fi
  done
fi

if [[ -z "${VENDOR}" || -z "${PRODUCT}" ]]; then
  echo "ERROR: could not auto-detect a joystick VID:PID." >&2
  echo "       Connect the wired controller, or pass them: $0 <vendor> <product>" >&2
  exit 1
fi

# --- Write the rule ----------------------------------------------------------
# SYMLINK+= is relative to /dev/, so list BOTH "joystick_wired" (=> /dev/joystick_wired)
# and "input/joystick_wired" (=> /dev/input/joystick_wired, the path joy_linux reads).
RULE="SUBSYSTEM==\"input\", ATTRS{idVendor}==\"${VENDOR}\", ATTRS{idProduct}==\"${PRODUCT}\", KERNEL==\"js[0-9]*\", SYMLINK+=\"joystick_wired input/joystick_wired\", MODE=\"0666\", GROUP=\"input\""

echo "Installing rule -> ${DEST}"
echo "  ${RULE}"
sudo tee "${DEST}" >/dev/null <<EOF
# Wired game controller -> stable symlink /dev/input/joystick_wired  (installed by install_udev.sh)
# Matches USB ${VENDOR}:${PRODUCT}. SYMLINK+= is /dev/-relative, hence "input/joystick_wired".
${RULE}
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Done. Verify with:  ls -la /dev/input/joystick_wired"
