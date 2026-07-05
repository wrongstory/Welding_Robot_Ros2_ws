#!/usr/bin/env bash
# Install the iAHRS IMU udev rule so the device always appears as /dev/IMU.
#
# Binds by USB vendor/product + CP2102 serial + physical USB port path (KERNELS),
# because the CP2102 serial "0001" is a shared factory default and is not unique
# on its own. The port path makes the match unique even against another CP210x.
#
# Usage:
#   ./install_udev.sh                 # auto-detect serial + port path from the device
#   ./install_udev.sh 0001 3-7.4      # force a specific serial and port path
set -euo pipefail

RULE_NAME="99-iahrs-imu.rules"
DEST="/etc/udev/rules.d/${RULE_NAME}"
VENDOR="10c4"
PRODUCT="ea60"

SERIAL="${1:-}"
KERNELS="${2:-}"

# --- Auto-detect serial + port path from the connected device ----------------
if [[ -z "${SERIAL}" || -z "${KERNELS}" ]]; then
  for dev in /dev/ttyUSB* /dev/ttyACM*; do
    [[ -e "${dev}" ]] || continue
    info="$(udevadm info -a -n "${dev}" 2>/dev/null || true)"
    if grep -q "ATTRS{idVendor}==\"${VENDOR}\"" <<<"${info}" \
       && grep -q "ATTRS{idProduct}==\"${PRODUCT}\"" <<<"${info}"; then
      [[ -z "${SERIAL}" ]] && SERIAL="$(grep -m1 -oP 'ATTRS\{serial\}=="\K[^"]+' <<<"${info}" || true)"
      # KERNELS of the USB device node = the block that also carries idVendor/idProduct.
      # It is the last "KERNELS==" line before that idVendor line in the walk output.
      if [[ -z "${KERNELS}" ]]; then
        # Walk the udevadm output top-down: remember the most recent KERNELS==,
        # and print it once we hit the block carrying this device's idVendor.
        # (Portable across mawk/gawk -- no 3-arg match / GNU extensions.)
        kline=""
        while IFS= read -r line; do
          case "${line}" in
            *KERNELS==\"*\"*)
              kline="${line#*KERNELS==\"}"; kline="${kline%%\"*}" ;;
            *ATTRS\{idVendor\}==\"${VENDOR}\"*)
              KERNELS="${kline}"; break ;;
          esac
        done <<<"${info}"
      fi
      echo "Detected iAHRS on ${dev}: serial=${SERIAL:-<none>} port-path=${KERNELS:-<none>}"
      break
    fi
  done
fi

if [[ -z "${SERIAL}" || -z "${KERNELS}" ]]; then
  echo "ERROR: could not auto-detect serial and/or port path for ${VENDOR}:${PRODUCT}." >&2
  echo "       Connect the IMU, or pass them explicitly: $0 <serial> <port-path>" >&2
  exit 1
fi

# --- Write the rule ----------------------------------------------------------
RULE="SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"${VENDOR}\", ATTRS{idProduct}==\"${PRODUCT}\", ATTRS{serial}==\"${SERIAL}\", KERNELS==\"${KERNELS}\", SYMLINK+=\"IMU\", MODE=\"0666\", GROUP=\"dialout\""

echo "Installing rule -> ${DEST}"
echo "  ${RULE}"
sudo tee "${DEST}" >/dev/null <<EOF
# WITHROBOT iAHRS IMU -> stable symlink /dev/IMU  (installed by install_udev.sh)
# serial=${SERIAL} (CP2102 default, non-unique) + USB port path KERNELS=${KERNELS} for uniqueness.
${RULE}
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Done. Verify with:  ls -la /dev/IMU"
