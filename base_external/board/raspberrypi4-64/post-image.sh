#!/bin/bash
set -e

BOARD_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
BOARD_NAME="$(basename "${BOARD_DIR}")"
GENIMAGE_CFG="${BOARD_DIR}/genimage-${BOARD_NAME}.cfg"
GENIMAGE_TMP="${BUILD_DIR}/genimage.tmp"

# --- Custom: build MCP3221 overlay + update firmware config.txt ---
FW_DIR="${BINARIES_DIR}/rpi-firmware"
OVERLAYS_DIR="${FW_DIR}/overlays"
CONFIG_FILE="${FW_DIR}/config.txt"
DTC="${HOST_DIR}/bin/dtc"
SRC="${BOARD_DIR}/dt-overlay/mcp3221-4d.dtso"
OUT="${OVERLAYS_DIR}/mcp3221-4d.dtbo"
MARKER="# --- assignment: enable i2c/spi/uart + mcp3221 overlay ---"

if [ -d "${FW_DIR}" ]; then
    mkdir -p "${OVERLAYS_DIR}"

    if [ -x "${DTC}" ] && [ -f "${SRC}" ]; then
        "${DTC}" -@ -I dts -O dtb -o "${OUT}" "${SRC}"
        echo "[post-image] Wrote overlay: ${OUT}"
    else
        echo "[post-image] Skipping overlay build (missing dtc or ${SRC})"
    fi

    # Ensure config.txt exists
    [ -f "${CONFIG_FILE}" ] || : > "${CONFIG_FILE}"

    # Append our block once
    if ! grep -Fq "${MARKER}" "${CONFIG_FILE}"; then
        cat >> "${CONFIG_FILE}" <<EOF

${MARKER}
dtparam=i2c_arm=on
dtparam=i2c_arm_baudrate=100000
dtparam=spi=on
enable_uart=1
dtoverlay=mcp3221-4d
EOF
        echo "[post-image] Appended custom lines to ${CONFIG_FILE}"
    else
        echo "[post-image] Custom config already present; skipping append"
    fi
else
    echo "[post-image] ${FW_DIR} not present; skipping firmware modifications"
fi
# --- End custom section ---

# Pass an empty rootpath. genimage makes a full copy of the given rootpath to
# ${GENIMAGE_TMP}/root so passing TARGET_DIR would be a waste of time and disk
# space. We don't rely on genimage to build the rootfs image, just to insert a
# pre-built one in the disk image.

trap 'rm -rf "${ROOTPATH_TMP}"' EXIT
ROOTPATH_TMP="$(mktemp -d)"

rm -rf "${GENIMAGE_TMP}"

genimage \
        --rootpath "${ROOTPATH_TMP}"   \
        --tmppath "${GENIMAGE_TMP}"    \
        --inputpath "${BINARIES_DIR}"  \
        --outputpath "${BINARIES_DIR}" \
        --config "${GENIMAGE_CFG}"

exit $?
