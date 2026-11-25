#!/usr/bin/env bash
# Build and install the SoapySDR driver for the ELAD FDM-S2 receiver.
#
#   ./setup.sh                   print the commands only
#   ./setup.sh --install         run them (uses sudo unless run as root)
#   ./setup.sh --install --server
#                                also enable the SoapyRemote server (port 55132)
#
# Run it on the computer the FDM-S2 is plugged into (Linux only). The driver is
# built from this checkout. The receiver's FPGA images are
# ELAD's and are not in this repository: they are taken out of ELAD's loader
# program, downloaded from eladit.com, and checked against known hashes.
# Always ends by running SoapySDRUtil --find="driver=fdms2"; exits non-zero
# when no device is listed. Works unchanged on amd64 and arm64 (Debian/Ubuntu).
set -euo pipefail

INSTALL=0
SERVER=0
while [ $# -gt 0 ]; do
    case "$1" in
        --install) INSTALL=1 ;;
        --server) SERVER=1 ;;
        -h|--help) sed -n '2,14p' "$0"; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
    shift
done

SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CACHE_DIR="${XDG_CACHE_HOME:-$HOME/.cache}/soapy-fdms2"
BUILD_DIR="$CACHE_DIR/build"
FW_DIR="$CACHE_DIR/firmware"
FW_INSTALL_DIR=/usr/local/share/soapy-fdms2
LOADER_URL="https://www.eladit.com/wp-content/uploads/elad-download/sdr/Linux/elad-firmware-2.0-intel"
LOADER_SHA=be204f58beaad0c9b8f2a44e3c5217f4adf9f6123c587792a4e8f9f4e28f7d75
UDEV_RULE=/etc/udev/rules.d/70-elad-fdms2.rules

SUDO=sudo
if [ "$(id -u)" -eq 0 ]; then SUDO=; fi

run() {
    echo "+ $*"
    if [ "$INSTALL" -eq 1 ]; then "$@"; fi
}

# name, offset in the loader file, size, sha256
IMAGES="
fdms2-192k.bit 814816 802294 d7e443b9802dac0ac756716d22564586e1caa471596c2b74748afa04a4c6a581
fdms2-384k.bit 2419424 802294 0474d5815f93c2f42b8a81e4146fffe664d44f42488ee4701ae21b8f57fc25e4
fdms2-768k.bit 4024032 802018 355ede47ac7dfd198becdbc1ab1c9018b86473e6549acb65ca117183ef7b04ba
fdms2-1536k.bit 5628384 802570 df6ff3fe595992888102b7ea5c6fbbff42b90d488954ce6d37f6bcd17a4b4402
fdms2-3072k.bit 7233280 801462 1cef009b521c5ad9e947b10b9c042ebdcc245b6dece3466eed22fd76bea17bcf
"

# Fetch ELAD's loader and cut the FPGA images out of it; a hash mismatch stops the script.
extract_images() {
    mkdir -p "$FW_DIR"
    curl -fsSL --retry 3 -o "$FW_DIR/elad-loader" "$LOADER_URL"
    echo "$LOADER_SHA  $FW_DIR/elad-loader" | sha256sum -c --quiet -
    while read -r name offset size sha; do
        [ -n "$name" ] || continue
        head -c "$((offset + size))" "$FW_DIR/elad-loader" | tail -c "$size" > "$FW_DIR/$name"
        echo "$sha  $FW_DIR/$name" | sha256sum -c --quiet -
    done <<<"$IMAGES"
    rm -f "$FW_DIR/elad-loader"
}

run $SUDO apt-get install -y libsoapysdr-dev soapysdr-tools libusb-1.0-0-dev cmake g++ curl
if [ "$SERVER" -eq 1 ]; then
    run $SUDO apt-get install -y soapyremote-server
fi

run cmake -S "$SRC_DIR" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release
run cmake --build "$BUILD_DIR" -j"$(nproc)"
run "$BUILD_DIR/fdms2_protocol_test"
run $SUDO cmake --install "$BUILD_DIR"
run $SUDO ldconfig

echo "+ fetch the FPGA images from $LOADER_URL into $FW_DIR"
if [ "$INSTALL" -eq 1 ]; then extract_images; fi
run $SUDO install -d "$FW_INSTALL_DIR"
while read -r name _; do
    [ -n "$name" ] || continue
    run $SUDO install -m 0644 "$FW_DIR/$name" "$FW_INSTALL_DIR/$name"
done <<<"$IMAGES"

# Lets the plugdev group open the receiver; the SoapyRemote server runs as root and does not need it.
echo "+ write $UDEV_RULE"
if [ "$INSTALL" -eq 1 ]; then
    printf '%s\n' '# ELAD FDM-S2 receiver: let the plugdev group open it' \
        'SUBSYSTEM=="usb", ATTRS{idVendor}=="1721", ATTRS{idProduct}=="061c", MODE="0660", GROUP="plugdev"' \
        | $SUDO tee "$UDEV_RULE" >/dev/null
fi
run $SUDO udevadm control --reload-rules
run $SUDO udevadm trigger --attr-match=idVendor=1721

if [ "$SERVER" -eq 1 ]; then
    # The soapyremote-server package ships its own unit, listening on every
    # interface, port 55132. Restart it so it loads the new module.
    run $SUDO systemctl enable soapyremote-server.service
    run $SUDO systemctl restart soapyremote-server.service
fi

echo "+ SoapySDRUtil --find=\"driver=fdms2\""
if ! OUT="$(SoapySDRUtil --find="driver=fdms2" 2>&1)"; then
    echo "$OUT"; echo "SoapySDRUtil failed" >&2; exit 1
fi
echo "$OUT"
if ! grep -q "driver *= *fdms2" <<<"$OUT"; then
    echo "No FDM-S2 found (is it plugged in, is the driver installed, and may this user open the USB device?)" >&2
    exit 1
fi
