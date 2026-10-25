#!/usr/bin/env bash
#
# EZModbus — ESP-IDF smoke test on a board
# ----------------------------------------
# Builds ci/smoke against THIS working copy with the chosen ESP-IDF version (official
# espressif/idf container), flashes it and reads the console until the app prints
# its verdict. Needs a Linux host with Docker and the board wired as in
# ci/smoke/main/main.cpp (two UARTs cross-connected).
#
# Usage:
#   ci/smoke.sh -p <port> [-i <idf>] [-t <target>] [--debug]
#     -p  board serial port (e.g. /dev/serial/by-id/usb-Espressif_USB_JTAG_...)
#     -i  espressif/idf image tag (default: release-v5.5), e.g. release-v5.4, v5.5.1, v6.1
#     -t  chip target (default: esp32s3)
#     --debug  run the EZMODBUS_DEBUG build
#
# Exit code: 0 pass, 1 fail / crash / timeout, 2 usage or build error.
# Env: SMOKE_TIMEOUT (s, default 90) for the run itself.

set -u
set -o pipefail

IDF="release-v5.5"
TARGET="esp32s3"
PORT=""
BUILD="build"

while [[ $# -gt 0 ]]; do
    case "$1" in
        -p|-i|-t)
            if [[ $# -lt 2 ]]; then echo "ERROR: $1 needs a value" >&2; exit 2; fi
            case "$1" in -p) PORT="$2" ;; -i) IDF="$2" ;; -t) TARGET="$2" ;; esac
            shift 2 ;;
        --debug) BUILD="build-debug"; shift ;;
        *) echo "Unknown argument: $1" >&2; exit 2 ;;
    esac
done
if [[ -z "$PORT" || ! -e "$PORT" ]]; then
    echo "ERROR: serial port missing or not found (-p)" >&2
    exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
DEV="$(readlink -f "$PORT")"
WORK="ci/.work/idf-$IDF"
LOG="ci/.logs/smoke-$IDF-$TARGET${BUILD#build}.log"
mkdir -p "$REPO_ROOT/ci/.logs" "$REPO_ROOT/ci/.work/ccache"

echo ">> Smoke test: IDF $IDF, $TARGET${BUILD#build}, port $DEV"
# Runs as root like the CI container (IDF writes under /opt/esp), files handed back on exit
docker run --rm --device "$DEV" \
    -e CCACHE_DIR=/project/ci/.work/ccache \
    -e WORK_DIR="/project/$WORK" -e LOG_DIR="/project/$WORK/logs" \
    -v "$REPO_ROOT:/project" -w /project \
    "espressif/idf:$IDF" \
    bash -c '
        set -e
        trap "chown -R $6 /project/ci/.work" EXIT
        echo ">> Building"
        bash ci/build_examples.sh "$1" smoke >/dev/null || { tail -n 40 "$2"/logs/"$1"-smoke*.log; exit 2; }
        echo ">> Flashing"
        cd "$2/$1/smoke/$3"
        python -m esptool --chip "$1" -p "$4" -b 921600 --after no_reset write_flash @flash_args >/dev/null || exit 2
        python /project/ci/smoke_monitor.py "$4" "$5"
    ' _ "$TARGET" "/project/$WORK" "$BUILD" "$DEV" "${SMOKE_TIMEOUT:-90}" "$(id -u):$(id -g)" \
    2>&1 | sed -n '/^>> Building/,$p' | tee "$REPO_ROOT/$LOG"
STATUS=${PIPESTATUS[0]}
echo ">> Log: $LOG"
exit "$STATUS"
