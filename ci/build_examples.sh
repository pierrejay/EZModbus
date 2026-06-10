#!/usr/bin/env bash
#
# EZModbus — ESP-IDF example compile check
# ----------------------------------------
# Builds every example under examples/esp-idf against THIS working copy of the
# library (not the published registry version), across one or more chip targets.
# Compile-only: nothing is flashed or run. Catches divergence between the
# Arduino/PlatformIO build path and the ESP-IDF build path, and target-specific
# breakage (Xtensa vs RISC-V, missing peripherals).
#
# Runs identically:
#   - locally  (sources ESP-IDF from IDF_EXPORT if idf.py isn't already on PATH)
#   - in CI    (inside the espressif/idf container, where idf.py is pre-exported)
#
# How the local override works:
#   Each example declares a managed dependency `pierrejay/ezmodbus`. We copy the
#   example into a work dir and drop a `components/ezmodbus` symlink pointing at
#   the repo root. A project component named `ezmodbus` overrides a managed
#   dependency of the same short name, so the registry version is never fetched.
#
# Usage:
#   ci/build_examples.sh                                  # all examples, esp32
#   ci/build_examples.sh esp32 esp32c6 esp32s3 esp32p4   # matrix over targets
#   ci/build_examples.sh esp32s3 tcp-server rtu-client   # targets + example filter
#   CLEAN=1 ci/build_examples.sh esp32                   # wipe work dirs first
#
# Env overrides:
#   IDF_EXPORT  path to esp-idf export.sh used only if idf.py isn't on PATH
#               (default: ~/esp/v5.5.1/esp-idf/export.sh; ignored in CI)

set -u
set -o pipefail

# --- Locate repo root (script lives in <repo>/ci) --------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null)"
[[ -z "$REPO_ROOT" ]] && REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
EXAMPLES_DIR="$REPO_ROOT/examples/esp-idf"
WORK_DIR="$SCRIPT_DIR/.work"
LOG_DIR="$SCRIPT_DIR/.logs"
IDF_EXPORT="${IDF_EXPORT:-$HOME/esp/v5.5.1/esp-idf/export.sh}"

# --- Per-example target support rules --------------------------------------
# Wi-Fi examples: skip on targets without native Wi-Fi.
WIFI_EXAMPLES=" tcp-client tcp-server bridge "
NO_WIFI_TARGETS=" esp32h2 esp32p4 "
# Examples wiring two on-chip UARTs (use UART_NUM_2): only chips with a 3rd UART.
DUAL_UART_EXAMPLES=" rtu-client-server-loopback "
DUAL_UART_TARGETS=" esp32 esp32s3 esp32p4 "

KNOWN_TARGETS=" esp32 esp32s2 esp32s3 esp32c2 esp32c3 esp32c5 esp32c6 esp32c61 esp32h2 esp32p4 "

TARGETS=()
SELECTED=()
for arg in "$@"; do
    if [[ "$KNOWN_TARGETS" == *" $arg "* ]]; then
        TARGETS+=("$arg")
    else
        SELECTED+=("$arg")
    fi
done
[[ ${#TARGETS[@]} -eq 0 ]] && TARGETS=("esp32")

# --- Sanity checks ---------------------------------------------------------
if [[ ! -d "$EXAMPLES_DIR" ]]; then
    echo "ERROR: examples dir not found: $EXAMPLES_DIR" >&2
    exit 1
fi

# In CI (espressif/idf container) idf.py is already exported. Locally, source it.
if ! command -v idf.py >/dev/null 2>&1; then
    if [[ -f "$IDF_EXPORT" ]]; then
        echo ">> Sourcing ESP-IDF: $IDF_EXPORT"
        # shellcheck disable=SC1090
        source "$IDF_EXPORT" >/dev/null
    else
        echo "ERROR: idf.py not on PATH and export.sh not found at $IDF_EXPORT" >&2
        echo "       source <idf>/export.sh first, or set IDF_EXPORT." >&2
        exit 1
    fi
fi

# Use ccache if available (caches the identical IDF core objects across projects).
if command -v ccache >/dev/null 2>&1; then
    export IDF_CCACHE_ENABLE=1
    echo ">> ccache: enabled"
else
    echo ">> ccache: not installed ('brew install ccache' / apt install ccache to speed up)"
fi

echo ">> IDF: $(idf.py --version 2>/dev/null)"
echo ">> Targets: ${TARGETS[*]}"
echo ">> Library under test: $REPO_ROOT"
echo

mkdir -p "$WORK_DIR" "$LOG_DIR"

if [[ ${#SELECTED[@]} -gt 0 ]]; then
    EXAMPLES=("${SELECTED[@]}")
else
    EXAMPLES=()
    for d in "$EXAMPLES_DIR"/*/; do
        [[ -f "$d/CMakeLists.txt" ]] && EXAMPLES+=("$(basename "$d")")
    done
fi

# --- Build loop ------------------------------------------------------------
declare -a RESULTS=()
OVERALL=0

for target in "${TARGETS[@]}"; do
  for ex in "${EXAMPLES[@]}"; do
    src="$EXAMPLES_DIR/$ex"
    dst="$WORK_DIR/$target/$ex"
    log="$LOG_DIR/$target-$ex.log"

    if [[ ! -d "$src" ]]; then
        echo "!! [$target] skip '$ex': not found in $EXAMPLES_DIR"
        RESULTS+=("SKIP  $target/$ex (not found)")
        continue
    fi

    if [[ "$WIFI_EXAMPLES" == *" $ex "* && "$NO_WIFI_TARGETS" == *" $target "* ]]; then
        echo ">> [$target] skip '$ex' (no Wi-Fi on this target)"
        RESULTS+=("SKIP  $target/$ex (no wifi)")
        continue
    fi

    if [[ "$DUAL_UART_EXAMPLES" == *" $ex "* && "$DUAL_UART_TARGETS" != *" $target "* ]]; then
        echo ">> [$target] skip '$ex' (needs a 2nd general-purpose UART)"
        RESULTS+=("SKIP  $target/$ex (no UART2)")
        continue
    fi

    echo "=================================================================="
    echo ">> [$target] Building example: $ex"
    echo "=================================================================="

    [[ -n "${CLEAN:-}" ]] && rm -rf "$dst"
    mkdir -p "$dst"

    # Refresh example sources while preserving generated artifacts in the work dir
    # (build cache, fetched components, injected components/ override, sdkconfig).
    # Portable equivalent of "rsync --delete --exclude ..." using only coreutils,
    # since the espressif/idf container ships no rsync.
    find "$dst" -mindepth 1 -maxdepth 1 \
        ! -name build ! -name managed_components ! -name components \
        ! -name dependencies.lock ! -name 'sdkconfig*' \
        -exec rm -rf {} +
    cp -a "$src/." "$dst/"
    rm -f "$dst/.DS_Store"

    # Inject local library as an overriding project component.
    mkdir -p "$dst/components"
    ln -sfn "$REPO_ROOT" "$dst/components/ezmodbus"

    if idf.py -C "$dst" set-target "$target" build >"$log" 2>&1; then
        bin_sz=$(grep -Eo '\.bin binary size 0x[0-9a-f]+ bytes' "$log" | tail -1 | grep -Eo '0x[0-9a-f]+')
        echo "   PASS  (app size ${bin_sz:-?})"
        RESULTS+=("PASS  $target/$ex  app=${bin_sz:-?}")
    else
        echo "   FAIL  (see $log)"
        echo "   --- last 25 lines ---"
        tail -n 25 "$log" | sed 's/^/   | /'
        RESULTS+=("FAIL  $target/$ex  -> $log")
        OVERALL=1
    fi
    echo
  done
done

# --- Summary ---------------------------------------------------------------
echo "=================================================================="
echo "  ESP-IDF EXAMPLES SUMMARY (targets: ${TARGETS[*]})"
echo "=================================================================="
for r in "${RESULTS[@]}"; do
    echo "  $r"
done
echo "=================================================================="
[[ $OVERALL -eq 0 ]] && echo "  ALL GREEN" || echo "  FAILURES PRESENT"
exit $OVERALL
