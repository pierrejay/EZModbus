# CI — ESP-IDF build check & smoke test

Three layers, from cheapest to most realistic:

| What | Where | Runs |
|---|---|---|
| `build_examples.sh`: every `examples/esp-idf` app + the `smoke` app (normal & `EZMODBUS_DEBUG`), compile-only | GitHub CI (IDF × target matrix), or locally | Nothing flashed |
| Host codec suite, plain & under ASan/UBSan (`pio test -e native -e native-san`) | GitHub CI, or locally | On the host |
| `smoke.sh`: the `smoke` app flashed and run on a board, for a chosen IDF version | Locally (bench with Docker) | On the chip |

The PlatformIO Unity suites (`pio test -e s3` / `-e c6`, Arduino core) stay the
reference: they test EZModbus against independent peers. The smoke app wires
EZModbus to itself, so it catches what changes between IDF versions and chips
(drivers, lwIP, FreeRTOS settings), not a protocol mistake shared by both ends.

`build_examples.sh` compiles against the **local working copy** of the library (not
the published registry version). It catches divergence between the
Arduino/PlatformIO build path and the ESP-IDF build path, target-specific breakage
(Xtensa vs RISC-V, missing peripherals) and, through the smoke app, template code
(`read<T>` / `write<T>` for every type, Bridge, multi-interface Server) and the
debug paths. It runs the same way locally and in CI: the GitHub workflow runs it
inside the official `espressif/idf` container.

## Usage

```bash
# All examples, default target (esp32)
ci/build_examples.sh

# Full target matrix
ci/build_examples.sh esp32 esp32c6 esp32s3 esp32p4

# Targets + example filter
ci/build_examples.sh esp32s3 tcp-server rtu-client

# Force clean builds
CLEAN=1 ci/build_examples.sh esp32
```

Exits non-zero if any example fails. Per-example logs land in `ci/.logs/`.

## Smoke test on a board

`ci/smoke/` is a self-checking app: two TCP clients (127.0.0.1, no Wi-Fi needed), a
bridge, an RTU link over two cross-wired UARTs and a multi-interface server. It
covers every function code, the longest frames, exceptions, client & bridge
timeouts followed by a recovery, a slave ID change, async requests, a concurrent
burst on both interfaces and `read<T>` / `write<T>` for every type, then prints
`SMOKE PASS` or `SMOKE FAIL` (about 10 s).

```bash
# On a Linux host with Docker and the board plugged in
ci/smoke.sh -p /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_XX-if00 -i release-v5.4 -t esp32s3
ci/smoke.sh -p <port> -i v6.1 -t esp32c6 --debug
```

The image (`espressif/idf:<tag>`) is pulled on first use. Build, flash and console
all run inside it (no local IDF or esptool needed); builds are kept per IDF version
under `ci/.work/idf-<tag>/`, the console log lands in `ci/.logs/`.

Wiring (same jumpers as the PlatformIO suites, pin map in `ci/smoke/main/main.cpp`):

- **ESP32-S3** (XIAO): UART1 RX 6 / TX 43 ↔ UART2 RX 44 / TX 7 (43→44, 7→6)
- **ESP32-C6**: UART1 RX 22 / TX 21 ↔ LP-UART RX 4 / TX 5 (21→4, 5→22)
- **ESP32**: UART1 RX 18 / TX 19 ↔ UART2 RX 16 / TX 17 (19→16, 17→18)

The console must not share those pins: S3 & C6 use the USB-Serial-JTAG port.

## How the library override works

Each example declares a managed dependency `pierrejay/ezmodbus`. The script copies
the example into `ci/.work/<target>/<name>/` and drops a `components/ezmodbus`
symlink pointing at the repo root. A project component overrides a managed
dependency of the same short name, so the registry version is never fetched — the
local sources are compiled.

## Per-example target rules

Not every example fits every chip; the matrix skips the impossible combos honestly:

- **Wi-Fi examples** (`tcp-client`, `tcp-server`, `bridge`) are skipped on targets
  without native Wi-Fi (`esp32h2`, `esp32p4`).
- **Dual-UART example** (`rtu-client-server-loopback`) needs a 3rd UART, so it only
  builds on `esp32`, `esp32s3`, `esp32p4`.
- **Smoke app** builds on the targets with a pin map: `esp32`, `esp32s3`, `esp32c6`,
  `esp32p4` (TCP over 127.0.0.1, so no Wi-Fi needed).

## Notes

- Locally the script sources `~/esp/v5.5.1/esp-idf/export.sh` if `idf.py` isn't
  already on PATH (override with `IDF_EXPORT`). In the CI container `idf.py` is
  already exported, so nothing is sourced.
- `ccache` is used automatically when installed — big speedup across the many
  example projects (the IDF core objects are identical and get cached).
- `ci/.work/` and `ci/.logs/` are gitignored and safe to delete anytime.
- The PlatformIO Unity suites need the boards and run manually (`pio test -e s3`).
