# CI — ESP-IDF example build check

`build_examples.sh` compiles every app under `examples/esp-idf` against the
**local working copy** of the library (not the published registry version), across
one or more chip targets. Compile-only — nothing is flashed or run. It catches
divergence between the Arduino/PlatformIO build path and the ESP-IDF build path,
and target-specific breakage (Xtensa vs RISC-V, missing peripherals).

It runs the same way locally and in CI: the GitHub workflow (added later) is a thin
wrapper that runs this script inside the official `espressif/idf` container, so
what passes locally passes in CI.

> Host unit tests are not here — run them directly with `pio test -e test_codec`.

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

## Notes

- Locally the script sources `~/esp/v5.5.1/esp-idf/export.sh` if `idf.py` isn't
  already on PATH (override with `IDF_EXPORT`). In the CI container `idf.py` is
  already exported, so nothing is sourced.
- `ccache` is used automatically when installed — big speedup across the many
  example projects (the IDF core objects are identical and get cached).
- `ci/.work/` and `ci/.logs/` are gitignored and safe to delete anytime.
- On-device loopback tests (the PlatformIO `test_*_loopback` envs) need a real
  ESP32-S3 + RS485 wiring and are **not** part of this rig — run them manually.
