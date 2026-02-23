# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Satellite1 ESPHome firmware for the FutureProofHomes Core Board — an ESP32-S3 based AI voice assistant and multisensor device. The ESP32-S3 handles WiFi/BLE/voice while communicating with an XMOS coprocessor via SPI for audio DSP (echo cancellation, etc.). Integrates with Home Assistant.

**Target**: ESPHome 2026.2.1 on ESP32-S3 with PSRAM (octal, 80MHz), 16MB flash.

## Build Commands

```bash
# First-time setup: creates .venv/ and installs dependencies
source scripts/setup_build_env.sh

# Compile firmware (main config)
esphome compile config/satellite1.yaml

# Upload to device via USB
esphome upload config/satellite1.yaml

# View device logs
esphome logs config/satellite1.yaml

# Build variant configs (mmWave radar options)
esphome compile config/satellite1.ld2410.yaml
esphome compile config/satellite1.ld2450.yaml
```

## Linting and Formatting

Pre-commit hooks enforce formatting. Run manually:

```bash
pre-commit run --all-files
```

- **C++ files** (`esphome/**/*.{cpp,h,c}`): clang-format v18, LLVM-based style, 120 char column limit, 2-space indent
- **YAML files** (`config/**/*.yaml`): yamllint, 2-space indent, max 1 empty line, no line length limit

## Architecture

### Two-Processor System

ESP32-S3 (this firmware) ↔ SPI ↔ XMOS coprocessor (separate firmware, flashed via `memory_flasher` component). The `satellite1` component manages the SPI protocol to XMOS. On boot, it checks XMOS firmware version and can flash embedded images automatically.

### Custom ESPHome Components (`esphome/components/`)

Each component follows the ESPHome external component pattern:
- `__init__.py` — Python config schema and `async def to_code(config)` code generation
- `.h` / `.cpp` — C++ implementation extending ESPHome base classes (`Component`, `SPIDevice`, `I2CDevice`, etc.)

Key components:
- **satellite1** — Core SPI interface to XMOS, firmware version checking, status triggers
- **i2s_audio** — I2S microphone input and speaker output (48kHz native, duplex secondary mode)
- **micro_wake_word** — On-device wake word detection (Hey Jarvis, Okay Nabu, etc.)
- **memory_flasher** — Downloads and flashes XMOS firmware, MD5 validation
- **fusb302b** — USB-C Power Delivery negotiation over I2C
- **tas2780** — Speaker amplifier/DAC, PD-aware power selection
- **pcm5122** — Line-out DAC with gain control
- **mixer** / **resampler** — Audio stream mixing and sample rate conversion
- **snapcast** — Network audio streaming

### YAML Configuration (`config/`)

Modular package-based system using `!include`:
- `satellite1.yaml` — Main entry point, package orchestration, OTA/BLE setup
- `satellite1.base.yaml` — Base config, globals, XMOS version management
- `config/common/` — Feature packages (voice_assistant, media_player, led_ring, buttons, etc.)

Hardware variant configs (`satellite1.ld2410.yaml`, `satellite1.ld2450.yaml`) add mmWave radar support.

### Hardware Pin Mapping (defined in `config/common/core_board.yaml`)

- I2C: GPIO5 (SDA), GPIO6 (SCL) @ 400kHz
- SPI: SPI2 — GPIO11 (CLK), GPIO12 (MOSI), GPIO13 (MISO)
- I2S: Duplex secondary mode, 48kHz

## Git Workflow

- **develop** — Active development (default branch, PR target)
- **staging** — Beta releases (tagged with `-beta` suffix)
- **main** — Production releases

CI runs clang-format and yamllint checks on PRs to `develop`. Build workflows produce firmware artifacts on push to `develop`/`staging`/`main`.

## Testing

Component tests live in `tests/components/` as YAML configs that exercise component configurations via ESPHome's compilation-based testing:

```bash
esphome compile tests/components/fusb302b/test_power_delivery.yaml
```
