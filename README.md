# Satellite1 ESPHome Firmware (flashingcursor fork)

Experimental ESPHome firmware for the [FutureProofHomes Satellite1](https://futureproofhomes.net/) Core Board -- an ESP32-S3 based AI voice assistant and multisensor that integrates with Home Assistant.

This is a fork of [FutureProofHomes/Satellite1-ESPHome](https://github.com/FutureProofHomes/Satellite1-ESPHome). We're using their excellent hardware and firmware as a starting point, then experimenting with reliability improvements and UX enhancements. If our changes prove stable and there's community interest, we may contribute them back upstream.

## What's Different in This Fork

### Connectivity UX

The stock firmware goes silent when Home Assistant or WiFi disconnects -- wake word detection stops, no audio feedback, the user has no idea what happened. We taught it some manners:

- **Always-listening mode** -- Wake word detection stays active during HA outages. Say the wake word and the device tells you what's wrong instead of ignoring you.
- **Spoken status updates** -- "I'm back online" when HA reconnects, "WiFi reconnected" when WiFi restores. Configurable as voice clips or sci-fi tones.
- **Proactive WiFi guidance** -- First-time setup prompt on boot, reconnect instructions after WiFi drops, contextual help when you say the wake word while disconnected.
- **Auto-reconnect retry** -- If you say the wake word while HA is down and it comes back within 30 seconds, the voice pipeline starts automatically.

All features are controllable via Home Assistant config entities:

| Entity | Type | Default | Description |
|--------|------|---------|-------------|
| Always listen | Switch | On | Keep wake word active during HA disconnects |
| Announce connections | Switch | On | Play sounds on connect/disconnect |
| Announcement style | Select | Voice | Choose between voice clips and tones |

### Reliability Hardening

Several rounds of bug fixes across the component codebase, covering critical audio pipeline issues, power delivery negotiation, network resilience, and general correctness.

## Hardware

You need a [FutureProofHomes Core Board](https://futureproofhomes.net/products/satellite1-core-board) (ESP32-S3 with PSRAM, 16MB flash). The [HAT board](https://futureproofhomes.net/products/satellite1-top-microphone-board) is strongly recommended to unlock microphone array, wake word, sensors, and LED ring features.

## Building

Set up the build environment (creates a `.venv/` with ESPHome and dependencies):

```bash
source scripts/setup_build_env.sh
```

Compile, upload, and monitor:

```bash
esphome compile config/satellite1.yaml
esphome upload config/satellite1.yaml
esphome logs config/satellite1.yaml
```

Hardware variant configs for mmWave radar:

```bash
esphome compile config/satellite1.ld2410.yaml
esphome compile config/satellite1.ld2450.yaml
```

### Linting

Pre-commit hooks enforce formatting. Run manually:

```bash
pre-commit run --all-files
```

## Architecture

The Satellite1 is a two-processor system: the ESP32-S3 (this firmware) handles WiFi, BLE, voice assistant integration, and sensor reading, while an XMOS coprocessor handles audio DSP (echo cancellation, beamforming). They communicate over SPI. On boot, the ESP32 checks the XMOS firmware version and can flash updated firmware automatically.

The firmware is built as a set of custom ESPHome components (`esphome/components/`) with modular YAML configuration (`config/common/`). Key components include `satellite1` (SPI protocol to XMOS), `i2s_audio` (48kHz audio I/O), `micro_wake_word` (on-device wake word), `fusb302b` (USB-C PD negotiation), `tas2780`/`pcm5122` (speaker/line-out DACs), and `mixer`/`resampler` (audio stream processing).

See [CLAUDE.md](CLAUDE.md) for detailed architectural notes.

## Relationship to Upstream

This is an independent experimental fork. The original project by [FutureProofHomes](https://github.com/FutureProofHomes) is where you should go for official releases, documentation, and support:

- **Official firmware**: [FutureProofHomes/Satellite1-ESPHome](https://github.com/FutureProofHomes/Satellite1-ESPHome)
- **Documentation**: [docs.futureproofhomes.net](https://docs.futureproofhomes.net)
- **Hardware**: [futureproofhomes.net](https://futureproofhomes.net/)
- **YouTube**: [youtube.com/@FutureProofHomes](https://www.youtube.com/@futureproofhomes)

We are not affiliated with FutureProofHomes. Buy their hardware -- it's great.

## License

Same license as upstream. See [LICENSE](LICENSE) for details.
