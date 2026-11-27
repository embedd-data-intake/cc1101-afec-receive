# CLAUDE.md

This is the receiver firmware (node 1) for the AFEC thesis. It runs on ESP32 + CC1101, listens through `afec::Node::poll()`, which ACKs or NACKs and relays, and prints delivered payloads. It is a submodule of the thesis repo `afec`. Read `docs/firmware.md` first. For protocol internals and the log schema, see `../cc1101-afec/docs/`.

## Status: frozen

The firmware is considered finished. **Do not refactor or add tests.** Only change it for a real bug found during measurement, or for something the user asks for. The planned serial command layer and the CRC32 `app_rx` check (`../docs/07-run-plan.md`, P0) are one such request, once approved.

## Layout

- `src/config.h`: pins, `NODE_ID` 1, radio settings. `TX_INTERVAL_MS` is unused here.
- `src/main.cpp`: setup, an `onData` callback that prints hex and ASCII, and a loop that only polls. `NodeConfig` uses the library defaults (mode C5).
- `src/afec_platform_arduino.h`: `afec::Platform` glue. Keep it identical to the sender's copy.
- `platformio.ini`: `esp32dev` uses the GitHub library; `dev` uses the `../cc1101-afec` symlink. Both use `AFEC_LOG_LEVEL=4` and `AFEC_LOG_FORMAT=1`, which the log parser needs.

## Commands

```sh
pio run -e dev                                   # build (allowed without asking)
pio run -e dev -t upload --upload-port <port>    # flashing touches hardware: ask first
```

- The node 1 port on the bench Mac is `/dev/cu.usbserial-0001`. Confirm it by the `node` field in the serial log.
- If PlatformIO fails to download packages, pass `REQUESTS_CA_BUNDLE=~/.config/certs/macos-bundle.pem SSL_CERT_FILE=~/.config/certs/macos-bundle.pem` (TLS-intercepting proxy).

## Conventions

- **The receiver's mode must match the sender's.** The receiver decides on NACKs, the feedback content and its own ACK level. Keep the radio settings identical too.
- **Firmware output that isn't JSON** must not start with `{`. The parser keeps only lines that start with `{`.
- **Committing:** this is its own repo. Commit here first, then commit the updated submodule pointer in the parent `afec` repo. When firmware changes, record the commit in the run's `meta.json` and update `docs/` here and `../docs/06-status-and-open-items.md`.
