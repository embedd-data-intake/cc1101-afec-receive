# Receiver firmware (node 1)

This describes the firmware after the serial command layer was added (run plan P0.1, 2026-11-27). The firmware is an ESP32 + CC1101 node that only listens. The library's `Node::poll()` decodes frames, ACKs or NACKs them, adapts the ACK level and relays. The firmware checks each delivered payload (CRC32 of the synthetic payload, logged as `app_rx`) and accepts serial commands to switch the protocol configuration at run time. Everything about the protocol lives in the `cc1101-afec` library, which the command layer does not change.

## Files

| File | Role |
| --- | --- |
| `platformio.ini` | `env:esp32dev` pulls the library from GitHub (unpinned). `env:dev` builds against `../cc1101-afec` through a symlink. Both build with `-D AFEC_LOG_LEVEL=4 -D AFEC_LOG_FORMAT=1 -D AFEC_LOG_SINK=1` (serial). `env:dev-mqtt` is `dev` with `AFEC_LOG_SINK=2`. No sensor libraries. |
| `src/config.h` | Pins, node IDs, boot radio settings |
| `src/main.cpp` | `setup()`, the `onData` payload check, the command handlers; `loop()` polls the node and the serial line |
| `src/afec_cmd.h` | Command layer, synthetic payload and CRC32. **Identical copy in the sender**, where `docs/firmware.md` documents it in full. |
| `src/afec_platform_arduino.h` | `afec::Platform` glue. Identical copy in the sender. |
| `src/afec_log_sink.h` | Log sink: serial, MQTT or off (`AFEC_LOG_SINK`). Identical copy in the sender. |
| `src/secrets.example.h` | Template for the gitignored `src/secrets.h` (WiFi + broker), only for MQTT builds |

## Configuration

| Constant | Value | Notes |
| --- | --- | --- |
| `PIN_SCK` / `PIN_MISO` / `PIN_MOSI` / `PIN_CSN` | 19 / 18 / 23 / 27 | Same wiring as the sender |
| `PIN_GDO0` / `PIN_GDO2` | 26 / 16 | GDO0 is the interrupt. GDO2 is unused. |
| `NODE_ID` / `PEER_NODE_ID` | 1 / 2 | `PEER_NODE_ID` is the default target of `reset` and of the `resetLink` after `cfg` |
| `RF_FREQ_MHZ` / `RF_SYNC_WORD` / `RF_TX_DBM` | 433.92 / `0x6B1D` / 0 | Must match the sender. `RF_TX_DBM` (ACK and NACK power) is only the boot value; `cfg tx=` changes it. |
| `USE_IRQ` | true | |

At boot the protocol configuration is the library default, **C5**.

**The receiver's mode must match the sender's**, because the receiver decides:
- whether to send NACKs (C4/C5 only);
- what feedback goes into the ACK (the score in C5, quantised RSSI in C4, 0 otherwise);
- whether its ACK level adapts.

So the orchestrator sends the same `cfg` to both nodes before each block. For C4 the `rssi_*`/`hys` policy matters on the receiver (it computes the quantised RSSI it feeds back) **and** on the sender (it applies the thresholds), so set it on both.

## Serial commands

Same syntax and reply format as the sender (see the sender's `docs/firmware.md` for the full `cfg` key list, the events and the re-initialisation notes).

| Command | Effect |
| --- | --- |
| `hello` | Logs `hello` (`role` = `receiver`, `peer`, `build`) and `cfg_ok`. Also logged at boot. |
| `cfg …` | Same keys as on the sender. Re-runs `radio.begin()` + `node.begin()`, `resetLink(2)`, then `cfg_ok`. `mode=3` is rejected. |
| `mark run=<id> block=<n> cfg=<Cx>` | Logs `mark`, so the receiver log can be split by block too |
| `reset [peer=<id>]` | `resetLink(peer)`, default 2 |
| `stats` | Cumulative `afec::Stats` as `stats` |

There is no `run`, `stop` or `bench` here.

## Runtime behaviour

- **`setup()`**: serial at 115200, `radio.begin()` (halts if the chip isn't found), `enableInterrupt()`, `node.begin()`, the `onData` callback, then `hello` and `cfg_ok`.
- **`loop()`**: `node.poll()`, then one complete command line if there is one.
- **Per received DATA frame for node 1, inside the library:**
  1. Decode and score it.
  2. Step the `hop` and `ack` levels (C4/C5).
  3. Send an ACK if requested, including for duplicates.
  4. Call `onData` only if it isn't a duplicate.
- **Per corrupted frame (C4/C5):** a broadcast NACK at level 5, at most one every 150 ms.
- **`onData`** logs one `app_rx` per delivered payload: `src`, `len`, `ok` (1 = CRC32 matches), `aseq`, `block`, `idx` (read from the payload; untrustworthy when `ok=0`) and `fb` (the feedback the receiver computed for that frame). A payload shorter than 12 B, such as a legacy sensor payload, logs `ok=0`. The ACK has already been sent before the callback runs, so the log line doesn't delay it.

## Serial output

This section applies to the serial sink (`AFEC_LOG_SINK=1`, the default).


- **Serial TX buffer: 8 KB** (`Serial.setTxBufferSize(8192)` before `begin()`). With the default buffer every log line blocked ~15 ms on the UART, which inflated RTT unevenly across configurations (2026-11-27 hardware test: C5 109 ms vs C2 53 ms; with the buffer 37 vs 34 ms, the gap being the ACK airtime). DBG logging averages about 1 KB/s, well under the UART's 11.5 KB/s.
- JSON Lines from the library and the firmware. Filter on lines starting with `{`. The library schema is in `cc1101-afec/docs/afec_log_schema.md`.
- Plain text: the boot lines only. (The hex/ASCII payload dump was replaced by `app_rx`.)
- `debugPrint` uses the same 192 B buffer as the sender. The longest observed library line is 175 chars (`rx`/`lvl_step`).

## Log sink (serial / MQTT / off)

Same as the sender, through the identical `src/afec_log_sink.h`: `-D AFEC_LOG_SINK=0|1|2|3` (off, serial, MQTT, both), `env:dev-mqtt` for MQTT, credentials in the gitignored `src/secrets.h`. The receiver publishes to **`afec/<run_id>/rx/log`**. See "Log sink" in `cc1101-afec-sender/docs/firmware.md` for the details (core 0 publisher, `log_drop`, `log_stats`, footprint).

## Build, flash, capture

```sh
pio run -e dev
pio run -e dev -t upload --upload-port /dev/cu.usbserial-0001   # ask before flashing
pio device monitor -b 115200 -p /dev/cu.usbserial-0001
```

On the current bench Mac, node 1 is on `/dev/cu.usbserial-0001`. Both boards are CP2102 with the same serial number, so check with `hello`. Capture and parse with `cc1101-afec-logs-parser/parse.py` (see the parent repo).

If PlatformIO fails to download packages behind the TLS-intercepting proxy, set `REQUESTS_CA_BUNDLE` and `SSL_CERT_FILE` to `~/.config/certs/macos-bundle.pem`.
