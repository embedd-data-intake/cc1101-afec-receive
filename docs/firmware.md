# Receiver firmware (node 1)

This describes commit `ce4deef`. The firmware is an ESP32 + CC1101 node that only listens. The library's `Node::poll()` decodes frames, ACKs or NACKs them, adapts the ACK level and relays. The firmware prints each delivered payload. Everything about the protocol lives in the `cc1101-afec` library. See its `docs/`.

## Files

| File | Role |
| --- | --- |
| `platformio.ini` | `env:esp32dev` pulls the library from GitHub (unpinned). `env:dev` builds against `../cc1101-afec` through a symlink. Both build with `-D AFEC_LOG_LEVEL=4 -D AFEC_LOG_FORMAT=1`. No sensor libraries. |
| `src/config.h` | Pins, node ID, radio settings |
| `src/main.cpp` | `setup()`: radio, IRQ, `node.begin()`, `onData` callback. `loop()`: `node.poll()`. |
| `src/afec_platform_arduino.h` | `afec::Platform` glue. Identical copy in the sender. |

## Configuration (`config.h`)

| Constant | Value | Notes |
| --- | --- | --- |
| `PIN_SCK` / `PIN_MISO` / `PIN_MOSI` / `PIN_CSN` | 19 / 18 / 23 / 27 | Same wiring as the sender |
| `PIN_GDO0` / `PIN_GDO2` | 26 / 16 | GDO0 is the interrupt. GDO2 is unused. |
| `NODE_ID` | 1 | |
| `RF_FREQ_MHZ` / `RF_SYNC_WORD` / `RF_TX_DBM` | 433.92 / `0x6B1D` / 0 | Must match the sender. `RF_TX_DBM` sets the power of the ACKs and NACKs. |
| `USE_IRQ` | true | |
| `TX_INTERVAL_MS` | 2000 | Unused here (copied from the sender) |

**Protocol configuration:** `NodeConfig` sets only `id` and `bitrateBps = 4800`. Everything else is a library default, so the mode is **C5**.

**The receiver's mode must match the sender's**, because the receiver decides:
- whether to send NACKs (C4/C5 only);
- what feedback goes into the ACK (the score in C5, quantised RSSI in C4, 0 otherwise);
- whether its ACK level adapts.

Today, changing the mode means editing `main.cpp` and reflashing **both** nodes. The run plan's serial `cfg` command (`../docs/07-run-plan.md`, P0) is not implemented yet.

## Runtime behaviour

- **`setup()`**: serial at 115200, `radio.begin()` (halts if the chip isn't found), `enableInterrupt()`, `node.begin()`, then the `onData` callback.
- **`loop()`**: just `node.poll()`. The radio is parked in RX between frames.
- **Per received DATA frame for node 1, inside the library:**
  1. Decode and score it.
  2. Step the `hop` and `ack` levels (C4/C5).
  3. Send an ACK if requested, including for duplicates.
  4. Call `onData` only if it isn't a duplicate.
- **Per corrupted frame (C4/C5):** a broadcast NACK at level 5, at most one every 150 ms.
- **`onData`** prints one plain-text line per delivered payload: the hex bytes, `|`, then the bytes as ASCII. For example:
  `74 3A 32 … | t:2345;h:4567;p:0;m:0;`

  The line is not JSON, and the callback's `fb` argument is ignored. Serial printing takes a few ms per line, during which `poll()` does not run. The ACK has already been sent before the callback runs, so this doesn't delay the ACK.

## Serial output

- JSON Lines from the library. Filter on lines starting with `{`. The schema is in `cc1101-afec/docs/afec_log_schema.md`.
- Plain text: boot lines and the `onData` payload lines.
- **Delivery and correctness:** the logs show that a frame was delivered (`rx_data`, `dup=0`), but not whether the payload is correct. A payload check (CRC32 over a synthetic payload, logged as `app_rx`) is planned in run-plan P0.
- `debugPrint` uses the same 192 B buffer as the sender. The longest observed line is 175 chars (`rx`/`lvl_step`). A longer line is cut with no newline.

## Build, flash, capture

```sh
pio run -e dev
pio run -e dev -t upload --upload-port /dev/cu.usbserial-0001   # ask before flashing
pio device monitor -b 115200 -p /dev/cu.usbserial-0001
```

On the current bench Mac, node 1 is on `/dev/cu.usbserial-0001`. Both boards are CP2102 with the same serial number, so check the `node` field in the log. Capture and parse with `cc1101-afec-logs-parser/parse.py` (see the parent repo).

If PlatformIO fails to download packages behind the TLS-intercepting proxy, set `REQUESTS_CA_BUNDLE` and `SSL_CERT_FILE` to `~/.config/certs/macos-bundle.pem`.
