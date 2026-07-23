# ESPHome Virtual UART <-> RF433 Bridge (Nartis)

A component that **presents a UART** (`uart::UARTComponent`) but relays every
request/reply over a 433/443 MHz radio (CMT2300A) instead of over serial wires.
Any component that already knows how to talk to a meter over UART can bind to
this bridge - by pointing its `uart_id` at it - and works unchanged against an
RF-only meter. The RF hop is invisible: the upstream writes request bytes and
polls for a reply exactly as against a real serial link.

This mirrors the `esphome-uart-nordic` BLE bridge, which presents the same
virtual-UART interface over a BLE link.

Target: Nartis I100/I300 electricity meters ("rf-433-2" / d101-2 protocol) via a
CMT2300A transceiver on an ESP32.

## Status

- **Working:** virtual-UART interface, non-blocking bridge state machine, CMT2300A
  PHY (443.9 MHz, asymmetric TX/RX channel, chunked TX for frames > 64 B), the
  type-5A DLMS-HDLC envelope (build on TX, CRC-carve + unwrap on RX), CRC-16/X.25,
  and link-layer ARQ.
- **Fixed for now:** the operating frequency is hard-set to **443.9 MHz**. The
  serial-derived frequency is computed and logged but not yet programmed (the
  arbitrary-frequency register encoding is TODO).

## How it works

```
   upstream UART consumer                   CMT2300A RF
  +---------------------+                 +---------------------+             +--------+
  |  DLMS/COSEM reader  | -write(HDLC)->  |  pack RF frame      | ->  TX ->   | NARTIS |
  | (esphome-dlms-cosem)|                 |                     |   over air  | meter  |
  |  bound via uart_id  | <-read(HDLC) <--|  unpack frame + CRC | <-  RX <-   |        |
  +---------------------+                 +---------------------+             +--------+
```

1. **Collect** - the upstream writes a request (a complete DLMS-HDLC `7E..7E`
   frame) via `write_array()`; the end is detected by an idle gap of `request_gap`
   (default `100ms`), or by the upstream calling `flush()`.
2. **Pack** - wrap the HDLC frame in the d101-2 type-5A envelope
   (`98 F3 | OLEN | 00 01 | HLEN | 5A | serial | hdlc | A5 | CRC16`).
3. **Transmit** - send it over RF (narrow ~4 kHz TX profile, LSB-first on air,
   FIFO chunk-refilled for frames larger than the 64-byte FIFO).
4. **Receive** - switch to the wide ~28 kHz RX profile centred on the reply and
   drain the RX FIFO for up to `rf_rx_timeout`.
5. **Unpack** - CRC-carve the reply (`OLEN` + CRC-16/X.25), strip the
   `OLEN | 00 01 | HLEN` transport header, and push the inner HDLC frame into the
   reply FIFO, from which the upstream drains it via `read_array()`/`available()`.

The bridge is strictly half-duplex, request/reply. On a bad CRC or no reply it
retransmits (see **ARQ** below). Bytes written while an RF transaction is in
flight are buffered as the *next* request.

## Wiring (CMT2300A, bit-bang 3-wire SPI + INT)

| Signal | Purpose |
| --- | --- |
| `pin_sdio` | bidirectional data (MOSI/MISO) |
| `pin_sclk` | serial clock |
| `pin_csb`  | chip-select for register access |
| `pin_fcsb` | chip-select for FIFO access |
| `pin_gpio3`| chip INT2 output (RX_FIFO_TH / TX_FIFO_TH), polled for FIFO draining |

## YAML configuration

```yaml
external_components:
  - source: github://latonita/esphome-uart-nartis-rf
    components: [uart_nartis_rf]
    refresh: 1s

# The bridge IS a virtual UART - it takes NO uart_id.
uart_nartis_rf:
  id: rf_uart
  address: "023240271060"   # meter serial (12 digits)
  pin_sdio: GPIO19
  pin_sclk: GPIO18
  pin_csb: GPIO5
  pin_fcsb: GPIO21
  pin_gpio3: GPIO22
  request_gap: 100ms
  rf_tx_timeout: 1000ms
  rf_rx_timeout: 1000ms
  rf_retries: 2
  rx_center_offset: 758

# A UART-speaking meter component binds to the bridge instead of a real `uart:`.
# dlms_cosem:
#   uart_id: rf_uart
#   ...
```

## Configuration variables

- **id** (Required): Component ID. Point a meter component's `uart_id` at this.
- **address** (Required, string): Meter serial, exactly 12 digits (e.g.
  `"023240271060"`). Used as the BCD little-endian serial in the RF envelope and
  (in future) to derive the frequency.
- **pin_sdio / pin_sclk / pin_csb / pin_fcsb** (Required): CMT2300A SPI pins.
- **pin_gpio3** (Required): CMT2300A INT2 pin (input).
- **request_gap** (Optional, time): Idle gap on the write side that marks the end
  of a request. Default `100ms`. (An upstream `flush()` finalizes immediately.)
- **rf_tx_timeout** (Optional, time): TX-completion timeout. Default `1000ms`.
- **rf_rx_timeout** (Optional, time): How long to wait for an RF reply, per
  attempt. Default `1000ms`.
- **rf_retries** (Optional, int 0-10): RF-layer retransmissions on no-reply / bad
  CRC. `0` = pure transparent (no ARQ). Default `2` (up to 3 attempts).
- **rx_center_offset** (Optional, int): RX center offset in CMT2300A frequency
  codes (1 code ~= 6.199 Hz), to sit on the meter's reply carrier (a few kHz above
  TX). Default `758` (~+4.7 kHz, proven). The wide RX profile + AFC absorb drift.

## ARQ (link-layer retry) and timeout budgeting

The bridge owns fast, bounded retransmission: on a bad-CRC reply or no reply
within `rf_rx_timeout`, it re-packs (fresh frame) and resends, up to `rf_retries`
times, then gives up (delivering nothing, so the upstream's own retry takes over).

Because the upstream (e.g. `dlms_cosem`) simply waits on its receive timeout while
the bridge works, the two retry layers **serialize** rather than fight - as long
as `(rf_retries + 1) * rf_rx_timeout + tx_time` stays **under** the upstream's
receive timeout. Set the upstream `receive_timeout` generously (seconds).

Diagnostic counters (logged): no-reply, CRC errors, retries, give-ups.

## Automations

- **on_uart_message**: a complete request was collected (just before RF TX).
- **on_rf_reply**: a valid reply was received and queued back to the upstream.
- **on_rf_timeout**: no valid reply after exhausting `rf_retries`.

## Radio / protocol summary

- **PHY:** 2-FSK, 1.2 kbps, `0x55` preamble, sync `98 f3`, LSB-first on air, no
  whitening, no RF-layer encryption. Asymmetric channel: narrow (~4 kHz) TX,
  wide (~28 kHz) RX centred on the reply.
- **Envelope (type-5A, DLMS-HDLC):**
  - TX (client->server): `98 F3 | OLEN | 00 01 | HLEN | 5A | serial(6) | hdlc | A5 | CRC16`
  - RX (server->client): `98 F3 | OLEN | 00 01 | HLEN | hdlc | CRC16` (no 5A/serial/A5)
  - `OLEN` = all bytes after OLEN excluding the CRC; `HLEN = OLEN - 1`; `CRC` is
    CRC-16/X.25 over `OLEN..end`, little-endian. The `0xA5` terminator is required
    on requests.
- There is also a separate type-`68` (AES "quick") protocol on the same PHY; this
  bridge implements the type-`5A` cleartext DLMS-HDLC path only.

## Extending

The radio is behind `rf_*_` hooks in `uart_nartis_rf.cpp` (`rf_init_`, `rf_pack_`,
`rf_unpack_`, `rf_start_transmit_`, `rf_transmit_done_`, `rf_enter_rx_mode_`,
`rf_poll_receive_`, `rf_set_idle_`) over the `Cmt2300aHal` driver. Remaining work:
serial-derived frequency programming (arbitrary-channel register encoding).

## Threading model

All RF work runs in `loop()` as a non-blocking, poll-based state machine on the
main ESPHome task - **not** a separate thread or CPU core, and it doesn't need
one. The BLE bridge only looks event-driven because the ESP32 BLE stack forces
callbacks from its own task; an SPI-driven radio is master-polled with no external
event source, so a cooperative polled state machine is the right fit. The one
blocking spot is `transmit()` (a few tens of ms of airtime); everything else
returns quickly and resumes next pass.
