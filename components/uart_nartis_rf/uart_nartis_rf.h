/*
 * Virtual UART <-> RF433 bridge (Nartis).
 *
 * This component PRESENTS a UART (it is a `uart::UARTComponent`). Any component
 * that talks to a meter "over UART" can bind to this instead of the standard
 * `uart` bus - by pointing its `uart_id` at this component's id - and its
 * requests/replies are transparently relayed over a 433 MHz radio. The RF hop is
 * invisible to the upstream component: it writes request bytes and polls for a
 * reply exactly as it would against a real serial link.
 *
 * TRANSPARENCY CONTRACT: this is a dumb pipe. It owns the RF transport ONLY -
 * the on-air envelope (sync, length fields, frame-type byte, terminator) and the
 * envelope CRC. It never inspects, validates, or depends on the bytes the
 * upstream handed it: the payload is opaque, and exactly the bytes inside the
 * reply envelope are handed back. All payload framing (HDLC flags, FCS, DLMS
 * structure, retry semantics) belongs to the upstream component.
 *
 * Data flow (strictly half-duplex, request/reply):
 *
 *   upstream.write_array(req)  ->  collect into uart_msg_buf_
 *        (end of request detected by an idle gap, or by upstream flush())
 *                              ->  rf_pack_  ->  rf_start_transmit_  (TX_RF)
 *                              ->  rf_enter_rx_mode_                  (RX_RF)
 *   reply received over RF     ->  rf_unpack_  ->  push into rx_buffer_
 *   upstream.available()/read_array()  <-- serves the reply back to upstream
 *
 * Design notes (embedded-safe):
 *   - No heap allocation after setup(): message/packet buffers are fixed-size
 *     std::array; the reply FIFO is a single RingBuffer allocated once in setup().
 *   - Every state that waits on hardware has a timeout; the switch has a default
 *     safe-state branch.
 *   - The radio is driven only from loop(). RX is fully non-blocking (the FIFO is
 *     drained in chunks across loop() calls), but TX is synchronous: the HAL
 *     blocks for the frame's airtime (tens to hundreds of ms at 1.2 kbps, bounded
 *     by rf_tx_timeout_ms_). flush() does NOT block through an RF round-trip - it
 *     only finalizes the pending request so loop() sends it; the upstream then
 *     polls for the reply.
 */

#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/gpio.h"
#include "esphome/core/log.h"
#include "esphome/components/uart/uart_component.h"
#include "esphome/components/ring_buffer/ring_buffer.h"

#include "cmt2300a_hal.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace esphome::uart_nartis_rf {

/// Largest request (and largest unpacked RF reply) we will buffer.
static constexpr size_t MAX_UART_MESSAGE_SIZE = 512;
/// Largest on-air RF packet (payload plus whatever framing overhead is added).
static constexpr size_t MAX_RF_PACKET_SIZE = 640;
/// Envelope overhead added to a request by rf_pack_: sync(2) + OLEN(1) + 00 01 +
/// HLEN(1) + type(1) + serial(6) + terminator(1) + CRC(2).
static constexpr size_t RF_TX_OVERHEAD = 16;
/// The envelope length field is one byte, so OLEN <= 255 caps a relayed request at
/// 255 - (RF_TX_OVERHEAD - sync(2) - OLEN(1) - CRC(2)) bytes of payload.
static constexpr size_t MAX_RF_PAYLOAD_SIZE = 255 - (RF_TX_OVERHEAD - 5);
/// Reply direction: body is OLEN(1) + 00 01 + HLEN(1) + payload, so the same 8-bit
/// OLEN caps an inbound payload at 255 - 3.
static constexpr size_t MAX_RF_RX_PAYLOAD_SIZE = 255 - 3;

/// Result of an RF operation. I/O helpers return a status instead of void so the
/// state machine can react to timeouts and errors deterministically.
enum class RfStatus : uint8_t {
  OK,       // operation completed successfully
  BUSY,     // still in progress - poll again next loop
  NO_DATA,  // nothing received yet
  TIMEOUT,  // the radio itself reported a timeout
  ERROR,    // unrecoverable error - go to the safe state
};

class UartNartisRfComponent : public uart::UARTComponent, public Component {
 public:
  /// Bridge state machine.
  enum class BridgeState : uint8_t {
    IDLE,     // no request pending; waiting for the upstream to write
    COLLECT,  // upstream is writing a request; waiting for the end-of-request gap (or flush())
    TX_RF,    // packet packed; waiting for the radio to finish transmitting
    RX_RF,    // radio in RX; waiting for a reply or a timeout
    FAULT,    // safe state: radio idle, then return to IDLE
  };

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // --- Configuration setters (called from generated code) ---
  void set_request_gap_ms(uint32_t ms) { this->request_gap_ms_ = ms; }
  void set_rf_tx_timeout_ms(uint32_t ms) { this->rf_tx_timeout_ms_ = ms; }
  void set_rf_rx_timeout_ms(uint32_t ms) { this->rf_rx_timeout_ms_ = ms; }
  /// Number of RF retransmissions on no-reply / bad envelope CRC (0 = leave all
  /// retrying to the upstream). Total on-air attempts per request = 1 + rf_retries.
  void set_rf_retries(uint8_t n) { this->rf_retries_ = n; }
  /// RX center offset in frequency codes (1 code ~= 6.199 Hz); centres the RX-half
  /// LO on the meter's reply carrier.
  void set_rx_center_offset(int off) { this->rx_center_offset_ = off; }

  // --- CMT2300A wiring (bit-bang 3-wire SPI + INT pin) ---
  void set_pin_sdio(esphome::InternalGPIOPin *p) { this->pin_sdio_ = p; }
  void set_pin_sclk(esphome::InternalGPIOPin *p) { this->pin_sclk_ = p; }
  void set_pin_csb(esphome::InternalGPIOPin *p) { this->pin_csb_ = p; }
  void set_pin_fcsb(esphome::InternalGPIOPin *p) { this->pin_fcsb_ = p; }
  void set_pin_gpio3(esphome::InternalGPIOPin *p) { this->pin_gpio3_ = p; }

  /// Meter serial / RF address (digits only, e.g. "023240271060"). Used as the BCD
  /// serial in the RF envelope and selects the channel frequency via its last 3 digits.
  void set_address(const std::string &address) { this->address_ = address; }

  // --- Automation callbacks (templatized to accept lightweight forwarders) ---
  template<typename F> void add_on_uart_message_callback(F &&callback) {
    this->on_uart_message_callback_.add(std::forward<F>(callback));
  }
  template<typename F> void add_on_rf_reply_callback(F &&callback) {
    this->on_rf_reply_callback_.add(std::forward<F>(callback));
  }
  template<typename F> void add_on_rf_timeout_callback(F &&callback) {
    this->on_rf_timeout_callback_.add(std::forward<F>(callback));
  }

  // ==========================================================================
  // uart::UARTComponent interface - this component IS the (virtual) UART that
  // the upstream meter component binds to. Writes are requests to relay over RF;
  // reads drain replies received over RF.
  // ==========================================================================
  void write_array(const uint8_t *data, size_t len) override;
  void write_byte(uint8_t data);
  bool read_byte(uint8_t *data);
  bool peek_byte(uint8_t *data) override;
  bool read_array(uint8_t *data, size_t len) override;
  size_t available() override;
  uart::UARTFlushResult flush() override;

  // An RF link has no line settings (baud, parity, ...) to (re)load, so this is
  // a no-op, and there is no shared TX pin to conflict with the logger.
  void load_settings(bool dump_config) override {}
  using uart::UARTComponent::load_settings;  // bring in the no-arg overload
  void check_logger_conflict() override {}

 protected:
  // --- State machine helpers ---
  void set_state_(BridgeState state);
  const LogString *state_to_string_(BridgeState state) const;
  void begin_rf_tx_();                              // pack the collected request and start the first attempt
  void start_tx_attempt_();                         // transmit the already-packed frame
  void retry_or_give_up_(const LogString *reason);  // ARQ: retransmit if attempts remain, else give up
  void finish_rf_rx_(size_t packet_len);            // unpack a received packet into the reply FIFO
  void enter_fault_(const LogString *reason);
  void discard_reply_();  // drop any queued reply + peek cache (resync on a new request)

  // ==========================================================================
  // RF radio layer (CMT2300A). The RfStatus contract keeps the state machine
  // deterministic:
  //   - rf_pack_/rf_unpack_ : envelope only - add/strip header, terminator, CRC.
  //                           MUST NOT look at the payload bytes.
  //   - rf_start_transmit_  : transmit one frame (synchronous in this HAL).
  //   - rf_transmit_done_   : poll TX completion (OK done / BUSY / ERROR).
  //   - rf_enter_rx_mode_   : put the radio into receive.
  //   - rf_poll_receive_    : poll for a received packet (OK / NO_DATA / ERROR).
  //   - rf_set_idle_        : park the radio in a safe idle state.
  // ==========================================================================
  RfStatus rf_init_();
  RfStatus rf_pack_(const uint8_t *payload, size_t payload_len, uint8_t *out, size_t out_cap, size_t *out_len);
  RfStatus rf_unpack_(const uint8_t *packet, size_t packet_len, uint8_t *out, size_t out_cap, size_t *out_len);
  RfStatus rf_start_transmit_(const uint8_t *packet, size_t len);
  RfStatus rf_transmit_done_();
  RfStatus rf_enter_rx_mode_();
  RfStatus rf_poll_receive_(uint8_t *out, size_t out_cap, size_t *out_len);
  RfStatus rf_set_idle_();

  // --- Radio driver + wiring ---
  Cmt2300aHal hal_;
  esphome::InternalGPIOPin *pin_sdio_{nullptr};
  esphome::InternalGPIOPin *pin_sclk_{nullptr};
  esphome::InternalGPIOPin *pin_csb_{nullptr};
  esphome::InternalGPIOPin *pin_fcsb_{nullptr};
  esphome::InternalGPIOPin *pin_gpio3_{nullptr};

  // --- Meter address (serial) + derived RF frequency ---
  std::string address_;         // digits-only meter serial, e.g. "023240271060"
  uint8_t serial_le_[6]{};      // address_ as 6-byte BCD little-endian (023240271060 -> 60 10 27 40 32 02)
  uint32_t rf_frequency_hz_{0};  // derived from the last 3 digits of address_ (see compute)

  /// Fill serial_le_ from the 12-digit address_ (BCD, little-endian).
  void derive_serial_le_();

  /// Frequency (Hz) from the last 3 digits of the address:
  ///   n3   = value of the last 3 digits
  ///   k    = n3 % 24
  ///   freq = 435500000 + k * 700000  (+100000 if k > 18)
  uint32_t frequency_from_address_() const;

  // --- Configuration ---
  uint32_t request_gap_ms_{100};     // idle gap on the write side that ends a request
  uint32_t rf_tx_timeout_ms_{1000};  // budget for one transmit incl. airtime (passed to the HAL)
  uint32_t rf_rx_timeout_ms_{1000};  // how long to wait for the FIRST reply byte
  uint8_t rf_retries_{2};            // ARQ retransmissions on no-reply/bad-CRC (0 = off)

  // --- Runtime state ---
  BridgeState state_{BridgeState::IDLE};
  uint32_t state_enter_ms_{0};
  uint32_t last_write_ms_{0};
  bool force_send_{false};  // set by flush(): finalize the pending request without waiting for the gap
  // Set when the upstream writes a NEW request while an RF exchange is still in
  // flight - i.e. it gave up waiting (its own receive timeout). We then stop
  // retransmitting the dead request and never deliver its (late) reply, so a stale
  // answer can't desync the next request/reply pair.
  bool req_abandoned_{false};

  // --- ARQ (retransmission) state ---
  // The request is packed ONCE into rf_tx_buf_ and that exact frame is resent on
  // every attempt (a transparent link-layer retransmit - the envelope carries no
  // counter or nonce, so re-packing would only reproduce identical bytes).
  // req_len_ is the in-flight request length; 0 means "no exchange in flight".
  size_t req_len_{0};
  uint8_t tx_attempts_{0};  // on-air attempts made for the in-flight request (1 = first send)

  // --- Diagnostic counters (report; expose as sensors later if wanted) ---
  uint32_t rf_no_reply_count_{0};   // RX windows that ended with no reply
  uint32_t rf_crc_error_count_{0};  // replies that failed rf_unpack_ (framing/CRC)
  uint32_t rf_retry_count_{0};      // retransmissions performed
  uint32_t rf_giveup_count_{0};     // requests abandoned after exhausting retries

  // --- RF RX accumulation (drain_rx fills rf_rx_buf_ across loop() calls) ---
  size_t rf_rx_accum_len_{0};
  uint32_t rf_rx_last_chunk_ms_{0};
  /// Stop draining once we have the whole frame (first byte = OLEN => frame is
  /// OLEN + 3 bytes incl. the 2-byte CRC), or, as a fallback, once the byte cap
  /// is reached or no new chunk has arrived for RF_RX_END_GAP_MS. A fixed time
  /// window does NOT work: at 1.2 kbps the FIFO threshold only fires every ~100 ms,
  /// so the frame must be bounded by length, not by elapsed time.
  ///
  /// The cap must let the LARGEST envelope the length field can describe arrive in
  /// full: OLEN(1) + 255 + CRC(2) = 258 bytes. drain_rx() appends whole 15-byte
  /// chunks and only runs while (accumulated + 15 <= cap), so the cap needs at
  /// least one spare chunk above 258 -> 288 (a previous value of 96 silently
  /// truncated every reply longer than ~90 bytes into an endless retry loop).
  static constexpr size_t RF_RX_DRAIN_CAP = 288;
  static constexpr uint32_t RF_RX_END_GAP_MS = 400;
  /// Absolute ceiling on one RX window once bytes have started arriving.
  /// rf_rx_timeout_ms_ only bounds the wait for the FIRST byte: it must not cut a
  /// frame short, since 258 bytes at 1.2 kbps is ~1.7 s of pure airtime.
  static constexpr uint32_t RF_RX_MAX_WINDOW_MS = 5000;

  // --- RX center offset (freq codes; 1 code ~= 6.199 Hz) ---
  // The meter's reply sits a few kHz above our TX; this shifts the RX-half LO to
  // centre on it. The wide RX profile + AFC absorb normal thermal drift, so a
  // single fixed offset is used (no per-read sweep). Tune per install if needed.
  int rx_center_offset_{758};  // +758 codes ~= +4.7 kHz (443.905 MHz)

  // --- Outgoing request collected from the upstream UART consumer ---
  std::array<uint8_t, MAX_UART_MESSAGE_SIZE> uart_msg_buf_{};
  size_t uart_msg_len_{0};

  // --- RF packet buffers (no heap after setup) ---
  std::array<uint8_t, MAX_RF_PACKET_SIZE> rf_tx_buf_{};  // packed packet to transmit
  size_t rf_tx_len_{0};
  std::array<uint8_t, MAX_RF_PACKET_SIZE> rf_rx_buf_{};      // raw packet received over RF
  std::array<uint8_t, MAX_UART_MESSAGE_SIZE> unpack_buf_{};  // unpacked reply, before it enters rx_buffer_

  // --- Incoming replies served back to the upstream UART consumer (FIFO) ---
  static constexpr size_t RX_BUFFER_CAPACITY = 1024;
  std::unique_ptr<esphome::ring_buffer::RingBuffer> rx_buffer_;
  bool peek_valid_{false};  // single-byte peek cache (UARTComponent peek contract)
  uint8_t peek_byte_cache_{0};

  // --- Automation callbacks ---
  LazyCallbackManager<void()> on_uart_message_callback_;
  LazyCallbackManager<void()> on_rf_reply_callback_;
  LazyCallbackManager<void()> on_rf_timeout_callback_;
};

}  // namespace esphome::uart_nartis_rf
