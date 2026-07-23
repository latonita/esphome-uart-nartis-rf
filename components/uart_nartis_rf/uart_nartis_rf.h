/*
 * Virtual UART <-> RF433 bridge (Nartis) - STUB.
 *
 * This component PRESENTS a UART (it is a `uart::UARTComponent`). Any component
 * that talks to a meter "over UART" can bind to this instead of the standard
 * `uart` bus - by pointing its `uart_id` at this component's id - and its
 * requests/replies are transparently relayed over a 433 MHz radio. The RF hop is
 * invisible to the upstream component: it writes request bytes and polls for a
 * reply exactly as it would against a real serial link.
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
 * The RF radio driver and the pack/unpack framing are intentionally left as
 * no-op extension points (rf_*_ methods below). Everything else - the virtual
 * UART interface and the non-blocking bridge state machine - is wired up.
 *
 * Design notes (embedded-safe):
 *   - No heap allocation after setup(): message/packet buffers are fixed-size
 *     std::array; the reply FIFO is a single RingBuffer allocated once in setup().
 *   - Every state that waits on hardware has a timeout; the switch has a default
 *     safe-state branch.
 *   - The radio is only ever driven from loop(); nothing blocks. In particular
 *     flush() does NOT block through an RF round-trip - it only finalizes the
 *     pending request so loop() sends it; the upstream then polls for the reply.
 */

#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/gpio.h"
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
  /// Number of RF retransmissions on no-reply / bad CRC (0 = pure transparent, no
  /// ARQ). Total on-air attempts per request = 1 + rf_retries.
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

  /// Meter serial / RF address (digits only, e.g. "023240271060"). Goes into the
  /// RF packet (TODO) and selects the channel via its last 3 digits.
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
  const char *state_to_string_(BridgeState state) const;
  void begin_rf_tx_();                     // latch the collected request and start the first attempt
  void start_tx_attempt_();                // (re)pack the latched request and transmit it
  void retry_or_give_up_(const char *reason);  // ARQ: retransmit if attempts remain, else give up
  void finish_rf_rx_(size_t packet_len);   // unpack a received packet into the reply FIFO
  void enter_fault_(const char *reason);

  // ==========================================================================
  // RF radio extension points - ALL STUBS.
  //
  // Replace the bodies in uart_nartis_rf.cpp with the real CC1101 / RF433 driver
  // calls. Keep the RfStatus contract so the state machine keeps working:
  //   - rf_pack_/rf_unpack_ : framing (add/remove header, CRC, addressing, ...).
  //   - rf_start_transmit_  : kick off a (non-blocking) transmit.
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
  uint32_t request_gap_ms_{100};   // idle gap on the write side that ends a request
  uint32_t rf_tx_timeout_ms_{1000};
  uint32_t rf_rx_timeout_ms_{1000};
  uint8_t rf_retries_{2};          // ARQ retransmissions on no-reply/bad-CRC (0 = off)

  // --- Runtime state ---
  BridgeState state_{BridgeState::IDLE};
  uint32_t state_enter_ms_{0};
  uint32_t last_write_ms_{0};
  bool force_send_{false};  // set by flush(): finalize the pending request without waiting for the gap

  // --- ARQ (retransmission) state ---
  // The latched request is kept for the whole exchange so it can be re-packed
  // (fresh framing/counter) and retransmitted. It is separate from uart_msg_buf_
  // so the upstream's NEXT request can collect while this one is in flight.
  std::array<uint8_t, MAX_UART_MESSAGE_SIZE> req_buf_{};
  size_t req_len_{0};
  uint8_t tx_attempts_{0};  // on-air attempts made for the latched request (1 = first send)

  // --- Diagnostic counters (report; expose as sensors later if wanted) ---
  uint32_t rf_no_reply_count_{0};   // RX windows that ended with no reply
  uint32_t rf_crc_error_count_{0};  // replies that failed rf_unpack_ (framing/CRC)
  uint32_t rf_retry_count_{0};      // retransmissions performed
  uint32_t rf_giveup_count_{0};     // requests abandoned after exhausting retries

  // --- RF RX accumulation (drain_rx fills rf_rx_buf_ across loop() calls) ---
  size_t rf_rx_accum_len_{0};
  uint32_t rf_rx_last_chunk_ms_{0};
  /// Stop draining once we have the whole frame (first byte = OLEN => frame is
  /// OLEN + 3 bytes incl. the 2-byte CRC), or, as a fallback, once a full byte cap
  /// is reached or no new chunk has arrived for RF_RX_END_GAP_MS. A fixed time
  /// window does NOT work: at 1.2 kbps the FIFO threshold only fires every ~100 ms,
  /// so the frame must be bounded by length, not by elapsed time.
  static constexpr size_t RF_RX_DRAIN_CAP = 96;
  static constexpr uint32_t RF_RX_END_GAP_MS = 400;

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
