#include "uart_nartis_rf.h"

#include "esphome/core/log.h"
#include "esphome/core/hal.h"

#include <cinttypes>
#include <cstring>

namespace esphome::uart_nartis_rf {

static const char *const TAG = "uart_nartis_rf";

void UartNartisRfComponent::setup() {
  // The reply FIFO is the only heap allocation, done once here.
  this->rx_buffer_ = esphome::ring_buffer::RingBuffer::create(RX_BUFFER_CAPACITY);
  if (this->rx_buffer_ == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate RX buffer");
    this->mark_failed();
    return;
  }
  this->peek_valid_ = false;

  // Bring up the radio (once).
  if (this->rf_init_() != RfStatus::OK) {
    ESP_LOGE(TAG, "RF init failed");
    this->mark_failed();
    return;
  }
  this->set_state_(BridgeState::IDLE);
}

void UartNartisRfComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Virtual UART <-> RF433 bridge (Nartis, CMT2300A):");
  if (this->pin_sdio_ != nullptr && this->pin_sclk_ != nullptr && this->pin_csb_ != nullptr &&
      this->pin_fcsb_ != nullptr && this->pin_gpio3_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Pins: SDIO=%d, SCLK=%d, CSB=%d, FCSB=%d, GPIO3=%d",
                  this->pin_sdio_->get_pin(), this->pin_sclk_->get_pin(), this->pin_csb_->get_pin(),
                  this->pin_fcsb_->get_pin(), this->pin_gpio3_->get_pin());
  }
  ESP_LOGCONFIG(TAG, "  Address (meter serial): %s", this->address_.c_str());
  ESP_LOGCONFIG(TAG, "  RF frequency: %.3f MHz (channel derived from meter serial)",
                this->frequency_from_address_() / 1e6f);
  ESP_LOGCONFIG(TAG, "  RX center offset: %d codes (~%+.1f kHz)", this->rx_center_offset_,
                this->rx_center_offset_ * RX_CODE_HZ / 1000.0f);
  ESP_LOGCONFIG(TAG, "  End-of-request gap: %" PRIu32 " ms", this->request_gap_ms_);
  ESP_LOGCONFIG(TAG, "  RF TX timeout: %" PRIu32 " ms", this->rf_tx_timeout_ms_);
  ESP_LOGCONFIG(TAG, "  RF RX timeout: %" PRIu32 " ms", this->rf_rx_timeout_ms_);
  ESP_LOGCONFIG(TAG, "  RF retries: %u (up to %u attempts per request)", this->rf_retries_,
                (unsigned) (this->rf_retries_ + 1));
  ESP_LOGCONFIG(TAG, "  Max payload: %zu bytes out, %zu bytes in", MAX_RF_PAYLOAD_SIZE, MAX_RF_RX_PAYLOAD_SIZE);
  ESP_LOGCONFIG(TAG, "  RF: RF433-2 PHY + envelope (CRC-16/X.25). Payload is relayed verbatim - "
                     "the bridge adds/strips the envelope only.");
}

void UartNartisRfComponent::loop() {
  const uint32_t now = millis();

  switch (this->state_) {
    case BridgeState::IDLE:
      // A write while idle moves us to COLLECT; bytes buffered during a previous
      // RF transaction are also picked up here.
      if (this->uart_msg_len_ > 0) {
        this->discard_reply_();  // fresh exchange: drop any stale reply left queued
        this->set_state_(BridgeState::COLLECT);
      }
      break;

    case BridgeState::COLLECT:
      // The request is complete once the upstream flushed, or once the write side
      // has been quiet for request_gap_ms_.
      if (this->uart_msg_len_ > 0 &&
          (this->force_send_ || (now - this->last_write_ms_) >= this->request_gap_ms_)) {
        ESP_LOGV(TAG, "Request complete (%zu bytes)%s", this->uart_msg_len_,
                 LOG_STR_ARG(this->force_send_ ? LOG_STR(" [flush]") : LOG_STR("")));
        this->force_send_ = false;
        this->on_uart_message_callback_.call();
        this->begin_rf_tx_();
      }
      break;

    case BridgeState::TX_RF: {
      const RfStatus st = this->rf_transmit_done_();
      if (st == RfStatus::OK) {
        ESP_LOGV(TAG, "RF TX done, switching to RX");
        if (this->rf_enter_rx_mode_() != RfStatus::OK) {
          this->enter_fault_(LOG_STR("failed to enter RF RX mode"));
          break;
        }
        this->set_state_(BridgeState::RX_RF);
      } else if (st == RfStatus::BUSY) {
        if ((now - this->state_enter_ms_) >= this->rf_tx_timeout_ms_) {
          this->enter_fault_(LOG_STR("RF TX timeout"));
        }
      } else {
        this->enter_fault_(LOG_STR("RF TX error"));
      }
      break;
    }

    case BridgeState::RX_RF: {
      size_t packet_len = 0;
      const RfStatus st = this->rf_poll_receive_(this->rf_rx_buf_.data(), this->rf_rx_buf_.size(), &packet_len);
      if (st == RfStatus::OK) {
        ESP_LOGV(TAG, "RF reply received (%zu bytes)", packet_len);
        this->finish_rf_rx_(packet_len);
      } else if (st == RfStatus::NO_DATA) {
        // Nothing has arrived at all - this is what rf_rx_timeout_ms_ bounds.
        if ((now - this->state_enter_ms_) >= this->rf_rx_timeout_ms_) {
          this->rf_no_reply_count_++;
          this->retry_or_give_up_(LOG_STR("no reply"));
        }
      } else if (st == RfStatus::BUSY) {
        // A frame IS arriving. rf_rx_timeout_ms_ must not apply here: a full-size
        // envelope is ~1.7 s of airtime at 1.2 kbps, so a 1 s reply timeout would
        // chop every long reply. rf_poll_receive_ ends a stalled frame via
        // RF_RX_END_GAP_MS; this is only the absolute backstop.
        if ((now - this->state_enter_ms_) >= RF_RX_MAX_WINDOW_MS) {
          this->rf_no_reply_count_++;
          this->retry_or_give_up_(LOG_STR("RX window exceeded mid-frame"));
        }
      } else {
        this->enter_fault_(LOG_STR("RF RX error"));
      }
      break;
    }

    case BridgeState::FAULT:
      // Safe state: park the radio and return to idle so the bridge self-recovers.
      this->rf_set_idle_();
      this->set_state_(BridgeState::IDLE);
      break;

    default:
      // Unknown state - fail safe.
      this->enter_fault_(LOG_STR("unknown state"));
      break;
  }
}

// ============================================================================
// uart::UARTComponent interface.
//
// The upstream component treats us as its serial link: write_array() feeds us a
// request, read_*/available()/peek_byte() drain the reply we received over RF.
// ============================================================================

void UartNartisRfComponent::write_array(const uint8_t *data, size_t len) {
  if (data == nullptr || len == 0) {
    return;
  }
  // A write arriving while an RF exchange is still in flight means the upstream
  // gave up waiting (its own receive timeout) and started a new request. Mark the
  // in-flight exchange abandoned so we stop retransmitting it and never deliver its
  // late reply as the answer to this new request.
  if (this->state_ == BridgeState::TX_RF || this->state_ == BridgeState::RX_RF) {
    this->req_abandoned_ = true;
  }

  // Accept bytes at any time. While an RF transaction is in flight (TX_RF/RX_RF)
  // begin_rf_tx_() has already zeroed uart_msg_len_, so these accumulate cleanly
  // as the NEXT request and are sent once we return to IDLE -> COLLECT.
  for (size_t i = 0; i < len; i++) {
    if (this->uart_msg_len_ >= this->uart_msg_buf_.size()) {
      ESP_LOGW(TAG, "Request buffer full (%zu bytes), dropping byte", this->uart_msg_buf_.size());
      break;
    }
    this->uart_msg_buf_[this->uart_msg_len_++] = data[i];
  }
  this->last_write_ms_ = millis();
  if (this->state_ == BridgeState::IDLE) {
    this->discard_reply_();  // fresh exchange: drop any stale reply from a timed-out one
    this->set_state_(BridgeState::COLLECT);
  }
}

void UartNartisRfComponent::write_byte(uint8_t data) { this->write_array(&data, 1); }

bool UartNartisRfComponent::read_byte(uint8_t *data) { return this->read_array(data, 1); }

bool UartNartisRfComponent::peek_byte(uint8_t *data) {
  if (this->peek_valid_) {
    if (data != nullptr) {
      *data = this->peek_byte_cache_;
    }
    return true;
  }
  if (this->rx_buffer_ == nullptr || this->rx_buffer_->available() == 0) {
    return false;
  }
  uint8_t tmp{0};
  if (this->rx_buffer_->read(&tmp, 1, 0) == 0) {
    return false;
  }
  this->peek_byte_cache_ = tmp;
  this->peek_valid_ = true;
  if (data != nullptr) {
    *data = tmp;
  }
  return true;
}

bool UartNartisRfComponent::read_array(uint8_t *data, size_t len) {
  if (data == nullptr || len == 0) {
    return true;
  }
  // All-or-nothing: check first, consume second. Consuming the peek cache before
  // discovering the FIFO is short would drop that byte on the floor - the caller
  // sees `false`, assumes nothing was read, and the byte is gone from the stream.
  if (this->available() < len) {
    return false;
  }

  size_t remaining = len;
  size_t offset = 0;

  // Serve the peeked byte first, if any.
  if (this->peek_valid_) {
    data[0] = this->peek_byte_cache_;
    this->peek_valid_ = false;
    remaining--;
    offset = 1;
  }
  if (remaining == 0) {
    return true;
  }
  return this->rx_buffer_->read(data + offset, remaining, 0) == remaining;
}

size_t UartNartisRfComponent::available() {
  const size_t buffered = (this->rx_buffer_ != nullptr) ? this->rx_buffer_->available() : 0;
  return buffered + (this->peek_valid_ ? 1 : 0);
}

uart::UARTFlushResult UartNartisRfComponent::flush() {
  // Non-blocking: instead of waiting through an RF round-trip (which would stall
  // the whole ESPHome loop), we mark the pending request ready so loop() sends it
  // on the next iteration. The upstream then polls available() for the reply.
  if (this->uart_msg_len_ > 0) {
    this->force_send_ = true;
  }
  return uart::UARTFlushResult::UART_FLUSH_RESULT_SUCCESS;
}

// ============================================================================
// State machine helpers.
// ============================================================================

void UartNartisRfComponent::begin_rf_tx_() {
  // Pack ONCE into rf_tx_buf_: that packed frame is what every attempt resends, so
  // uart_msg_buf_ is immediately free to collect the upstream's next request while
  // this one is in flight.
  this->req_len_ = this->uart_msg_len_;
  this->rf_tx_len_ = 0;
  const RfStatus st = this->rf_pack_(this->uart_msg_buf_.data(), this->req_len_, this->rf_tx_buf_.data(),
                                     this->rf_tx_buf_.size(), &this->rf_tx_len_);
  this->uart_msg_len_ = 0;
  this->tx_attempts_ = 0;
  this->req_abandoned_ = false;  // fresh request: not (yet) superseded by a newer one
  if (st != RfStatus::OK) {
    // Packing is deterministic - retrying won't help, so fail safe.
    this->enter_fault_(LOG_STR("RF packing failed"));
    return;
  }
  this->start_tx_attempt_();
}

void UartNartisRfComponent::start_tx_attempt_() {
  // Resend the frame packed by begin_rf_tx_(). A transparent retransmit is
  // byte-identical: the envelope carries no frame counter or nonce, so there is
  // nothing to regenerate per attempt.
  // Count the attempt BEFORE transmitting so a persistent transmit-start
  // failure still advances toward give-up instead of looping forever.
  this->tx_attempts_++;
  const RfStatus st = this->rf_start_transmit_(this->rf_tx_buf_.data(), this->rf_tx_len_);
  if (st != RfStatus::OK) {
    // Couldn't even start the transmit: treat as a failed attempt so ARQ can
    // retry (the radio may have been momentarily busy).
    this->retry_or_give_up_(LOG_STR("RF transmit start failed"));
    return;
  }
  this->set_state_(BridgeState::TX_RF);
}

void UartNartisRfComponent::retry_or_give_up_(const LogString *reason) {
  // If the upstream already moved on to a new request, stop burning airtime on the
  // dead one - drop it and let loop() pick up the buffered new request.
  if (this->req_abandoned_) {
    ESP_LOGW(TAG, "RF %s - upstream abandoned the request, stopping retries", LOG_STR_ARG(reason));
    this->req_len_ = 0;
    this->rf_set_idle_();
    this->set_state_(BridgeState::IDLE);
    return;
  }

  // ARQ: one send plus rf_retries_ retransmissions. tx_attempts_ counts sends
  // already made (>=1 here), so retry while tx_attempts_ <= rf_retries_.
  //
  // This is a blind link-layer retransmit: the transport cannot tell a repeatable
  // request from one with side effects, because it never parses the payload. Set
  // rf_retries: 0 to leave all retrying to the upstream, which does know.
  if (this->tx_attempts_ <= this->rf_retries_) {
    this->rf_retry_count_++;
    ESP_LOGW(TAG, "RF %s - retransmit (attempt %u/%u)", LOG_STR_ARG(reason),
             (unsigned) (this->tx_attempts_ + 1), (unsigned) (this->rf_retries_ + 1));
    this->rf_set_idle_();
    this->start_tx_attempt_();  // resend the same packed frame
    return;
  }

  this->rf_giveup_count_++;
  ESP_LOGW(TAG, "RF %s - giving up after %u attempt(s) [no_reply=%" PRIu32 " crc_err=%" PRIu32
                " retries=%" PRIu32 " giveups=%" PRIu32 "]",
           LOG_STR_ARG(reason), (unsigned) this->tx_attempts_, this->rf_no_reply_count_, this->rf_crc_error_count_,
           this->rf_retry_count_, this->rf_giveup_count_);
  this->req_len_ = 0;
  this->on_rf_timeout_callback_.call();  // no valid reply delivered this request
  this->rf_set_idle_();
  this->set_state_(BridgeState::IDLE);
}

void UartNartisRfComponent::finish_rf_rx_(size_t packet_len) {
  // If the upstream gave up and started a new request, this reply is stale - never
  // hand it back, or the next request would read the previous answer (desync).
  if (this->req_abandoned_) {
    ESP_LOGW(TAG, "RF reply (%zu bytes) arrived after the request was abandoned - discarding", packet_len);
    this->req_len_ = 0;
    this->discard_reply_();
    this->rf_set_idle_();
    this->set_state_(BridgeState::IDLE);
    return;
  }

  ESP_LOGVV(TAG, "RF RX raw [%zu]: %s", packet_len,
           format_hex_pretty(this->rf_rx_buf_.data(), packet_len).c_str());

  size_t unpacked_len = 0;
  const RfStatus st = this->rf_unpack_(this->rf_rx_buf_.data(), packet_len, this->unpack_buf_.data(),
                                       this->unpack_buf_.size(), &unpacked_len);
  if (st != RfStatus::OK) {
    // A corrupt/invalid frame (bad framing or CRC) is a transient RF error -
    // retransmit via ARQ rather than faulting the whole bridge.
    this->rf_crc_error_count_++;
    this->retry_or_give_up_(LOG_STR("bad CRC/framing"));
    return;
  }

  // Success: the latched request is done.
  this->req_len_ = 0;
  ESP_LOGVV(TAG, "RF RX payload [%zu]: %s", unpacked_len,
           format_hex_pretty(this->unpack_buf_.data(), unpacked_len).c_str());

  if (unpacked_len > 0 && this->rx_buffer_ != nullptr) {
    const size_t written = this->rx_buffer_->write(this->unpack_buf_.data(), unpacked_len);
    if (written < unpacked_len) {
      ESP_LOGW(TAG, "Reply FIFO overflow, dropped %zu bytes (upstream not reading?)",
               unpacked_len - written);
    }
    ESP_LOGV(TAG, "Queued %zu reply byte(s) for upstream", written);
  }
  this->on_rf_reply_callback_.call();
  this->rf_set_idle_();
  this->set_state_(BridgeState::IDLE);
}

void UartNartisRfComponent::discard_reply_() {
  // Drop any queued reply and the one-byte peek cache so a stale answer from a
  // previous (abandoned/timed-out) exchange can't be read as this request's reply.
  if (this->rx_buffer_ != nullptr) {
    this->rx_buffer_->reset();
  }
  this->peek_valid_ = false;
}

void UartNartisRfComponent::enter_fault_(const LogString *reason) {
  ESP_LOGE(TAG, "Entering FAULT: %s", LOG_STR_ARG(reason));
  if (this->uart_msg_len_ > 0) {
    // Say so rather than swallowing it: the upstream already handed these bytes
    // over, so a silent drop looks like a black hole instead of a failed request.
    ESP_LOGW(TAG, "FAULT discards %zu byte(s) of a pending request", this->uart_msg_len_);
  }
  const bool had_request = this->uart_msg_len_ > 0 || this->req_len_ > 0;
  this->uart_msg_len_ = 0;
  this->req_len_ = 0;
  this->rf_tx_len_ = 0;
  this->force_send_ = false;
  this->set_state_(BridgeState::FAULT);
  // A fault means no reply will ever be delivered for this request - give
  // automations the same signal they get from an exhausted-retry give-up.
  if (had_request) {
    this->on_rf_timeout_callback_.call();
  }
}

void UartNartisRfComponent::set_state_(BridgeState state) {
  if (this->state_ != state) {
    ESP_LOGV(TAG, "State: %s -> %s", LOG_STR_ARG(this->state_to_string_(this->state_)),
             LOG_STR_ARG(this->state_to_string_(state)));
    this->state_ = state;
  }
  this->state_enter_ms_ = millis();
}

const LogString *UartNartisRfComponent::state_to_string_(BridgeState state) const {
  switch (state) {
    case BridgeState::IDLE:
      return LOG_STR("IDLE");
    case BridgeState::COLLECT:
      return LOG_STR("COLLECT");
    case BridgeState::TX_RF:
      return LOG_STR("TX_RF");
    case BridgeState::RX_RF:
      return LOG_STR("RX_RF");
    case BridgeState::FAULT:
      return LOG_STR("FAULT");
    default:
      return LOG_STR("UNKNOWN");
  }
}

uint32_t UartNartisRfComponent::frequency_from_address_() const {
  // n3 = value of the last 3 digits; k = n3 % 24;
  // freq = 435.5 MHz + k * 0.7 MHz, plus 100 kHz when k > 18.
  //
  // The full k -> frequency table (uniform 0.7 MHz step for k 0..18; the +100 kHz
  // for k > 18 makes the k=18->19 gap 0.8 MHz, then 0.7 MHz again):
  //
  //   k   freq (Hz)      MHz        k   freq (Hz)      MHz
  //   --  -----------  --------     --  -----------  --------
  //    0  435 500 000  435.500      12  443 900 000  443.900
  //    1  436 200 000  436.200      13  444 600 000  444.600
  //    2  436 900 000  436.900      14  445 300 000  445.300
  //    3  437 600 000  437.600      15  446 000 000  446.000
  //    4  438 300 000  438.300      16  446 700 000  446.700
  //    5  439 000 000  439.000      17  447 400 000  447.400
  //    6  439 700 000  439.700      18  448 100 000  448.100
  //    7  440 400 000  440.400      19  448 900 000  448.900  (+100 kHz)
  //    8  441 100 000  441.100      20  449 600 000  449.600  (+100 kHz)
  //    9  441 800 000  441.800      21  450 300 000  450.300  (+100 kHz)
  //   10  442 500 000  442.500      22  451 000 000  451.000  (+100 kHz)
  //   11  443 200 000  443.200      23  451 700 000  451.700  (+100 kHz)
  //
  // n3 (000..999) maps to k by n3 % 24, so each k is hit by ~42 last-3-digit
  // values (e.g. "...060" -> 60 % 24 = 12 -> 443.900 MHz).
  uint32_t n3 = 0;
  const size_t len = this->address_.size();
  const size_t start = (len >= 3) ? (len - 3) : 0;
  for (size_t i = start; i < len; i++) {
    const char c = this->address_[i];
    if (c >= '0' && c <= '9') {
      n3 = n3 * 10 + static_cast<uint32_t>(c - '0');
    }
  }
  const uint32_t k = n3 % 24;
  uint32_t freq = 435500000u + k * 700000u;
  if (k > 18) {
    freq += 100000u;
  }
  return freq;
}

// ============================================================================
// RF radio layer - CMT2300A PHY.
//
// PHY: 443.9 MHz, asymmetric channel (narrow TX / wide RX), 98 f3 sync, LSB-first
// on air, CRC-16/X.25. Framing is an asymmetric envelope:
//   TX (client->server): 98 F3 | OLEN | 00 01 | HLEN | TYPE | serial | payload | A5 | CRC
//   RX (server->client): 98 F3 | OLEN | 00 01 | HLEN | payload | CRC  (no TYPE/serial/A5)
// rf_pack_ builds the request envelope; rf_unpack_ CRC-carves the reply and strips
// the fixed OLEN|00 01|HLEN header to hand the payload back to the upstream.
// Both directions treat the payload as opaque bytes.
// ============================================================================

// CRC-16/X.25 (poly 0x1021, init/xorout 0xFFFF, reflected in/out) - the RF433-2
// frame CRC. Ported from the test app.
static uint16_t crc16_x25(const uint8_t *d, size_t n) {
  uint16_t c = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    c ^= (uint16_t) Cmt2300aHal::reverse8(d[i]) << 8;
    for (int k = 0; k < 8; k++)
      c = (c & 0x8000) ? (uint16_t) ((c << 1) ^ 0x1021) : (uint16_t) (c << 1);
  }
  uint16_t r = 0;
  for (int i = 0; i < 16; i++)
    if (c & (1 << i))
      r |= (1 << (15 - i));
  return r ^ 0xFFFF;
}

static constexpr uint8_t RF433_2_SYNC0 = 0x98;
static constexpr uint8_t RF433_2_SYNC1 = 0xF3;
/// Frame-type byte of a client->server request. A fixed field of the RF433-2
/// envelope, like the sync words and the 0xA5 terminator - it describes the
/// transport, not the payload, so it is a constant and not a config option.
static constexpr uint8_t RF433_2_TYPE_REQUEST = 0x5A;
/// Terminator: the meter ignores request frames without it.
static constexpr uint8_t RF433_2_TERMINATOR = 0xA5;

RfStatus UartNartisRfComponent::rf_init_() {
  if (this->pin_sdio_ == nullptr || this->pin_sclk_ == nullptr || this->pin_csb_ == nullptr ||
      this->pin_fcsb_ == nullptr || this->pin_gpio3_ == nullptr) {
    ESP_LOGE(TAG, "rf_init_: CMT2300A pins not configured");
    return RfStatus::ERROR;
  }

  this->hal_.set_pins(this->pin_sdio_, this->pin_sclk_, this->pin_csb_, this->pin_fcsb_, this->pin_gpio3_);

  this->derive_serial_le_();

  // Channel frequency is derived from the meter serial (last 3 digits) and applied
  // to the CMT2300A frequency bank (AN199 formula in the HAL). Must be set before
  // init(), which writes the computed bank.
  this->rf_frequency_hz_ = this->frequency_from_address_();
  this->hal_.set_frequency(this->rf_frequency_hz_);

  if (!this->hal_.init()) {
    ESP_LOGE(TAG, "rf_init_: CMT2300A initialization failed - check wiring");
    return RfStatus::ERROR;
  }

  ESP_LOGI(TAG, "rf_init_: CMT2300A ready at %.3f MHz (channel derived from address %s)",
           this->rf_frequency_hz_ / 1e6f, this->address_.c_str());
  return RfStatus::OK;
}

void UartNartisRfComponent::derive_serial_le_() {
  // 12 ASCII digits -> 6 BCD bytes (MSB pair first) -> reverse to little-endian.
  // "023240271060" -> BCD 02 32 40 27 10 60 -> LE 60 10 27 40 32 02.
  uint8_t bcd[6] = {0, 0, 0, 0, 0, 0};
  if (this->address_.size() >= 12) {
    for (size_t i = 0; i < 6; i++) {
      const uint8_t hi = (uint8_t) (this->address_[2 * i] - '0');
      const uint8_t lo = (uint8_t) (this->address_[2 * i + 1] - '0');
      bcd[i] = (uint8_t) ((hi << 4) | lo);
    }
  }
  for (size_t i = 0; i < 6; i++)
    this->serial_le_[i] = bcd[5 - i];
}

RfStatus UartNartisRfComponent::rf_pack_(const uint8_t *payload, size_t payload_len, uint8_t *out, size_t out_cap,
                                         size_t *out_len) {
  // Wrap the upstream's bytes, whatever they are, in the RF433-2 RF envelope
  // (request direction):
  //   98 F3 | OLEN | 00 01 | HLEN | TYPE | serial(6) | <payload> | A5 | CRC16(LE)
  //   OLEN = len(00 01 | HLEN | TYPE | serial | payload | A5)  (after OLEN, excl. CRC)
  //   HLEN = OLEN ^ 1   (see below)
  //   CRC  = CRC-16/X.25 over [OLEN .. A5], little-endian
  // The 0xA5 terminator is REQUIRED (the meter ignores frames without it).
  // The HAL adds the 0x55 pad and the LSB-first bit-reversal.
  // TYPE and the terminator are fixed RF433-2 envelope fields (see the constants
  // above). `payload` is copied verbatim - never parsed, and its length is taken
  // from the caller, never derived from its contents.
  if (payload == nullptr || out == nullptr || out_len == nullptr) {
    return RfStatus::ERROR;
  }
  const size_t olen = 2 + 1 + 1 + 6 + payload_len + 1;  // 00 01 + HLEN + TYPE + serial(6) + payload + A5
  // HLEN = OLEN ^ 1. This was OLEN - 1, which is the same thing for odd OLEN and
  // differs for even. Every even-length frame seen from the meter uses OLEN ^ 1:
  // two replies on this link (OLEN 24 -> 25, 12 -> 13), the D101-2 display's own
  // status poll (0x12 -> 0x13) and the DI 0xF202 reply (0x60 -> 0x61). No
  // counterexample.
  //
  // The field is ignored on receive - the meter answers frames built either way -
  // so this is about matching the real display exactly, not about working at all.
  const size_t hlen = olen ^ 1;
  const size_t total = 2 + 1 + olen + 2;  // sync(2) + OLEN(1) + content(olen) + CRC(2)
  if (payload_len > MAX_RF_PAYLOAD_SIZE) {
    ESP_LOGW(TAG, "rf_pack_: request of %zu bytes exceeds the %zu-byte on-air limit (8-bit length field)", payload_len,
             MAX_RF_PAYLOAD_SIZE);
    return RfStatus::ERROR;
  }
  if (olen > 0xFF || total > out_cap) {
    ESP_LOGW(TAG, "rf_pack_: frame (%zu) exceeds buffer (%zu) or OLEN>255", total, out_cap);
    return RfStatus::ERROR;
  }

  size_t p = 0;
  out[p++] = RF433_2_SYNC0;
  out[p++] = RF433_2_SYNC1;
  out[p++] = (uint8_t) olen;  // OLEN
  out[p++] = 0x00;
  out[p++] = 0x01;
  out[p++] = (uint8_t) hlen;  // HLEN = OLEN ^ 1
  out[p++] = RF433_2_TYPE_REQUEST;
  for (size_t i = 0; i < 6; i++)
    out[p++] = this->serial_le_[i];
  std::memcpy(out + p, payload, payload_len);
  p += payload_len;
  out[p++] = RF433_2_TERMINATOR;

  const uint16_t crc = crc16_x25(out + 2, olen + 1);  // OLEN byte + content
  out[p++] = (uint8_t) (crc & 0xFF);
  out[p++] = (uint8_t) ((crc >> 8) & 0xFF);

  *out_len = p;
  ESP_LOGVV(TAG, "rf_pack_: %zu payload -> %zu on-air bytes (OLEN=%u HLEN=%u)", payload_len, p, (unsigned) olen,
           (unsigned) hlen);
  return RfStatus::OK;
}

RfStatus UartNartisRfComponent::rf_unpack_(const uint8_t *packet, size_t packet_len, uint8_t *out, size_t out_cap,
                                           size_t *out_len) {
  // The chip strips 98 f3, so `packet` starts at the OLEN byte and runs into
  // trailing noise (fixed-length capture). Carve the frame by length + CRC:
  //   OLEN = packet[lp]; CRC-16/X.25 over packet[lp .. lp+OLEN] == next 2 (LE).
  // Scan a few start positions for robustness.
  //
  // The reply envelope (server->client) is OLEN | 00 01 | HLEN | <payload>, with no
  // type/serial/terminator. We validate only OUR OWN fields (CRC, the 00 01 marker,
  // HLEN's self-consistency), then strip the fixed 4-byte header unconditionally and
  // hand back the payload verbatim. The payload is opaque: it is NOT inspected, and
  // how many bytes we strip must never depend on what is inside it.
  if (packet == nullptr || out == nullptr || out_len == nullptr) {
    return RfStatus::ERROR;
  }
  static constexpr size_t RX_HEADER_LEN = 4;  // OLEN + 00 01 + HLEN
  for (size_t lp = 0; lp <= 3; lp++) {
    if (lp >= packet_len)
      break;
    const size_t olen = packet[lp];
    if (olen < 3 || lp + 1 + olen + 2 > packet_len)
      continue;
    const uint16_t calc = crc16_x25(packet + lp, olen + 1);
    const uint16_t got = (uint16_t) packet[lp + 1 + olen] | ((uint16_t) packet[lp + 2 + olen] << 8);
    if (calc != got)
      continue;

    // CRC-OK: this is our frame. body spans OLEN..end-of-content (CRC excluded).
    const uint8_t *body = packet + lp;
    const size_t body_len = olen + 1;
    if (body_len < RX_HEADER_LEN) {
      ESP_LOGW(TAG, "rf_unpack_: CRC-OK but body too short for the envelope header (%zu bytes)", body_len);
      return RfStatus::ERROR;
    }
    // Envelope-level anomalies are REPORTED, not enforced: a matching CRC-16/X.25
    // over the whole body is already strong proof this is our frame, and the exact
    // reply-direction rules for these two fields are inferred from captures rather
    // than specified. Rejecting on them could drop perfectly good replies.
    if (body[1] != 0x00 || body[2] != 0x01) {
      ESP_LOGW(TAG, "rf_unpack_: unexpected envelope marker %02X %02X (expected 00 01) - relaying anyway", body[1],
               body[2]);
    }
    if (body[3] != (uint8_t) (olen ^ 1)) {
      ESP_LOGD(TAG, "rf_unpack_: HLEN=%u, expected OLEN^1=%u", body[3], (unsigned) (olen ^ 1));
    }

    const size_t payload_len = body_len - RX_HEADER_LEN;
    if (payload_len > out_cap) {
      ESP_LOGW(TAG, "rf_unpack_: payload (%zu) exceeds buffer (%zu)", payload_len, out_cap);
      return RfStatus::ERROR;
    }
    std::memcpy(out, body + RX_HEADER_LEN, payload_len);
    *out_len = payload_len;
    ESP_LOGV(TAG, "rf_unpack_: CRC-OK, %zu-byte payload (body %zu, start %zu)", payload_len, body_len, lp);
    return RfStatus::OK;
  }
  ESP_LOGE(TAG, "rf_unpack_: no CRC-OK frame in %zu bytes", packet_len);
  return RfStatus::ERROR;  // -> ARQ retransmit
}

RfStatus UartNartisRfComponent::rf_start_transmit_(const uint8_t *packet, size_t len) {
  // hal_.transmit() is synchronous: applies the TX profile, pads + bit-reverses,
  // fills the FIFO, GO_TX, and blocks until TX_DONE or rf_tx_timeout_ms_. It holds
  // the loop for the frame's airtime (~tens to hundreds of ms at 1.2 kbps), which
  // is why TX_RF is a pass-through state rather than a polled wait.
  ESP_LOGVV(TAG, "RF TX [%zu]: %s", len, format_hex_pretty(packet, len).c_str());
  // rf_tx_timeout_ms_ bounds a STUCK radio, so it must never be shorter than the
  // frame's own airtime - otherwise a long request "times out" mid-transmission.
  // At 1.2 kbps one byte is ~6.7 ms; give that plus 50% margin, whichever is larger.
  const uint32_t airtime_ms = (uint32_t) len * 20u / 3u;
  const uint32_t needed_ms = airtime_ms + airtime_ms / 2u + 100u;
  const uint32_t budget_ms = (needed_ms > this->rf_tx_timeout_ms_) ? needed_ms : this->rf_tx_timeout_ms_;
  if (!this->hal_.transmit(packet, len, budget_ms)) {
    ESP_LOGW(TAG, "rf_start_transmit_: TX did not complete");
    return RfStatus::ERROR;
  }
  return RfStatus::OK;
}

RfStatus UartNartisRfComponent::rf_transmit_done_() {
  // hal_.transmit() already blocked until TX_DONE (bounded by rf_tx_timeout_ms_)
  // and a failure there was reported by rf_start_transmit_, so TX is complete here.
  // The TX_RF state therefore costs exactly one loop() iteration.
  return RfStatus::OK;
}

RfStatus UartNartisRfComponent::rf_enter_rx_mode_() {
  // Apply the wide RX profile centred on the reply and enter RX.
  if (!this->hal_.begin_rx(this->rx_center_offset_)) {
    ESP_LOGW(TAG, "rf_enter_rx_mode_: failed to enter RX");
    return RfStatus::ERROR;
  }
  this->rf_rx_accum_len_ = 0;
  this->rf_rx_last_chunk_ms_ = millis();
  ESP_LOGVV(TAG, "rf_enter_rx_mode_: RX armed (center offset %d codes)", this->rx_center_offset_);
  return RfStatus::OK;
}

RfStatus UartNartisRfComponent::rf_poll_receive_(uint8_t *out, size_t out_cap, size_t *out_len) {
  if (out == nullptr || out_len == nullptr) {
    return RfStatus::ERROR;
  }
  *out_len = 0;
  const uint32_t now = millis();
  const size_t cap = (out_cap < RF_RX_DRAIN_CAP) ? out_cap : RF_RX_DRAIN_CAP;

  // Drain full RX_FIFO_TH chunks (non-blocking; returns 0 when the line is low).
  if (this->rf_rx_accum_len_ + FIFO_TH_VALUE <= cap) {
    const size_t got = this->hal_.drain_rx(out + this->rf_rx_accum_len_, cap - this->rf_rx_accum_len_);
    if (got > 0) {
      this->rf_rx_accum_len_ += got;
      this->rf_rx_last_chunk_ms_ = now;
    }
  }

  if (this->rf_rx_accum_len_ == 0) {
    return RfStatus::NO_DATA;  // nothing yet - RX_RF timeout governs give-up
  }

  // The first received byte is OLEN, so the whole frame is OLEN + 3 bytes (OLEN +
  // content + 2-byte CRC). Fixed-length capture keeps the FIFO fed with noise past
  // the real frame, so once the frame is fully in we return EXACTLY those bytes and
  // drop the trailing noise (a timer would truncate; the byte cap / inter-chunk gap
  // are only fallbacks when OLEN looks bogus and no clean frame is coming).
  const size_t olen = out[0];
  if (olen >= 3 && this->rf_rx_accum_len_ >= olen + 3) {
    *out_len = olen + 3;  // OLEN byte + content + 2-byte CRC (trim noise)
    this->rf_rx_accum_len_ = 0;
    return RfStatus::OK;
  }
  if (this->rf_rx_accum_len_ + FIFO_TH_VALUE > cap ||
      (now - this->rf_rx_last_chunk_ms_) >= RF_RX_END_GAP_MS) {
    *out_len = this->rf_rx_accum_len_;  // fallback: hand over what we have, let the carve scan
    this->rf_rx_accum_len_ = 0;
    return RfStatus::OK;
  }
  return RfStatus::BUSY;  // keep draining until the frame is complete
}

RfStatus UartNartisRfComponent::rf_set_idle_() {
  this->rf_rx_accum_len_ = 0;
  this->hal_.go_standby();
  ESP_LOGVV(TAG, "rf_set_idle_: standby");
  return RfStatus::OK;
}

}  // namespace esphome::uart_nartis_rf
