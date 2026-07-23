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
  ESP_LOGCONFIG(TAG, "  RF frequency: 443.900 MHz (fixed; serial-derived %.3f MHz not yet applied)",
                this->frequency_from_address_() / 1e6f);
  ESP_LOGCONFIG(TAG, "  RX center offset: %d codes (~%+.1f kHz)", this->rx_center_offset_,
                this->rx_center_offset_ * RX_CODE_HZ / 1000.0f);
  ESP_LOGCONFIG(TAG, "  End-of-request gap: %" PRIu32 " ms", this->request_gap_ms_);
  ESP_LOGCONFIG(TAG, "  RF TX timeout: %" PRIu32 " ms", this->rf_tx_timeout_ms_);
  ESP_LOGCONFIG(TAG, "  RF RX timeout: %" PRIu32 " ms", this->rf_rx_timeout_ms_);
  ESP_LOGCONFIG(TAG, "  RF retries: %u (up to %u attempts per request)", this->rf_retries_,
                (unsigned) (this->rf_retries_ + 1));
  ESP_LOGCONFIG(TAG, "  RF: d101-2 PHY + type-5A envelope (CRC-16/X.25) live; TX wraps HDLC, "
                     "RX carves + extracts inner 7E..7E HDLC.");
}

void UartNartisRfComponent::loop() {
  const uint32_t now = millis();

  switch (this->state_) {
    case BridgeState::IDLE:
      // A write while idle moves us to COLLECT; bytes buffered during a previous
      // RF transaction are also picked up here.
      if (this->uart_msg_len_ > 0) {
        this->set_state_(BridgeState::COLLECT);
      }
      break;

    case BridgeState::COLLECT:
      // The request is complete once the upstream flushed, or once the write side
      // has been quiet for request_gap_ms_.
      if (this->uart_msg_len_ > 0 &&
          (this->force_send_ || (now - this->last_write_ms_) >= this->request_gap_ms_)) {
        ESP_LOGD(TAG, "Request complete (%zu bytes)%s", this->uart_msg_len_,
                 this->force_send_ ? " [flush]" : "");
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
          this->enter_fault_("failed to enter RF RX mode");
          break;
        }
        this->set_state_(BridgeState::RX_RF);
      } else if (st == RfStatus::BUSY) {
        if ((now - this->state_enter_ms_) >= this->rf_tx_timeout_ms_) {
          this->enter_fault_("RF TX timeout");
        }
      } else {
        this->enter_fault_("RF TX error");
      }
      break;
    }

    case BridgeState::RX_RF: {
      size_t packet_len = 0;
      const RfStatus st = this->rf_poll_receive_(this->rf_rx_buf_.data(), this->rf_rx_buf_.size(), &packet_len);
      if (st == RfStatus::OK) {
        ESP_LOGD(TAG, "RF reply received (%zu bytes)", packet_len);
        this->finish_rf_rx_(packet_len);
      } else if (st == RfStatus::NO_DATA || st == RfStatus::BUSY) {
        if ((now - this->state_enter_ms_) >= this->rf_rx_timeout_ms_) {
          this->rf_no_reply_count_++;
          this->retry_or_give_up_("no reply");
        }
      } else {
        this->enter_fault_("RF RX error");
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
      this->enter_fault_("unknown state");
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
  if (this->rx_buffer_ == nullptr || this->rx_buffer_->available() < remaining) {
    // Not enough data. The peeked byte (if any) has already been consumed into
    // data[0]; the UARTComponent contract lets callers gate on available() first.
    return false;
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
  // Latch the collected request so it survives across retransmissions, and free
  // uart_msg_buf_ to collect the upstream's next request while this is in flight.
  this->req_len_ = this->uart_msg_len_;
  std::memcpy(this->req_buf_.data(), this->uart_msg_buf_.data(), this->req_len_);
  this->uart_msg_len_ = 0;
  this->tx_attempts_ = 0;
  this->start_tx_attempt_();
}

void UartNartisRfComponent::start_tx_attempt_() {
  // Re-pack the latched request on every attempt (NOT resend the same bytes):
  // when rf_pack_ carries a frame counter / GCM nonce, each attempt must be a
  // fresh valid frame or the meter rejects the retransmission as a replay.
  this->rf_tx_len_ = 0;
  RfStatus st = this->rf_pack_(this->req_buf_.data(), this->req_len_, this->rf_tx_buf_.data(),
                               this->rf_tx_buf_.size(), &this->rf_tx_len_);
  if (st != RfStatus::OK) {
    // Packing is deterministic - retrying won't help, so fail safe.
    this->enter_fault_("RF packing failed");
    return;
  }

  // Count the attempt BEFORE transmitting so a persistent transmit-start
  // failure still advances toward give-up instead of looping forever.
  this->tx_attempts_++;
  st = this->rf_start_transmit_(this->rf_tx_buf_.data(), this->rf_tx_len_);
  if (st != RfStatus::OK) {
    // Couldn't even start the transmit: treat as a failed attempt so ARQ can
    // retry (the radio may have been momentarily busy).
    this->retry_or_give_up_("RF transmit start failed");
    return;
  }
  this->set_state_(BridgeState::TX_RF);
}

void UartNartisRfComponent::retry_or_give_up_(const char *reason) {
  // ARQ: one send plus rf_retries_ retransmissions. tx_attempts_ counts sends
  // already made (>=1 here), so retry while tx_attempts_ <= rf_retries_.
  //
  // NOTE (idempotency): we retransmit on both no-reply AND bad-CRC-reply. A
  // bad-CRC reply means the meter likely received and acted on the request, so
  // resending re-executes it. That is safe for reads (GET) but NOT for writes
  // (SET/ACTION). This bridge targets meter reads; revisit if writes are added.
  if (this->tx_attempts_ <= this->rf_retries_) {
    this->rf_retry_count_++;
    ESP_LOGW(TAG, "RF %s - retransmit (attempt %u/%u)", reason,
             (unsigned) (this->tx_attempts_ + 1), (unsigned) (this->rf_retries_ + 1));
    this->rf_set_idle_();
    this->start_tx_attempt_();  // re-pack (fresh frame) + resend
    return;
  }

  this->rf_giveup_count_++;
  ESP_LOGW(TAG, "RF %s - giving up after %u attempt(s) [no_reply=%" PRIu32 " crc_err=%" PRIu32
                " retries=%" PRIu32 " giveups=%" PRIu32 "]",
           reason, (unsigned) this->tx_attempts_, this->rf_no_reply_count_, this->rf_crc_error_count_,
           this->rf_retry_count_, this->rf_giveup_count_);
  this->req_len_ = 0;
  this->on_rf_timeout_callback_.call();  // no valid reply delivered this request
  this->rf_set_idle_();
  this->set_state_(BridgeState::IDLE);
}

void UartNartisRfComponent::finish_rf_rx_(size_t packet_len) {
  ESP_LOGW(TAG, "RF RX raw [%zu]: %s", packet_len,
           format_hex_pretty(this->rf_rx_buf_.data(), packet_len).c_str());

  size_t unpacked_len = 0;
  const RfStatus st = this->rf_unpack_(this->rf_rx_buf_.data(), packet_len, this->unpack_buf_.data(),
                                       this->unpack_buf_.size(), &unpacked_len);
  if (st != RfStatus::OK) {
    // A corrupt/invalid frame (bad framing or CRC) is a transient RF error -
    // retransmit via ARQ rather than faulting the whole bridge.
    this->rf_crc_error_count_++;
    this->retry_or_give_up_("bad CRC/framing");
    return;
  }

  // Success: the latched request is done.
  this->req_len_ = 0;
  ESP_LOGW(TAG, "RF RX HDLC [%zu]: %s", unpacked_len,
           format_hex_pretty(this->unpack_buf_.data(), unpacked_len).c_str());

  if (unpacked_len > 0 && this->rx_buffer_ != nullptr) {
    const size_t written = this->rx_buffer_->write(this->unpack_buf_.data(), unpacked_len);
    if (written < unpacked_len) {
      ESP_LOGW(TAG, "Reply FIFO overflow, dropped %zu bytes (upstream not reading?)",
               unpacked_len - written);
    }
    ESP_LOGD(TAG, "Queued %zu reply byte(s) for upstream", written);
  }
  this->on_rf_reply_callback_.call();
  this->rf_set_idle_();
  this->set_state_(BridgeState::IDLE);
}

void UartNartisRfComponent::enter_fault_(const char *reason) {
  ESP_LOGE(TAG, "Entering FAULT: %s", reason);
  this->uart_msg_len_ = 0;
  this->req_len_ = 0;
  this->rf_tx_len_ = 0;
  this->force_send_ = false;
  this->set_state_(BridgeState::FAULT);
}

void UartNartisRfComponent::set_state_(BridgeState state) {
  if (this->state_ != state) {
    ESP_LOGV(TAG, "State: %s -> %s", this->state_to_string_(this->state_), this->state_to_string_(state));
    this->state_ = state;
  }
  this->state_enter_ms_ = millis();
}

const char *UartNartisRfComponent::state_to_string_(BridgeState state) const {
  switch (state) {
    case BridgeState::IDLE:
      return "IDLE";
    case BridgeState::COLLECT:
      return "COLLECT";
    case BridgeState::TX_RF:
      return "TX_RF";
    case BridgeState::RX_RF:
      return "RX_RF";
    case BridgeState::FAULT:
      return "FAULT";
    default:
      return "UNKNOWN";
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
// on air, CRC-16/X.25. Framing is a two-level scheme (asymmetric envelope):
//   TX (client->server): 98 F3 | OLEN | 00 01 | HLEN | 5A | serial | hdlc | A5 | CRC
//   RX (server->client): 98 F3 | OLEN | 00 01 | HLEN | hdlc | CRC   (no 5A/serial/A5)
// rf_pack_ builds the request envelope; rf_unpack_ CRC-carves the reply and strips
// the OLEN|00 01|HLEN header to hand the inner HDLC frame back to the upstream.
// ============================================================================

// CRC-16/X.25 (poly 0x1021, init/xorout 0xFFFF, reflected in/out) - the d101-2
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

static constexpr uint8_t D101_SYNC0 = 0x98;
static constexpr uint8_t D101_SYNC1 = 0xF3;

RfStatus UartNartisRfComponent::rf_init_() {
  if (this->pin_sdio_ == nullptr || this->pin_sclk_ == nullptr || this->pin_csb_ == nullptr ||
      this->pin_fcsb_ == nullptr || this->pin_gpio3_ == nullptr) {
    ESP_LOGE(TAG, "rf_init_: CMT2300A pins not configured");
    return RfStatus::ERROR;
  }

  this->hal_.set_pins(this->pin_sdio_, this->pin_sclk_, this->pin_csb_, this->pin_fcsb_, this->pin_gpio3_);

  if (!this->hal_.init()) {
    ESP_LOGE(TAG, "rf_init_: CMT2300A initialization failed - check wiring");
    return RfStatus::ERROR;
  }

  this->derive_serial_le_();

  // NOTE: frequency is FIXED at 443.9 MHz for now. The serial-derived frequency
  // (frequency_from_address_) is computed for logging but not yet applied - the
  // register encoding for arbitrary channels is TODO.
  this->rf_frequency_hz_ = this->frequency_from_address_();
  ESP_LOGI(TAG, "rf_init_: CMT2300A ready at 443.9 MHz (address=%s; serial-derived freq %.3f MHz not yet applied)",
           this->address_.c_str(), this->rf_frequency_hz_ / 1e6f);
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
  // Wrap the DLMS-HDLC frame (a complete 7E..7E frame from the upstream) in the
  // d101-2 RF envelope (request direction, type-5A):
  //   98 F3 | OLEN | 00 01 | HLEN | 5A | serial(6) | <hdlc> | A5 | CRC16(LE)
  //   OLEN = len(00 01 | HLEN | 5A | serial | hdlc | A5)   (all bytes after OLEN, excl. CRC)
  //   HLEN = OLEN - 1   (inner DLMS-HDLC block incl. its 2 CRC bytes)
  //   CRC  = CRC-16/X.25 over [OLEN .. A5], little-endian
  // The 0xA5 terminator is REQUIRED (meter ignores type-5A frames without it).
  // The HAL adds the 0x55 pad and the LSB-first bit-reversal.
  if (payload == nullptr || out == nullptr || out_len == nullptr) {
    return RfStatus::ERROR;
  }
  const size_t olen = 2 + 1 + 1 + 6 + payload_len + 1;  // 00 01 + HLEN + 5A + serial(6) + hdlc + A5
  const size_t hlen = olen - 1;
  const size_t total = 2 + 1 + olen + 2;  // sync(2) + OLEN(1) + content(olen) + CRC(2)
  if (olen > 0xFF || total > out_cap) {
    ESP_LOGW(TAG, "rf_pack_: frame (%zu) exceeds buffer (%zu) or OLEN>255", total, out_cap);
    return RfStatus::ERROR;
  }

  size_t p = 0;
  out[p++] = D101_SYNC0;
  out[p++] = D101_SYNC1;
  out[p++] = (uint8_t) olen;  // OLEN
  out[p++] = 0x00;
  out[p++] = 0x01;
  out[p++] = (uint8_t) hlen;  // HLEN = OLEN - 1
  out[p++] = 0x5A;  // type-5A (DLMS-HDLC, request)
  for (size_t i = 0; i < 6; i++)
    out[p++] = this->serial_le_[i];
  std::memcpy(out + p, payload, payload_len);
  p += payload_len;
  out[p++] = 0xA5;  // REQUIRED transport terminator

  const uint16_t crc = crc16_x25(out + 2, olen + 1);  // OLEN byte + content
  out[p++] = (uint8_t) (crc & 0xFF);
  out[p++] = (uint8_t) ((crc >> 8) & 0xFF);

  *out_len = p;
  ESP_LOGV(TAG, "rf_pack_: %zu hdlc -> %zu on-air bytes (OLEN=%u HLEN=%u)", payload_len, p, (unsigned) olen,
           (unsigned) hlen);
  return RfStatus::OK;
}

RfStatus UartNartisRfComponent::rf_unpack_(const uint8_t *packet, size_t packet_len, uint8_t *out, size_t out_cap,
                                           size_t *out_len) {
  // The chip strips 98 f3, so `packet` starts at the OLEN byte and runs into
  // trailing noise (fixed-length capture). Carve the frame by length + CRC:
  //   OLEN = packet[lp]; CRC-16/X.25 over packet[lp .. lp+OLEN] == next 2 (LE).
  // Scan a few start positions for robustness. Then extract the inner HDLC frame
  // (first 0x7E .. last 0x7E) - the reply envelope carries no 5A/serial header,
  // so we locate the HDLC directly rather than assuming the request layout.
  if (packet == nullptr || out == nullptr || out_len == nullptr) {
    return RfStatus::ERROR;
  }
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

    // CRC-OK. The reply envelope (server->client) is OLEN | 00 01 | HLEN | <hdlc>,
    // with no 5A/serial/A5. Strip the 4-byte transport header (OLEN + 00 01 +
    // HLEN); the remainder of the body is the inner HDLC frame (7E..7E).
    const uint8_t *body = packet + lp;
    const size_t body_len = olen + 1;
    if (body_len >= 6 && body[4] == 0x7E && body[body_len - 1] == 0x7E) {
      const uint8_t *hdlc = body + 4;
      const size_t hdlc_len = body_len - 4;
      if (hdlc_len > out_cap)
        return RfStatus::ERROR;
      std::memcpy(out, hdlc, hdlc_len);
      *out_len = hdlc_len;
      ESP_LOGD(TAG, "rf_unpack_: CRC-OK, %zu-byte HDLC (body %zu, start %zu)", hdlc_len, body_len, lp);
      return RfStatus::OK;
    }
    // CRC-OK but not the expected OLEN|00 01|HLEN|7E..7E layout - hand back the
    // whole body so the upstream can decide (shouldn't happen for a DLMS reply).
    if (body_len > out_cap)
      return RfStatus::ERROR;
    std::memcpy(out, body, body_len);
    *out_len = body_len;
    ESP_LOGW(TAG, "rf_unpack_: CRC-OK but unexpected reply layout (%zu-byte body)", body_len);
    return RfStatus::OK;
  }
  ESP_LOGW(TAG, "rf_unpack_: no CRC-OK frame in %zu bytes", packet_len);
  return RfStatus::ERROR;  // -> ARQ retransmit
}

RfStatus UartNartisRfComponent::rf_start_transmit_(const uint8_t *packet, size_t len) {
  // hal_.transmit() is synchronous: applies the TX profile, pads + bit-reverses,
  // fills the FIFO, GO_TX, and blocks until TX_DONE (or ~600 ms). It blocks the
  // loop only for the frame's airtime (~tens of ms at 1.2 kbps).
  ESP_LOGW(TAG, "RF TX [%zu]: %s", len, format_hex_pretty(packet, len).c_str());
  if (!this->hal_.transmit(packet, len)) {
    ESP_LOGW(TAG, "rf_start_transmit_: TX did not complete");
    return RfStatus::ERROR;
  }
  return RfStatus::OK;
}

RfStatus UartNartisRfComponent::rf_transmit_done_() {
  // hal_.transmit() already blocked until TX_DONE, so TX is complete here.
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
  ESP_LOGV(TAG, "rf_enter_rx_mode_: RX armed (center offset %d codes)", this->rx_center_offset_);
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
  ESP_LOGV(TAG, "rf_set_idle_: standby");
  return RfStatus::OK;
}

}  // namespace esphome::uart_nartis_rf
