#include "ac_hi.h"
#include <cmath>
#include <algorithm>

namespace esphome {
namespace ac_hi {

// ---- Encoding helpers ----
uint8_t ACHIClimate::encode_mode_hi_(climate::ClimateMode m) {
  uint8_t code = 4; // auto=4 in legacy order fan(0), heat(1), cool(2), dry(3), auto(4)
  switch (m) {
    case climate::CLIMATE_MODE_FAN_ONLY: code = 0; break;
    case climate::CLIMATE_MODE_HEAT:     code = 1; break;
    case climate::CLIMATE_MODE_COOL:     code = 2; break;
    case climate::CLIMATE_MODE_DRY:      code = 3; break;
    case climate::CLIMATE_MODE_AUTO:     code = 4; break;
    default:                              code = 2; break;
  }
  uint8_t odd = static_cast<uint8_t>((code << 1) | 0x01); // 1,3,5,7,9
  return static_cast<uint8_t>(odd << 4);
}

uint8_t ACHIClimate::encode_power_lo_(bool on) {
  return on ? 0x0C : 0x04;
}

uint8_t ACHIClimate::encode_target_temp_(uint8_t c) {
  c = clamp16_30_(c);
  return static_cast<uint8_t>((c << 1) | 0x01);
}

climate::ClimateMode ACHIClimate::decode_mode_from_hi_(uint8_t b18_hi) {
  uint8_t odd = b18_hi & 0x0F;     // already shifted
  uint8_t code = static_cast<uint8_t>(odd >> 1);
  switch (code) {
    case 0: return climate::CLIMATE_MODE_FAN_ONLY;
    case 1: return climate::CLIMATE_MODE_HEAT;
    case 2: return climate::CLIMATE_MODE_COOL;
    case 3: return climate::CLIMATE_MODE_DRY;
    case 4: return climate::CLIMATE_MODE_AUTO;
    default: return climate::CLIMATE_MODE_COOL;
  }
}

bool ACHIClimate::decode_power_from_lo_(uint8_t b18_lo) {
  return (b18_lo & 0x08) != 0;
}

uint8_t ACHIClimate::encode_fan_byte_(climate::ClimateFanMode f) {
  // AUTO->1, QUIET->10, LOW->12, MED->14, HIGH->16 ; device expects +1 on write
  uint8_t code = 1;
  switch (f) {
    case climate::CLIMATE_FAN_AUTO:   code = 1;  break;
    case climate::CLIMATE_FAN_QUIET:  code = 10; break;
    case climate::CLIMATE_FAN_LOW:    code = 12; break;
    case climate::CLIMATE_FAN_MEDIUM: code = 14; break;
    case climate::CLIMATE_FAN_HIGH:   code = 16; break;
    default:                          code = 1;  break;
  }
  return static_cast<uint8_t>(code + 1);
}

uint8_t ACHIClimate::encode_sleep_byte_(uint8_t stage) {
  // 0(off),1,2,4,8 then <<1 | 1
  uint8_t code = 0;
  switch (stage) {
    case 0: code = 0; break;
    case 1: code = 1; break;
    case 2: code = 2; break;
    case 3: code = 4; break;
    case 4: code = 8; break;
    default: code = 0; break;
  }
  return static_cast<uint8_t>((code << 1) | 0x01);
}

uint8_t ACHIClimate::encode_swing_ud_(bool on) {
  return on ? 0b11000000 : 0b01000000;
}
uint8_t ACHIClimate::encode_swing_lr_(bool on) {
  return on ? 0b00110000 : 0b00010000;
}

// ---- Traits ----
climate::ClimateTraits ACHIClimate::traits() {
  climate::ClimateTraits t;
  t.set_supported_modes({
    climate::CLIMATE_MODE_OFF,
    climate::CLIMATE_MODE_FAN_ONLY,
    climate::CLIMATE_MODE_HEAT,
    climate::CLIMATE_MODE_COOL,
    climate::CLIMATE_MODE_DRY,
    climate::CLIMATE_MODE_AUTO,
  });
  t.set_supported_fan_modes({
    climate::CLIMATE_FAN_AUTO,
    climate::CLIMATE_FAN_QUIET,
    climate::CLIMATE_FAN_LOW,
    climate::CLIMATE_FAN_MEDIUM,
    climate::CLIMATE_FAN_HIGH,
  });
  t.set_supported_swing_modes({
    climate::CLIMATE_SWING_OFF,
    climate::CLIMATE_SWING_VERTICAL,
    climate::CLIMATE_SWING_HORIZONTAL,
    climate::CLIMATE_SWING_BOTH,
  });
  if (enable_presets_) {
    t.set_supported_presets({
      climate::CLIMATE_PRESET_NONE,
      climate::CLIMATE_PRESET_BOOST,
      climate::CLIMATE_PRESET_ECO,
      climate::CLIMATE_PRESET_SLEEP
    });
  }
  t.set_visual_min_temperature(16);
  t.set_visual_max_temperature(30);
  t.set_visual_temperature_step(1.0f);
  t.set_supports_current_temperature(true);
  return t;
}

// ---- Control (no optimistic publish) ----
void ACHIClimate::control(const climate::ClimateCall &call) {
  bool changed = false;

  if (call.get_mode().has_value()) {
    auto m = *call.get_mode();
    if (m == climate::CLIMATE_MODE_OFF) {
      this->power_on_ = false;
    } else {
      this->power_on_ = true;
      this->mode_ = m;
    }
    changed = true;
  }

  if (call.get_target_temperature().has_value()) {
    uint8_t c = static_cast<uint8_t>(std::lround(*call.get_target_temperature()));
    this->target_c_ = clamp16_30_(c);
    changed = true;
  }

  if (call.get_fan_mode().has_value()) {
    this->fan_ = *call.get_fan_mode();
    changed = true;
  }

  if (call.get_swing_mode().has_value()) {
    this->swing_ = *call.get_swing_mode();
    changed = true;
  }

  if (call.get_preset().has_value()) {
    auto p = *call.get_preset();
    this->turbo_ = (p == climate::CLIMATE_PRESET_BOOST);
    this->eco_   = (p == climate::CLIMATE_PRESET_ECO);
    this->sleep_stage_ = (p == climate::CLIMATE_PRESET_SLEEP) ? 1 : 0;
    changed = true;
  }

  if (!changed) return;

  // Coalesce and respect ACK
  this->dirty_ = true;
  if (this->writing_lock_) return;

  const uint32_t now = millis();
  if (now - this->last_tx_ms_ < kMinGapMs) return;

  this->send_now_();
}

// ---- Transport ----
void ACHIClimate::calc_and_patch_crc_(std::vector<uint8_t> &buf) const {
  // Simple 16-bit additive checksum over bytes [3 .. size-5], as per device examples.
  if (buf.size() < 8) return;
  uint16_t sum = 0;
  for (size_t i = 3; i + 5 <= buf.size(); i++) sum = static_cast<uint16_t>(sum + buf[i]);
  size_t n = buf.size();
  buf[n - 4] = static_cast<uint8_t>((sum >> 8) & 0xFF);
  buf[n - 3] = static_cast<uint8_t>(sum & 0xFF);
}

void ACHIClimate::send_write_frame_(const std::vector<uint8_t> &frame) {
  for (uint8_t b : frame) this->write_byte(b);
  this->flush();
}

void ACHIClimate::send_query_status_() {
  for (uint8_t b : this->query_) this->write_byte(b);
  this->flush();
}

bool ACHIClimate::extract_next_frame_(std::vector<uint8_t> &out) {
  // Find header
  size_t i = rx_start_;
  const size_t N = rx_.size();
  while (i + 1 < N && !(rx_[i] == HI_HDR0 && rx_[i+1] == HI_HDR1)) ++i;
  if (i + 1 >= N) { rx_start_ = i; return false; }
  size_t j = i + 2;
  // Find tail
  while (j + 1 < N && !(rx_[j] == HI_TAIL0 && rx_[j+1] == HI_TAIL1)) ++j;
  if (j + 1 >= N) { rx_start_ = i; return false; }
  out.assign(rx_.begin() + i, rx_.begin() + j + 2);
  rx_start_ = j + 2;
  return true;
}

void ACHIClimate::handle_frame_(const std::vector<uint8_t> &frame) {
  if (frame.size() < 16) return;
  uint8_t cmd = frame[13]; // position where command resides in our templates
  if (cmd == CMD_STATUS) {
    this->parse_status_102_(frame);
  } else if (cmd == CMD_WRITE) {
    this->handle_ack_101_();
  }
}

void ACHIClimate::handle_ack_101_() {
  this->writing_lock_ = false;
  // If anything changed while we waited — send newest snapshot
  if (this->dirty_) {
    const uint32_t now = millis();
    if (now - this->last_tx_ms_ >= kMinGapMs) this->send_now_();
  }
}

void ACHIClimate::parse_status_102_(const std::vector<uint8_t> &b) {
  if (b.size() <= static_cast<size_t>(IDX_LED)) return; // sanity

  // Power / mode
  uint8_t b18 = b[IDX_MODE_POWER];
  bool power = decode_power_from_lo_(static_cast<uint8_t>(b18 & 0x0F));
  climate::ClimateMode mode = decode_mode_from_hi_(static_cast<uint8_t>(b18 >> 4));

  this->power_on_ = power;
  this->mode_ = mode;

  // Target temp — device reports either coded or plain; accept both
  uint8_t raw = b[IDX_TARGET_TEMP];
  if (raw >= 16 && raw <= 30) {
    this->target_c_ = raw;
  } else {
    this->target_c_ = static_cast<uint8_t>(clamp16_30_(raw >> 1));
  }
  this->target_temperature = this->target_c_;

  // Current temperatures
  this->current_temperature = b[IDX_AIR_TEMP];
#ifdef USE_SENSOR
  if (this->pipe_sensor_ != nullptr) this->pipe_sensor_->publish_state(b[IDX_PIPE_TEMP]);
#endif

  // Fan
  uint8_t rf = b[IDX_FAN];
  if      (rf ==  1 || rf ==  2) this->fan_ = climate::CLIMATE_FAN_AUTO;
  else if (rf == 11)             this->fan_ = climate::CLIMATE_FAN_QUIET;
  else if (rf == 13)             this->fan_ = climate::CLIMATE_FAN_LOW;
  else if (rf == 15)             this->fan_ = climate::CLIMATE_FAN_MEDIUM;
  else if (rf == 17)             this->fan_ = climate::CLIMATE_FAN_HIGH;

  // Sleep
  uint8_t rs = b[IDX_SLEEP];
  uint8_t sc = (rs >> 1);
  if      (sc == 0) this->sleep_stage_ = 0;
  else if (sc == 1) this->sleep_stage_ = 1;
  else if (sc == 2) this->sleep_stage_ = 2;
  else if (sc == 4) this->sleep_stage_ = 3;
  else if (sc == 8) this->sleep_stage_ = 4;
  else              this->sleep_stage_ = 0;

  // Flags / quiet / led / swing (best-effort decode)
  this->turbo_ = (b[IDX_FLAGS]  & 0b00000010) != 0;
  this->eco_   = (b[IDX_FLAGS]  & 0b00000100) != 0;
  this->quiet_ = (b[IDX_FLAGS2] & 0b00110000) == 0b00110000;
  this->led_   = (b[IDX_LED]    & 0b10000000) != 0;

  bool updown    = (b[IDX_FLAGS2] & 0b10000000) != 0;
  bool leftright = (b[IDX_FLAGS2] & 0b01000000) != 0;
  if (updown && leftright) this->swing_ = climate::CLIMATE_SWING_BOTH;
  else if (updown)         this->swing_ = climate::CLIMATE_SWING_VERTICAL;
  else if (leftright)      this->swing_ = climate::CLIMATE_SWING_HORIZONTAL;
  else                     this->swing_ = climate::CLIMATE_SWING_OFF;

  // Publish real state (no optimistic)
  this->mode = this->power_on_ ? this->mode_ : climate::CLIMATE_MODE_OFF;
  this->fan_mode = this->fan_;
  this->swing_mode = this->swing_;
  if (enable_presets_) {
    if (this->turbo_) this->preset = climate::CLIMATE_PRESET_BOOST;
    else if (this->eco_) this->preset = climate::CLIMATE_PRESET_ECO;
    else if (this->sleep_stage_ > 0) this->preset = climate::CLIMATE_PRESET_SLEEP;
    else this->preset = climate::CLIMATE_PRESET_NONE;
  }
  this->publish_state();
}

void ACHIClimate::send_now_() {
  // Build frame snapshot using legacy encoding we agreed on
  std::vector<uint8_t> frame = this->tx_bytes_;

  uint8_t lo = encode_power_lo_(this->power_on_);
  uint8_t hi = encode_mode_hi_(this->mode_);
  frame[IDX_MODE_POWER]  = static_cast<uint8_t>(hi | lo);
  frame[IDX_TARGET_TEMP] = encode_target_temp_(this->target_c_);

  frame[IDX_FAN]   = encode_fan_byte_(this->fan_);
  frame[IDX_SLEEP] = encode_sleep_byte_(this->sleep_stage_);

  bool v = (this->swing_ == climate::CLIMATE_SWING_VERTICAL) || (this->swing_ == climate::CLIMATE_SWING_BOTH);
  bool h = (this->swing_ == climate::CLIMATE_SWING_HORIZONTAL) || (this->swing_ == climate::CLIMATE_SWING_BOTH);
  frame[IDX_SWING] = static_cast<uint8_t>(encode_swing_ud_(v) | encode_swing_lr_(h));

  // Turbo/Eco flags (mutually exclusive here)
  if (this->turbo_)      frame[IDX_FLAGS] = 0b00000010;
  else if (this->eco_)   frame[IDX_FLAGS] = 0b00000100;
  else                   frame[IDX_FLAGS] = 0;

  // Quiet (mirrors fan quiet)
  frame[IDX_FLAGS2] = this->quiet_ || (this->fan_ == climate::CLIMATE_FAN_QUIET) ? 0b00110000 : 0;

  // LED state
  frame[IDX_LED] = this->led_ ? 0b11000000 : 0b01000000;

  // Patch checksum and send
  this->calc_and_patch_crc_(frame);
  this->send_write_frame_(frame);

  const uint32_t now = millis();
  this->last_tx_ms_ = now;
  this->ack_deadline_ms_ = now + kAckTimeoutMs;
  this->writing_lock_ = true;
  this->dirty_ = false;
}

// ---- Loop / Update ----
void ACHIClimate::loop() {
  // Read incoming bytes into buffer
  uint8_t byte;
  while (this->read_byte(&byte)) rx_.push_back(byte);

  // Compact prefix that we've consumed
  if (rx_start_ > 4096) {
    rx_.erase(rx_.begin(), rx_.begin() + static_cast<std::ptrdiff_t>(rx_start_));
    rx_start_ = 0;
  }

  // Try to parse as many complete frames as present
  std::vector<uint8_t> frame;
  uint32_t start_ms = millis();
  while (millis() - start_ms < 20) {
    if (!this->extract_next_frame_(frame)) break;
    this->handle_frame_(frame);
  }
}

void ACHIClimate::update() {
  const uint32_t now = millis();

  // ACK timeout handling
  if (this->writing_lock_ && now > this->ack_deadline_ms_) {
    this->writing_lock_ = false;
  }

  // If something changed and we can send — do it
  if (!this->writing_lock_ && this->dirty_ && (now - this->last_tx_ms_ >= kMinGapMs)) {
    this->send_now_();
    return;
  }

  // Otherwise, poll status
  if (!this->writing_lock_) this->send_query_status_();
}

}  // namespace ac_hi
}  // namespace esphome
