#include "ac_hi.h"
#include <algorithm>
#include <cmath>

namespace esphome {
namespace ac_hi {

static const char *const TAG = "ac_hi";

// ---------- Encoding helpers ----------
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
  return static_cast<uint8_t>(code + 0); // already encoded value used by protocol
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

uint8_t ACHIClimate::encode_swing_ud_(bool on) { return on ? 0b11000000 : 0b01000000; }
uint8_t ACHIClimate::encode_swing_lr_(bool on) { return on ? 0b00110000 : 0b00010000; }

// Compare reported STATUS with pending desired values.
// We don't require LED to match to avoid spurious mismatches on models that don't echo it.
bool ACHIClimate::reported_matches_pending_(bool rep_power,
                                            climate::ClimateMode rep_mode,
                                            uint8_t rep_set_c,
                                            climate::ClimateFanMode rep_fan,
                                            climate::ClimateSwingMode rep_swing,
                                            bool rep_turbo,
                                            bool rep_eco,
                                            bool rep_quiet,
                                            bool rep_led) const {
  if (!this->have_pending_) return false;
  if (rep_power != this->pending_power_) return false;
  if (rep_power) { // only check deep fields if powered ON
    if (rep_mode != this->pending_mode_) return false;
    if (rep_set_c != this->pending_target_c_) return false;
    if (rep_fan != this->pending_fan_) return false;
    if (rep_swing != this->pending_swing_) return false;
    // Turbo/Eco/Quiet may be model-dependent; only enforce when our pending flag is ON
    if (this->pending_turbo_ && !rep_turbo) return false;
    if (this->pending_eco_ && !rep_eco) return false;
    if (this->pending_quiet_ && !rep_quiet) return false;
  }
  return true;
}

// ---------- Traits ----------
climate::ClimateTraits ACHIClimate::traits() {
  climate::ClimateTraits t;
  t.set_supported_modes({
    climate::CLIMATE_MODE_OFF,
    climate::CLIMATE_MODE_FAN_ONLY,
    climate::CLIMATE_MODE_HEAT,
    climate::CLIMATE_MODE_COOL,
    climate::CLIMATE_MODE_DRY,
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
      climate::CLIMATE_PRESET_SLEEP,
    });
  }
  t.set_visual_min_temperature(16);
  t.set_visual_max_temperature(30);
  t.set_visual_temperature_step(1.0f);
  return t;
}

// ---------- Control ----------
void ACHIClimate::control(const climate::ClimateCall &call) {
  bool changed = false;

  if (call.get_mode().has_value()) {
    auto m = *call.get_mode();
    if (m == climate::CLIMATE_MODE_OFF) {
      if (this->power_on_) changed = true;
      this->power_on_ = false;
    } else {
      if (!this->power_on_ || this->mode_ != m) changed = true;
      this->power_on_ = true;
      this->mode_ = m;
    }
    ESP_LOGD(TAG, "control(): mode=%d (power_on=%s)", (int)m, this->power_on_ ? "YES" : "NO");
  }

  if (call.get_target_temperature().has_value()) {
    uint8_t c = static_cast<uint8_t>(std::lround(*call.get_target_temperature()));
    c = clamp16_30_(c);
    if (c != this->target_c_) changed = true;
    this->target_c_ = c;
    ESP_LOGD(TAG, "control(): target_temp=%u", (unsigned)this->target_c_);
  }

  if (call.get_fan_mode().has_value()) {
    auto f = *call.get_fan_mode();
    if (f != this->fan_) changed = true;
    this->fan_ = f;
    ESP_LOGD(TAG, "control(): fan=%d", (int)f);
  }

  if (call.get_swing_mode().has_value()) {
    auto s = *call.get_swing_mode();
    if (s != this->swing_) changed = true;
    this->swing_ = s;
    ESP_LOGD(TAG, "control(): swing=%d", (int)s);
  }

  if (call.get_preset().has_value()) {
    auto p = *call.get_preset();
    if (p == climate::CLIMATE_PRESET_BOOST) { turbo_ = true; eco_ = false; }
    else if (p == climate::CLIMATE_PRESET_ECO) { turbo_ = false; eco_ = true; }
    else { turbo_ = false; eco_ = false; }
    changed = true;
  }

  if (changed) {
    this->dirty_ = true;
  }
}

// ---------- CRC helpers ----------
// For all CRC calculations we sum over bytes [3 .. size-4) as observed in logs

void ACHIClimate::calc_and_patch_crc1_(std::vector<uint8_t> &buf) const {
  if (buf.size() < 8) return;
  uint8_t sum = 0;
  size_t crc_lo_pos = buf.size() - 4; // we'll store 0,sum,F4,FB
  for (size_t i = 3; i < crc_lo_pos; i++) sum = static_cast<uint8_t>(sum + buf[i]);
  buf[buf.size() - 4] = 0x00;
  buf[buf.size() - 3] = sum;
}

void ACHIClimate::calc_and_patch_crc16_sum_(std::vector<uint8_t> &buf) const {
  if (buf.size() < 10) return;
  uint16_t sum = 0;
  size_t crc_lo_pos = buf.size() - 4;
  for (size_t i = 3; i < crc_lo_pos; i++) sum = static_cast<uint16_t>(sum + buf[i]);
  buf[buf.size() - 4] = static_cast<uint8_t>(sum & 0xFF);
  buf[buf.size() - 3] = static_cast<uint8_t>((sum >> 8) & 0xFF);
}

void ACHIClimate::calc_and_patch_crc16_modbus_(std::vector<uint8_t> &buf) const {
  if (buf.size() < 10) return;
  uint16_t crc = 0xFFFF;
  size_t crc_lo_pos = buf.size() - 4;
  for (size_t i = 3; i < crc_lo_pos; i++) {
    crc ^= static_cast<uint16_t>(buf[i]);
    for (int b = 0; b < 8; b++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else crc >>= 1;
    }
  }
  buf[buf.size() - 4] = static_cast<uint8_t>(crc & 0xFF);        // LO first
  buf[buf.size() - 3] = static_cast<uint8_t>((crc >> 8) & 0xFF); // HI
}

void ACHIClimate::calc_and_patch_crc16_ccitt_(std::vector<uint8_t> &buf) const {
  if (buf.size() < 10) return;
  uint16_t crc = 0xFFFF;
  size_t crc_lo_pos = buf.size() - 4;
  for (size_t i = 3; i < crc_lo_pos; i++) {
    crc ^= (uint16_t)buf[i] << 8;
    for (int b = 0; b < 8; b++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc <<= 1;
    }
  }
  uint16_t out = crc;
  buf[buf.size() - 4] = static_cast<uint8_t>(out & 0xFF);        // LO
  buf[buf.size() - 3] = static_cast<uint8_t>((out >> 8) & 0xFF); // HI
}

// ---------- Build variant ----------
void ACHIClimate::build_variant_(uint8_t attempt, std::vector<uint8_t> &frame) {
  // Variants (0..7):
  // 0: len=total-9, CRC16 SUM
  // 1: len=total-9, CRC8 SUM
  // 2: len=total-9, CRC16 MODBUS
  // 3: len=total-9, CRC16 CCITT
  // 4: len=total,   CRC16 SUM
  // 5: len=total,   CRC8 SUM
  // 6: len=total,   CRC16 MODBUS
  // 7: len=total,   CRC16 CCITT
  bool len_is_total = (attempt >= 4);
  uint8_t len_value = len_is_total ? static_cast<uint8_t>(frame.size()) : static_cast<uint8_t>(frame.size() - 9);
  frame[4] = len_value;

  // Zero CRC bytes first
  frame[frame.size() - 4] = 0x00;
  frame[frame.size() - 3] = 0x00;

  uint8_t scheme = attempt % 4;
  switch (scheme) {
    case 0:
      this->calc_and_patch_crc16_sum_(frame);
      ESP_LOGD(TAG, "build: variant #%u len=0x%02X (mode=%s), CRC=SUM16",
               attempt, len_value, len_is_total ? "TOTAL" : "TOTAL-9");
      break;
    case 1:
      this->calc_and_patch_crc1_(frame);
      ESP_LOGD(TAG, "build: variant #%u len=0x%02X (mode=%s), CRC=SUM8",
               attempt, len_value, len_is_total ? "TOTAL" : "TOTAL-9");
      break;
    case 2:
      this->calc_and_patch_crc16_modbus_(frame);
      ESP_LOGD(TAG, "build: variant #%u len=0x%02X (mode=%s), CRC=MODBUS",
               attempt, len_value, len_is_total ? "TOTAL" : "TOTAL-9");
      break;
    case 3:
      this->calc_and_patch_crc16_ccitt_(frame);
      ESP_LOGD(TAG, "build: variant #%u len=0x%02X (mode=%s), CRC=CCITT",
               attempt, len_value, len_is_total ? "TOTAL" : "TOTAL-9");
      break;
  }

  // Debug CRC bytes
  ESP_LOGD(TAG, "build: len_byte[4]=0x%02X, crc_lo[%u]=0x%02X crc_hi[%u]=0x%02X",
           frame[4], (unsigned)(frame.size()-4), frame[frame.size()-4],
           (unsigned)(frame.size()-3), frame[frame.size()-3]);
}

// ---------- Transport helpers ----------
void ACHIClimate::send_write_frame_(const std::vector<uint8_t> &frame) {
  for (uint8_t b : frame) this->write_byte(b);
}

void ACHIClimate::send_query_status_() {
  for (uint8_t b : this->query_) this->write_byte(b);
  ESP_LOGV(TAG, "poll: status query sent");
}

// Build snapshot and send (with variants)
void ACHIClimate::send_now_() {
  // Prepare base frame of 41 bytes: ensure template size is 41
  std::vector<uint8_t> frame = this->write_template_;
  // Ensure tail is present
  if (frame.size() < 41) frame.resize(41, 0x00);
  frame[0] = HI_HDR0; frame[1] = HI_HDR1;
  frame[13] = CMD_WRITE;
  frame[frame.size()-2] = HI_TAIL0;
  frame[frame.size()-1] = HI_TAIL1;

  // Fill fields according to mapping
  // FAN
  frame[IDX_FAN] = encode_fan_byte_(this->fan_);
  // SLEEP
  frame[IDX_SLEEP] = encode_sleep_byte_(this->sleep_stage_);
  // MODE|POWER
  uint8_t mode_hi = encode_mode_hi_write_legacy_(this->mode_);
  uint8_t power_lo = encode_power_lo_write_(this->power_on_);
  frame[IDX_MODE_POWER] = static_cast<uint8_t>(mode_hi | power_lo);
  // TARGET
  frame[IDX_TARGET_TEMP] = encode_target_temp_write_legacy_(this->target_c_);
  // SWING bits
  bool v = (this->swing_ == climate::CLIMATE_SWING_VERTICAL) || (this->swing_ == climate::CLIMATE_SWING_BOTH);
  bool h = (this->swing_ == climate::CLIMATE_SWING_HORIZONTAL) || (this->swing_ == climate::CLIMATE_SWING_BOTH);
  frame[IDX_SWING] = static_cast<uint8_t>(encode_swing_ud_(v) | encode_swing_lr_(h));
  // FLAGS
  if (this->turbo_)      frame[IDX_FLAGS] = 0b00000010;
  else if (this->eco_)   frame[IDX_FLAGS] = 0b00000100;
  else                   frame[IDX_FLAGS] = 0x00;
  // Quiet mapped via FLAGS2
  this->quiet_ = (this->fan_ == climate::CLIMATE_FAN_QUIET);
  frame[IDX_FLAGS2] = this->quiet_ ? 0b00110000 : 0x00;
  // LED
  frame[IDX_LED] = this->led_ ? 0b11000000 : 0b01000000;

  // Capture pending desired snapshot for implicit ACK via STATUS
  this->have_pending_   = true;
  this->pending_power_  = this->power_on_;
  this->pending_mode_   = this->mode_;
  this->pending_target_c_ = this->target_c_;
  this->pending_fan_    = this->fan_;
  this->pending_swing_  = this->swing_;
  this->pending_turbo_  = this->turbo_;
  this->pending_eco_    = this->eco_;
  this->pending_quiet_  = this->quiet_;
  this->pending_led_    = this->led_;

  // Prefer starting from legacy-friendly variant (len=TOTAL) on a fresh write
  if (this->write_attempt_ == 0) {
    this->write_attempt_ = kInitialAttempt;
  }

  uint8_t attempt = this->write_attempt_;
  this->build_variant_(attempt, frame);

  ESP_LOGD(TAG, "build(write-legacy): b18=0x%02X (mode_hi=0x%02X power_lo=0x%02X) b19=0x%02X (2*C+1)",
           frame[IDX_MODE_POWER], mode_hi, power_lo, frame[IDX_TARGET_TEMP]);

  // Dump frame (limited)
  char hexbuf[512]; size_t p = 0;
  for (size_t i = 0; i < frame.size() && p + 4 < sizeof(hexbuf); i++) {
    p += snprintf(hexbuf + p, sizeof(hexbuf) - p, "%02X ", frame[i]);
  }
  ESP_LOGD(TAG, "TX write (%u bytes): %s", (unsigned)frame.size(), hexbuf);

  // Send
  this->send_write_frame_(frame);

  const uint32_t now = millis();
  this->last_tx_ms_ = now;
  this->ack_deadline_ms_ = now + kAckTimeoutMs;
  this->force_poll_at_ms_ = now + kForcePollDelayMs;
  this->writing_lock_ = true;
  this->dirty_ = false;
}

// ---------- RX / parsing ----------
bool ACHIClimate::extract_next_frame_(std::vector<uint8_t> &out) {
  size_t i = rx_start_;
  const size_t N = rx_.size();

  // find header
  while (i + 1 < N && !(rx_[i] == HI_HDR0 && rx_[i + 1] == HI_HDR1)) ++i;
  if (i + 1 >= N) { rx_start_ = i; return false; }

  // find tail from i+2 onwards
  size_t j = i + 2;
  while (j + 1 < N && !(rx_[j] == HI_TAIL0 && rx_[j + 1] == HI_TAIL1)) ++j;
  if (j + 1 >= N) { rx_start_ = i; return false; }

  // copy frame [i .. j+1]
  out.assign(rx_.begin() + i, rx_.begin() + j + 2);
  rx_start_ = j + 2;
  // truncate buffer periodically
  if (rx_start_ > 1024) {
    rx_.erase(rx_.begin(), rx_.begin() + rx_start_);
    rx_start_ = 0;
  }
  return true;
}

void ACHIClimate::handle_frame_(const std::vector<uint8_t> &frame) {
  if (frame.size() < 16) return;
  uint8_t cmd = frame[13];
  if (cmd == CMD_STATUS) {
    ESP_LOGV(TAG, "RX frame: STATUS (0x66), len=%u", (unsigned)frame.size());
    this->parse_status_102_(frame);
  } else if (cmd == CMD_WRITE) {
    ESP_LOGV(TAG, "RX frame: ACK (0x65)");
    this->handle_ack_101_();
  } else if (cmd == CMD_NAK) {
    ESP_LOGW(TAG, "RX frame: NAK (0xFD) — will try next variant");
    this->handle_nak_fd_();
  } else {
    ESP_LOGV(TAG, "RX frame: unknown cmd=0x%02X len=%u", cmd, (unsigned)frame.size());
  }
}

void ACHIClimate::parse_status_102_(const std::vector<uint8_t> &b) {
  if (b.size() <= (size_t)IDX_LED) return;

  this->last_status_ms_ = millis();

  uint8_t b18 = b[IDX_MODE_POWER];
  uint8_t lo = (uint8_t)(b18 & 0x0F);
  uint8_t hi = (uint8_t)((b18 >> 4) & 0x0F);

  bool power = (lo & 0x08) != 0;
  this->power_on_ = power;

  climate::ClimateMode m;
  if      (hi == 0x00) m = climate::CLIMATE_MODE_FAN_ONLY;
  else if (hi == 0x01) m = climate::CLIMATE_MODE_HEAT;
  else if (hi == 0x02) m = climate::CLIMATE_MODE_COOL;
  else if (hi == 0x03) m = climate::CLIMATE_MODE_DRY;
  else                 m = this->mode_;
  this->mode_ = m;

  uint8_t raw_set = b[IDX_TARGET_TEMP];
  uint8_t set_c = (raw_set >= 16 && raw_set <= 30) ? raw_set : (uint8_t)(raw_set >> 1);
  this->target_c_ = clamp16_30_(set_c);
  this->target_temperature = this->target_c_;

  this->current_temperature = b[IDX_AIR_TEMP];
#ifdef USE_SENSOR
  if (this->pipe_sensor_ != nullptr) this->pipe_sensor_->publish_state(b[IDX_PIPE_TEMP]);
#endif

  uint8_t rf = b[IDX_FAN];
  climate::ClimateFanMode fan = climate::CLIMATE_FAN_AUTO;
  if      (rf ==  1 || rf ==  2) fan = climate::CLIMATE_FAN_AUTO;
  else if (rf == 11)             fan = climate::CLIMATE_FAN_QUIET;
  else if (rf == 13)             fan = climate::CLIMATE_FAN_LOW;
  else if (rf == 15)             fan = climate::CLIMATE_FAN_MEDIUM;
  else if (rf == 17)             fan = climate::CLIMATE_FAN_HIGH;
  this->fan_ = fan;

  uint8_t rs = b[IDX_SLEEP];
  uint8_t sc = (uint8_t)(rs >> 1);
  uint8_t sleep_stage = 0;
  if      (sc == 0) sleep_stage = 0;
  else if (sc == 1) sleep_stage = 1;
  else if (sc == 2) sleep_stage = 2;
  else if (sc == 4) sleep_stage = 3;
  else if (sc == 8) sleep_stage = 4;
  this->sleep_stage_ = sleep_stage;

  bool turbo = (b[IDX_FLAGS]  & 0b00000010) != 0;
  bool eco   = (b[IDX_FLAGS]  & 0b00000100) != 0;
  bool quiet = (b[IDX_FLAGS2] & 0b00110000) == 0b00110000;
  bool led   = (b[IDX_LED]    & 0b10000000) != 0;
  this->turbo_ = turbo;
  this->eco_   = eco;
  this->quiet_ = quiet;
  this->led_   = led;

  bool swing_v = (b[IDX_SWING]  & 0b10000000) != 0;
  bool swing_h = (b[IDX_FLAGS2] & 0b01000000) != 0;
  climate::ClimateSwingMode swing_mode = climate::CLIMATE_SWING_OFF;
  if (swing_v && swing_h) swing_mode = climate::CLIMATE_SWING_BOTH;
  else if (swing_v)       swing_mode = climate::CLIMATE_SWING_VERTICAL;
  else if (swing_h)       swing_mode = climate::CLIMATE_SWING_HORIZONTAL;
  this->swing_ = swing_mode;

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

  ESP_LOGD(TAG, "RX status: b18=0x%02X (hi=0x%X lo=0x%X) set=%u air=%u pipe=%u fan=%u flags=0x%02X led=0x%02X",
           b18, hi, lo, (unsigned)this->target_c_,
           (unsigned)b[IDX_AIR_TEMP], (unsigned)b[IDX_PIPE_TEMP],
           (unsigned)rf, (unsigned)b[IDX_FLAGS], (unsigned)b[IDX_LED]);

  // STATUS: only treat as implicit ACK if it reflects our pending desired state
  if (this->writing_lock_) {
    if (this->reported_matches_pending_(power, m, this->target_c_, fan, swing_mode, turbo, eco, quiet, led)) {
      ESP_LOGV(TAG, "STATUS matches pending command → clearing lock (implicit ACK)");
      this->writing_lock_ = false;
      this->write_attempt_ = 0;
      this->have_pending_ = false;
    } else {
      ESP_LOGV(TAG, "STATUS received while waiting ACK but state differs → keep waiting");
    }
  }
}

void ACHIClimate::handle_ack_101_() {
  if (!this->writing_lock_) return;
  ESP_LOGV(TAG, "ACK: clearing lock");
  this->writing_lock_ = false;
  this->write_attempt_ = 0;
  this->have_pending_ = false;
}

void ACHIClimate::handle_nak_fd_() {
  // negative ACK — try next variant
  if (!this->writing_lock_) return;
  if (this->write_attempt_ + 1 < kWriteAttemptsMax) {
    this->write_attempt_++;
    ESP_LOGW(TAG, "NAK: retry with variant #%u", (unsigned)this->write_attempt_);
    const uint32_t now = millis();
    if (now - this->last_tx_ms_ >= kMinGapMs) {
      this->send_now_();
    } else {
      // send later in update()
      this->dirty_ = true;
      this->writing_lock_ = false;
    }
  } else {
    ESP_LOGE(TAG, "NAK: all variants exhausted; giving up");
    this->writing_lock_ = false;
    this->write_attempt_ = 0;
  }
}

// ---------- Loop/Update ----------
void ACHIClimate::loop() {
  uint8_t c;
  while (this->read_byte(&c)) rx_.push_back(c);

  // Try to extract frames
  std::vector<uint8_t> frame;
  while (this->extract_next_frame_(frame)) {
    this->handle_frame_(frame);
  }
}

void ACHIClimate::update() {
  const uint32_t now = millis();

  // If a write is pending but we passed ack deadline, retry next variant
  if (this->writing_lock_ && now > this->ack_deadline_ms_) {
    ESP_LOGW(TAG, "ACK timeout: switching to next write variant");
    this->handle_nak_fd_();
  }

  // If we have dirty changes and not currently waiting, send now (respect gap)
  if (!this->writing_lock_ && this->dirty_) {
    if (now - this->last_tx_ms_ >= kMinGapMs) {
      this->send_now_();
    }
  }

  // Force a poll a bit after we sent a command, to catch implicit-ACK STATUS
  if (this->writing_lock_ && now >= this->force_poll_at_ms_) {
    this->send_query_status_();
    this->force_poll_at_ms_ = now + 1000; // once per second at most until ACK
  }

  // Regular status polling when idle
  if (!this->writing_lock_) {
    if (now - this->last_status_ms_ > 900) {
      this->send_query_status_();
    } else {
      ESP_LOGV(TAG, "poll: skipped (fresh status %ums ago)", (unsigned)(now - this->last_status_ms_));
    }
  }
}

} // namespace ac_hi
} // namespace esphome
