#include "ac_hi.h"

namespace esphome {
namespace ac_hi {

static const char *const TAG = "ac_hi";

// ---------- Encoding helpers ----------
uint8_t ACHIClimate::encode_fan_byte_(climate::ClimateFanMode f) {
  // Используем значения, которые видим в STATUS (на практике они же ожидаются в WRITE)
  switch (f) {
    case climate::CLIMATE_FAN_AUTO:   return 0x02; // авто — 0x02 (в статусе встречаются 0x01/0x02)
    case climate::CLIMATE_FAN_QUIET:  return 0x0B;
    case climate::CLIMATE_FAN_LOW:    return 0x0D;
    case climate::CLIMATE_FAN_MEDIUM: return 0x0F;
    case climate::CLIMATE_FAN_HIGH:   return 0x11;
    default:                          return 0x02;
  }
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
bool ACHIClimate::reported_matches_pending_(bool rep_power,
                                            climate::ClimateMode rep_mode,
                                            uint8_t rep_set_c,
                                            climate::ClimateFanMode rep_fan,
                                            climate::ClimateSwingMode rep_swing,
                                            bool rep_turbo,
                                            bool rep_eco,
                                            bool rep_quiet,
                                            bool /*rep_led*/) const {
  if (!this->have_pending_) return false;
  if (rep_power != this->pending_power_) return false;
  if (rep_power) { // only check deep fields if powered ON
    if (rep_mode != this->pending_mode_) return false;
    if (rep_set_c != this->pending_target_c_) return false;
    if (rep_fan != this->pending_fan_) return false;
    if (rep_swing != this->pending_swing_) return false;
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
    bool new_turbo = (p == climate::CLIMATE_PRESET_BOOST);
    bool new_eco   = (p == climate::CLIMATE_PRESET_ECO);
    uint8_t new_sleep = (p == climate::CLIMATE_PRESET_SLEEP) ? 1 : 0;
    if (new_turbo != this->turbo_ || new_eco != this->eco_ || new_sleep != this->sleep_stage_) changed = true;
    this->turbo_ = new_turbo;
    this->eco_   = new_eco;
    this->sleep_stage_ = new_sleep;
  }

  if (changed) this->dirty_ = true;
}

// ---------- Legacy CRC16-SUM (HI,LO; sum over [2 .. size-4)) ----------
void ACHIClimate::crc16_sum_legacy_patch(std::vector<uint8_t> &buf) {
  if (buf.size() < 10) return;
  uint16_t sum = 0;
  // Legacy: start from index 2, stop before last 4 bytes (CRC HI, CRC LO, F4, FB)
  const size_t stop = buf.size() - 4;
  for (size_t i = 2; i < stop; i++) {
    sum = static_cast<uint16_t>(sum + buf[i]);
  }
  uint8_t hi = static_cast<uint8_t>((sum >> 8) & 0xFF);
  uint8_t lo = static_cast<uint8_t>(sum & 0xFF);
  buf[buf.size() - 4] = hi;  // HI first (legacy)
  buf[buf.size() - 3] = lo;  // LO second
}

// ---------- Build WRITE (strict legacy layout) ----------
void ACHIClimate::build_legacy_write_(std::vector<uint8_t> &frame) {
  // 41 bytes total
  frame.assign(41, 0x00);
  // Header
  frame[0] = HI_HDR0; frame[1] = HI_HDR1;
  frame[2] = 0x00;
  frame[3] = 0x40;
  frame[4] = 0x29;      // length byte (legacy uses 0x29)
  // Address / fixed part seen in legacy & your logs
  frame[5] = 0x00; frame[6] = 0x00; frame[7] = 0x01; frame[8] = 0x01;
  frame[9] = 0xFE; frame[10]= 0x01; frame[11]= 0x00; frame[12]= 0x00;
  // Command
  frame[13] = CMD_WRITE;

  // As in legacy dumps: [14]=0x00, [15]=0x00
  frame[14] = 0x00;
  frame[15] = 0x00;

  // Payload fields (совпадают с тем, что ты присылал в самом первом TX)
  frame[IDX_FAN]   = encode_fan_byte_(this->fan_);                         // [16] (AUTO -> 0x02)
  frame[IDX_SLEEP] = encode_sleep_byte_(this->sleep_stage_);               // [17] (обычно 0x01)

  const uint8_t mode_hi = encode_mode_hi_write_legacy_(this->mode_);
  const uint8_t power_lo = (this->power_on_ ? 0x0C : 0x04);
  frame[IDX_MODE_POWER] = static_cast<uint8_t>(mode_hi | power_lo);        // [18]

  frame[IDX_TARGET_TEMP] = encode_target_temp_write_legacy_(this->target_c_); // [19]

  // [20..29] zeros on write (legacy ignores)
  for (int i = 20; i <= 29; i++) frame[i] = 0x00;

  // Reserved area EXACTLY как в первом твоём TX:
  frame[30] = 0x00;
  frame[31] = 0x00;
  frame[32] = 0x50; // НЕ пишем сюда swing — в legacy WRITE тут "магическая" 0x50
  // [33] — eco/turbo флаги
  if (this->turbo_)      frame[IDX_FLAGS] = 0b00000010;
  else if (this->eco_)   frame[IDX_FLAGS] = 0b00000100;
  else                   frame[IDX_FLAGS] = 0x00;
  // [34] = 0x00
  frame[34] = 0x00;
  // [35] quiet — в legacy WRITE лучше оставить 0x00, quiet достигается выбором FAN_QUIET
  frame[IDX_FLAGS2] = 0x00;
  // [36] LED — 0x40 выключено, 0xC0 включено
  frame[IDX_LED] = this->led_ ? 0xC0 : 0x40;

  // CRC placeholders + tail
  frame[39] = HI_TAIL0;
  frame[40] = HI_TAIL1;

  ESP_LOGD(TAG, "build(write-legacy): b18=0x%02X (mode_hi=0x%02X power_lo=0x%02X) b19=0x%02X (2*C+1)",
           frame[IDX_MODE_POWER], mode_hi, power_lo, frame[IDX_TARGET_TEMP]);
}

// ---------- Build STATUS query (legacy short) ----------
void ACHIClimate::build_legacy_query_status_(std::vector<uint8_t> &frame) {
  // Короткая форма, которая отвечала в старых логах (len=0x11),
  // критичный байт [17] = 0x01 (seq/маркер)
  frame.assign(22, 0x00); // 22 bytes total
  frame[0] = HI_HDR0; frame[1] = HI_HDR1;
  frame[2] = 0x00;
  frame[3] = 0x40;
  frame[4] = 0x11; // длина короткого кадра
  frame[5] = 0x00; frame[6] = 0x00; frame[7] = 0x01; frame[8] = 0x01;
  frame[9] = 0xFE; frame[10]= 0x01; frame[11]= 0x00; frame[12]= 0x00;
  frame[13] = CMD_STATUS;
  // [14..16] нули
  frame[14] = 0x00;
  frame[15] = 0x00;
  frame[16] = 0x00;
  // [17] ДОЛЖЕН быть 0x01 — иначе некоторые платы молчат
  frame[17] = 0x01;

  // CRC placeholders + tail
  frame[18] = 0x00; // CRC HI placeholder
  frame[19] = 0x00; // CRC LO placeholder
  frame[20] = HI_TAIL0;
  frame[21] = HI_TAIL1;

  crc16_sum_legacy_patch(frame);
}

// ---------- Transport helpers ----------
void ACHIClimate::send_write_frame_(const std::vector<uint8_t> &frame) {
  for (uint8_t b : frame) this->write_byte(b);
}

void ACHIClimate::send_query_status_() {
  std::vector<uint8_t> frame;
  this->build_legacy_query_status_(frame);
  for (uint8_t b : frame) this->write_byte(b);

  // Отладочный дамп запроса статуса
  char hexbuf[256]; size_t p = 0;
  for (size_t i = 0; i < frame.size() && p + 4 < sizeof(hexbuf); i++)
    p += snprintf(hexbuf + p, sizeof(hexbuf) - p, "%02X ", frame[i]);
  ESP_LOGV(TAG, "poll: status query sent (len=%u): %s", (unsigned)frame.size(), hexbuf);
}

// Build snapshot and send
void ACHIClimate::send_now_() {
  std::vector<uint8_t> frame;
  this->build_legacy_write_(frame);

  // Pending snapshot — для implicit ACK через STATUS
  this->have_pending_   = true;
  this->pending_power_  = this->power_on_;
  this->pending_mode_   = this->mode_;
  this->pending_target_c_ = this->target_c_;
  this->pending_fan_    = this->fan_;
  this->pending_swing_  = this->swing_;
  this->pending_turbo_  = this->turbo_;
  this->pending_eco_    = this->eco_;
  this->pending_quiet_  = (this->fan_ == climate::CLIMATE_FAN_QUIET);
  this->pending_led_    = this->led_;

  // CRC (legacy)
  crc16_sum_legacy_patch(frame);

  ESP_LOGD(TAG, "build: len_byte[4]=0x%02X, crc_hi[%u]=0x%02X crc_lo[%u]=0x%02X",
           frame[4],
           (unsigned)(frame.size()-4), frame[frame.size()-4],
           (unsigned)(frame.size()-3), frame[frame.size()-3]);

  // Dump TX
  char hexbuf[512]; size_t p = 0;
  for (size_t i = 0; i < frame.size() && p + 4 < sizeof(hexbuf); i++)
    p += snprintf(hexbuf + p, sizeof(hexbuf) - p, "%02X ", frame[i]);
  ESP_LOGD(TAG, "TX write (%u bytes): %s", (unsigned)frame.size(), hexbuf);

  // Send
  this->send_write_frame_(frame);

  const uint32_t now = millis();
  this->last_tx_ms_ = now;
  this->ack_deadline_ms_ = now + 1000;          // after 1s poll again
  this->force_poll_at_ms_ = now + 150;          // quick poll to catch STATUS update
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
    ESP_LOGW(TAG, "RX frame: NAK (0xFD)");
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

  bool swing_v = (b[IDX_SWING]  & 0b10000000) != 0;         // NB: в WRITE тут 0x50, а в STATUS поле валидное
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

  // Treat STATUS as implicit ACK only when it reflects our pending state
  if (this->writing_lock_) {
    if (this->reported_matches_pending_(power, m, this->target_c_, fan, swing_mode, turbo, eco, quiet, led)) {
      ESP_LOGV(TAG, "STATUS matches pending command → clearing lock (implicit ACK)");
      this->writing_lock_ = false;
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
  this->have_pending_ = false;
}

void ACHIClimate::handle_nak_fd_() {
  // On true NAK allow resend later
  this->dirty_ = true;
  this->writing_lock_ = false;
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

  // If a write is pending and we passed ack deadline, poll again and allow a resend
  if (this->writing_lock_ && now > this->ack_deadline_ms_) {
    ESP_LOGW(TAG, "ACK timeout: polling status / allowing resend");
    this->send_query_status_();
    this->writing_lock_ = false;
    this->dirty_ = true;  // one more try
  }

  // If we have changes and not currently waiting, send now (respect gap)
  if (!this->writing_lock_ && this->dirty_) {
    if (now - this->last_tx_ms_ >= 120) {
      this->send_now_();
    }
  }

  // Force a quick poll after we sent a command, to catch implicit-ACK STATUS
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
