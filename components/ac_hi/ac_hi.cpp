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

uint8_t ACHIClimate::encode_swing_ud_(bool on) { return on ? 0b11000000 : 0b01000000; }
uint8_t ACHIClimate::encode_swing_lr_(bool on) { return on ? 0b00110000 : 0b00010000; }

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
      climate::CLIMATE_PRESET_SLEEP
    });
  }
  t.set_visual_min_temperature(16);
  t.set_visual_max_temperature(30);
  t.set_visual_temperature_step(1.0f);
  t.set_supports_current_temperature(true);
  return t;
}

// ---------- Control (coalesce + wait STATUS as ACK) ----------
void ACHIClimate::control(const climate::ClimateCall &call) {
  bool changed = false;

  if (call.get_mode().has_value()) {
    auto m = *call.get_mode();
    if (m == climate::CLIMATE_MODE_OFF) {
      if (this->power_on_) { changed = true; }
      this->power_on_ = false;
    } else {
      if (!this->power_on_ || this->mode_ != m) { changed = true; }
      this->power_on_ = true;
      this->mode_ = m;
    }
    ESP_LOGD(TAG, "control(): mode=%d (power_on=%s)", (int)m, YESNO(this->power_on_));
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
    this->eco_ = new_eco;
    this->sleep_stage_ = new_sleep;
    ESP_LOGD(TAG, "control(): preset turbo=%d eco=%d sleep=%u", (int)this->turbo_, (int)this->eco_, (unsigned)this->sleep_stage_);
  }

  if (!changed) {
    ESP_LOGV(TAG, "control(): nothing changed, skipping send");
    return;
  }

  this->dirty_ = true;

  if (this->writing_lock_) {
    ESP_LOGV(TAG, "control(): write in-flight, mark dirty and wait for STATUS");
    return;
  }

  const uint32_t now = millis();
  if (now - this->last_tx_ms_ < kMinGapMs) {
    ESP_LOGV(TAG, "control(): respecting min gap (%ums), will send later", (unsigned)kMinGapMs);
    return;
  }

  this->send_now_();
}

// ---------- Transport helpers ----------
void ACHIClimate::calc_and_patch_crc_(std::vector<uint8_t> &buf) const {
  // В этой прошивке статус-кадры используют 1-байтовую контрольную сумму перед хвостом.
  // Делаем тот же 8-битный сумматор (bytes[3 .. size-4]).
  if (buf.size() < 8) return;
  uint8_t sum = 0;
  for (size_t i = 3; i + 4 <= buf.size(); i++) sum = static_cast<uint8_t>(sum + buf[i]);
  buf[buf.size() - 3] = sum; // один байт CRC перед хвостом
}

void ACHIClimate::send_write_frame_(const std::vector<uint8_t> &frame) {
  for (uint8_t b : frame) this->write_byte(b);
  // flush() убираем, чтобы не блокировать цикл
}

void ACHIClimate::send_query_status_() {
  for (uint8_t b : this->query_) this->write_byte(b);
  ESP_LOGV(TAG, "poll: status query sent");
}

// Build snapshot and send
void ACHIClimate::send_now_() {
  std::vector<uint8_t> frame = this->tx_bytes_;

  // Byte 18: mode|power (mode hi-nibble direct 0..3, power lo-nibble = 0x00 !!!)
  uint8_t mode_hi = encode_mode_hi_direct_(this->mode_);
  uint8_t power_lo = encode_power_lo_neutral_();
  frame[IDX_MODE_POWER] = static_cast<uint8_t>(mode_hi | power_lo);

  // Byte 19: target temp (device reports plain C in your logs)
  frame[IDX_TARGET_TEMP] = encode_target_temp_direct_(this->target_c_);

  // Fan / Sleep
  frame[IDX_FAN]   = encode_fan_byte_(this->fan_);
  frame[IDX_SLEEP] = encode_sleep_byte_(this->sleep_stage_);

  // Swing combine
  bool v = (this->swing_ == climate::CLIMATE_SWING_VERTICAL) || (this->swing_ == climate::CLIMATE_SWING_BOTH);
  bool h = (this->swing_ == climate::CLIMATE_SWING_HORIZONTAL) || (this->swing_ == climate::CLIMATE_SWING_BOTH);
  frame[IDX_SWING] = static_cast<uint8_t>(encode_swing_ud_(v) | encode_swing_lr_(h));

  // Flags: Turbo/Eco (mutually exclusive)
  if (this->turbo_)      frame[IDX_FLAGS] = 0b00000010;
  else if (this->eco_)   frame[IDX_FLAGS] = 0b00000100;
  else                   frame[IDX_FLAGS] = 0;

  // Quiet mirrors fan quiet
  this->quiet_ = (this->fan_ == climate::CLIMATE_FAN_QUIET);
  frame[IDX_FLAGS2] = this->quiet_ ? 0b00110000 : 0x00;

  // LED
  frame[IDX_LED] = this->led_ ? 0b11000000 : 0b01000000;

  // Patch LEN field (byte[4]) = total_len - 9
  uint8_t total = static_cast<uint8_t>(frame.size());
  frame[4] = static_cast<uint8_t>(total - 9);

  // CRC and send
  this->calc_and_patch_crc_(frame);
  this->send_write_frame_(frame);

  // Log full frame (hex)
  char hexbuf[1024];
  size_t n = std::min(frame.size(), sizeof(hexbuf)/3);
  size_t p = 0;
  for (size_t i = 0; i < n; i++) {
    p += snprintf(hexbuf + p, sizeof(hexbuf) - p, "%02X ", frame[i]);
    if (p >= sizeof(hexbuf)) break;
  }
  ESP_LOGD(TAG, "TX write (%u bytes): %s", (unsigned)frame.size(), hexbuf);

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

  // find tail
  size_t j = i + 2;
  while (j + 1 < N && !(rx_[j] == HI_TAIL0 && rx_[j + 1] == HI_TAIL1)) ++j;
  if (j + 1 >= N) { rx_start_ = i; return false; }

  out.assign(rx_.begin() + (std::ptrdiff_t)i, rx_.begin() + (std::ptrdiff_t)(j + 2));
  rx_start_ = j + 2;
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

  // Power: в вашей прошивке lo==0x0; считаем «включено», если hi сообщает режим 0..3
  bool power = (hi <= 0x03);
  this->power_on_ = power;

  // Mode decode (direct 0..3)
  climate::ClimateMode m;
  if      (hi == 0x00) m = climate::CLIMATE_MODE_FAN_ONLY;
  else if (hi == 0x01) m = climate::CLIMATE_MODE_HEAT;
  else if (hi == 0x02) m = climate::CLIMATE_MODE_COOL;
  else if (hi == 0x03) m = climate::CLIMATE_MODE_DRY;
  else                 m = this->mode_; // leave last known

  this->mode_ = m;

  // Target temp: device reports plain C (ваш лог: set=25)
  uint8_t raw_set = b[IDX_TARGET_TEMP];
  uint8_t set_c = (raw_set >= 16 && raw_set <= 30) ? raw_set : (uint8_t)(raw_set >> 1);
  this->target_c_ = clamp16_30_(set_c);
  this->target_temperature = this->target_c_;

  // Current temperatures
  this->current_temperature = b[IDX_AIR_TEMP];
#ifdef USE_SENSOR
  if (this->pipe_sensor_ != nullptr) this->pipe_sensor_->publish_state(b[IDX_PIPE_TEMP]);
#endif

  // Fan
  uint8_t rf = b[IDX_FAN];
  climate::ClimateFanMode fan = climate::CLIMATE_FAN_AUTO;
  if      (rf ==  1 || rf ==  2) fan = climate::CLIMATE_FAN_AUTO;
  else if (rf == 11)             fan = climate::CLIMATE_FAN_QUIET;
  else if (rf == 13)             fan = climate::CLIMATE_FAN_LOW;
  else if (rf == 15)             fan = climate::CLIMATE_FAN_MEDIUM;
  else if (rf == 17)             fan = climate::CLIMATE_FAN_HIGH;
  this->fan_ = fan;

  // Sleep
  uint8_t rs = b[IDX_SLEEP];
  uint8_t sc = (uint8_t)(rs >> 1);
  uint8_t sleep_stage = 0;
  if      (sc == 0) sleep_stage = 0;
  else if (sc == 1) sleep_stage = 1;
  else if (sc == 2) sleep_stage = 2;
  else if (sc == 4) sleep_stage = 3;
  else if (sc == 8) sleep_stage = 4;
  this->sleep_stage_ = sleep_stage;

  // Flags/LED/Swing
  bool turbo = (b[IDX_FLAGS]  & 0b00000010) != 0;
  bool eco   = (b[IDX_FLAGS]  & 0b00000100) != 0;
  bool quiet = (b[IDX_FLAGS2] & 0b00110000) == 0b00110000;
  bool led   = (b[IDX_LED]    & 0b10000000) != 0;
  this->turbo_ = turbo;
  this->eco_   = eco;
  this->quiet_ = quiet;
  this->led_   = led;

  bool swing_v = (b[IDX_FLAGS2] & 0b10000000) != 0;
  bool swing_h = (b[IDX_FLAGS2] & 0b01000000) != 0;
  climate::ClimateSwingMode swing_mode = climate::CLIMATE_SWING_OFF;
  if (swing_v && swing_h) swing_mode = climate::CLIMATE_SWING_BOTH;
  else if (swing_v)       swing_mode = climate::CLIMATE_SWING_VERTICAL;
  else if (swing_h)       swing_mode = climate::CLIMATE_SWING_HORIZONTAL;
  this->swing_ = swing_mode;

  // Publish real state
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

  // STATUS == implicit ACK
  if (this->writing_lock_) {
    ESP_LOGV(TAG, "STATUS received while waiting ACK → clearing lock");
    this->writing_lock_ = false;
    // если накопились изменения — сразу отправить (с паузой)
    const uint32_t now = millis();
    if (this->dirty_ && (now - this->last_tx_ms_ >= kMinGapMs)) {
      this->send_now_();
    }
  }
}

void ACHIClimate::handle_ack_101_() {
  this->writing_lock_ = false;
  ESP_LOGV(TAG, "ACK received; dirty=%d", (int)this->dirty_);
  if (this->dirty_) {
    const uint32_t now = millis();
    if (now - this->last_tx_ms_ >= kMinGapMs) {
      this->send_now_();
    } else {
      ESP_LOGV(TAG, "ACK: deferring resend until min gap");
    }
  }
}

// ---------- Loop/Update ----------
void ACHIClimate::loop() {
  // Read UART into rx buffer
  uint8_t c;
  while (this->read_byte(&c)) {
    rx_.push_back(c);
  }

  // Compact buffer if needed
  if (rx_start_ > 4096) {
    rx_.erase(rx_.begin(), rx_.begin() + (std::ptrdiff_t)rx_start_);
    rx_start_ = 0;
  }

  // Parse frames with small time budget (3 ms)
  uint32_t start = millis();
  std::vector<uint8_t> frame;
  while (millis() - start < 3) {
    if (!this->extract_next_frame_(frame)) break;
    this->handle_frame_(frame);
  }
}

void ACHIClimate::update() {
  const uint32_t now = millis();

  // Форсированный опрос после write, даже если ещё ждём ACK/STATUS
  if (this->writing_lock_ && this->force_poll_at_ms_ != 0 && now >= this->force_poll_at_ms_) {
    this->force_poll_at_ms_ = 0;
    this->send_query_status_();
  }

  // Если долго не было ACK/STATUS — снимем лок
  if (this->writing_lock_ && now > this->ack_deadline_ms_) {
    ESP_LOGW(TAG, "ACK timeout after %ums; clearing lock", (unsigned)kAckTimeoutMs);
    this->writing_lock_ = false;
  }

  // Если есть несообщённые изменения и можно слать — отправляем
  if (!this->writing_lock_ && this->dirty_ && (now - this->last_tx_ms_ >= kMinGapMs)) {
    ESP_LOGV(TAG, "update(): pending dirty state, sending now");
    this->send_now_();
    return;
  }

  // Опрос статуса (не чаще ~1с, и не спамим если недавно был STATUS)
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
