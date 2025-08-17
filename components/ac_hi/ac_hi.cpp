#include "ac_hi.h"

namespace esphome {
namespace ac_hi {

static const char *const TAG = "ac_hi";

// ---------- Climate boilerplate ----------
climate::ClimateTraits ACHIClimate::traits() {
  climate::ClimateTraits t;
  t.set_supported_modes({
      climate::CLIMATE_MODE_OFF,
      climate::CLIMATE_MODE_COOL,
      climate::CLIMATE_MODE_HEAT,
      climate::CLIMATE_MODE_DRY,
      climate::CLIMATE_MODE_FAN_ONLY,
  });
  t.set_supported_fan_modes({
      climate::CLIMATE_FAN_AUTO,
      climate::CLIMATE_FAN_LOW,
      climate::CLIMATE_FAN_MEDIUM,
      climate::CLIMATE_FAN_HIGH,
  });
  t.set_supported_swing_modes({
      climate::CLIMATE_SWING_OFF,
      climate::CLIMATE_SWING_BOTH,
  });
  t.set_visual_min_temperature(16);
  t.set_visual_max_temperature(30);
  t.set_visual_temperature_step(1.0f);
  return t;
}

void ACHIClimate::control(const climate::ClimateCall &call) {
  if (call.get_mode().has_value()) {
    this->mode_ = *call.get_mode();
    this->power_on_ = (this->mode_ != climate::CLIMATE_MODE_OFF);
  }
  if (call.get_target_temperature().has_value())
    this->target_c_ = static_cast<uint8_t>(std::round(*call.get_target_temperature()));

  if (call.get_fan_mode().has_value())
    this->fan_ = *call.get_fan_mode();

  if (call.get_swing_mode().has_value())
    this->swing_ = *call.get_swing_mode();

  this->dirty_ = true;
  ESP_LOGD(TAG, "control(): mode=%d (power_on=%s)", (int) this->mode_, this->power_on_ ? "YES" : "NO");
}

void ACHIClimate::update() {
  // Poll раз в ~0.9s, чтобы не спамить шину
  const uint32_t now = millis();
  if (this->writing_lock_ && now < this->force_poll_at_ms_) {
    // ждём ближайшее окно опроса
  }
  this->send_query_status_();
}

void ACHIClimate::loop() {
  // RX accumulation (неблокирующий)
  uint8_t byte;
  while (this->available()) {
    if (this->read_byte(&byte)) {
      this->rx_.push_back(byte);
    } else {
      break;
    }
  }

  // Извлечь и обработать все полные фреймы
  std::vector<uint8_t> frame;
  while (this->extract_next_frame_(frame)) {
    this->handle_frame_(frame);
    frame.clear();
  }

  // Отправка ожидающих изменений
  const uint32_t now = millis();
  if (this->dirty_ && !this->writing_lock_) {
    this->send_now_();
  }

  // Таймаут ожидания ACK/STATUS
  if (this->writing_lock_ && now > this->ack_deadline_ms_) {
    ESP_LOGW(TAG, "ACK timeout: polling status / allowing resend");
    this->writing_lock_ = false;  // разрешить повтор
  }
}

// ---------- Builders & IO ----------

void ACHIClimate::build_legacy_write_(std::vector<uint8_t> &b) {
  // 50-байтный шаблон (CRC16 SUM в [46]=HI, [47]=LO, хвост [48],[49])
  b.assign(50, 0x00);
  b[0]  = HI_HDR0; b[1] = HI_HDR1;
  b[2]  = 0x00;    b[3] = 0x40;
  b[4]  = 0x29;    // length, как в legacy
  b[5]  = 0x00;    b[6]  = 0x00; b[7]  = 0x01; b[8]  = 0x01;
  b[9]  = 0xFE;    b[10] = 0x01; b[11] = 0x00; b[12] = 0x00;
  b[13] = CMD_WRITE; // 0x65
  b[14] = 0x00; b[15] = 0x00;

  // Управляющие поля
  b[IDX_FAN]         = encode_fan_byte_(this->fan_);
  b[IDX_SLEEP]       = encode_sleep_byte_(this->sleep_stage_);
  b[IDX_MODE_POWER]  = encode_mode_hi_write_legacy_(this->mode_) | (this->power_on_ ? 0x0C : 0x00);
  b[IDX_TARGET_TEMP] = encode_target_temp_write_legacy_(this->target_c_);

  // Swing / флаги
  b[IDX_SWING]  = encode_swing_ud_(this->swing_ != climate::CLIMATE_SWING_OFF);
  b[IDX_FLAGS]  = (this->turbo_ ? 0x0C : 0x00) | (this->eco_ ? 0x10 : 0x00);
  b[IDX_FLAGS2] = (this->quiet_ ? 0x10 : 0x00);
  b[IDX_LED]    = (this->led_ ? 0x40 : 0x00);

  // CRC + tail
  crc16_sum_legacy_patch(b);
  b[48] = HI_TAIL0;
  b[49] = HI_TAIL1;

  ESP_LOGD(TAG, "build: len_byte[4]=0x%02X, crc_hi[46]=0x%02X crc_lo[47]=0x%02X",
           b[4], b[46], b[47]);
  ESP_LOGD(TAG, "build(write-legacy): b18=0x%02X (mode_hi=0x%02X power_lo=0x%02X) b19=0x%02X (2*C+1)",
           b[IDX_MODE_POWER], (b[IDX_MODE_POWER] & 0xF0), (b[IDX_MODE_POWER] & 0x0F), b[IDX_TARGET_TEMP]);
}

void ACHIClimate::build_legacy_query_status_(std::vector<uint8_t> &b) {
  // Короткий 21-байтный STATUS-запрос с SUM8
  b.assign(21, 0x00);
  b[0]=HI_HDR0; b[1]=HI_HDR1;
  b[2]=0x00; b[3]=0x40;
  b[4]=0x0C;             // длина короткого запроса
  b[5]=0x00; b[6]=0x00; b[7]=0x01; b[8]=0x01;
  b[9]=0xFE; b[10]=0x01; b[11]=0x00; b[12]=0x00;
  b[13]=CMD_STATUS;      // 0x66
  b[14]=0x00; b[15]=0x00; b[16]=0x00;
  b[17]=0x01;
  // [18] — SUM8, [19],[20] — хвост
  crc8_sum_legacy_patch(b);
  b[19] = HI_TAIL0;
  b[20] = HI_TAIL1;

  ESP_LOGV(TAG, "poll: status query sent (len=%d): %02X %02X 00 40 %02X 00 00 01 01 FE 01 00 00 66 00 00 00 01 %02X F4 FB",
           (int) b.size(), b[0], b[1], b[4], b[18]);
}

void ACHIClimate::crc16_sum_legacy_patch(std::vector<uint8_t> &buf) {
  uint16_t sum = 0;
  const int stop = (int) buf.size() - 4; // до CRC+tail
  for (int i = 2; i < stop; i++) sum += buf[i];
  buf[46] = (sum >> 8) & 0xFF;  // HI
  buf[47] = sum & 0xFF;         // LO
}

void ACHIClimate::crc8_sum_legacy_patch(std::vector<uint8_t> &buf) {
  uint8_t sum = 0;
  const int stop = (int) buf.size() - 3; // до CRC8+tail
  for (int i = 2; i < stop; i++) sum = (uint8_t)(sum + buf[i]);
  buf[18] = sum;
}

uint8_t ACHIClimate::encode_fan_byte_(climate::ClimateFanMode f) {
  switch (f) {
    case climate::CLIMATE_FAN_AUTO:   return 0x02;
    case climate::CLIMATE_FAN_LOW:    return 0x0D;
    case climate::CLIMATE_FAN_MEDIUM: return 0x0F;
    case climate::CLIMATE_FAN_HIGH:   return 0x11;
    default:                          return 0x02;
  }
}
uint8_t ACHIClimate::encode_sleep_byte_(uint8_t stage) {
  switch (stage) {
    case 1: return 0x01;
    case 2: return 0x11;
    case 3: return 0x21;
    default: return 0x00;
  }
}
uint8_t ACHIClimate::encode_swing_ud_(bool on) { return on ? 0x50 : 0x00; }
uint8_t ACHIClimate::encode_swing_lr_(bool on) { return on ? 0x50 : 0x00; }

void ACHIClimate::send_now_() {
  std::vector<uint8_t> frame;
  this->build_legacy_write_(frame);

  // snapshot желаемого
  this->pending_power_ = this->power_on_;
  this->pending_mode_  = this->mode_;
  this->pending_target_c_ = this->target_c_;
  this->pending_fan_   = this->fan_;
  this->pending_swing_ = this->swing_;
  this->pending_turbo_ = this->turbo_;
  this->pending_eco_   = this->eco_;
  this->pending_quiet_ = this->quiet_;
  this->pending_led_   = this->led_;
  this->have_pending_  = true;

  this->send_write_frame_(frame);
}

void ACHIClimate::send_write_frame_(const std::vector<uint8_t> &b) {
  if (b.size() < 10) return;

  this->write_array(b.data(), b.size());
  this->flush();                 // опустошить FIFO
  delayMicroseconds(2000);       // отпустить RS485 DE/RE

  this->last_tx_ms_ = millis();
  this->ack_deadline_ms_ = this->last_tx_ms_ + 1200; // ждём ~1.2s
  this->writing_lock_ = true;

  // dump
  char dump[512]; size_t pos = 0;
  for (size_t i = 0; i < b.size() && pos + 3 < sizeof(dump); i++)
    pos += snprintf(dump + pos, sizeof(dump) - pos, "%02X%s", b[i], (i + 1 < b.size() ? " " : ""));
  ESP_LOGD(TAG, "TX write (%u bytes): %s", (unsigned) b.size(), dump);
}

void ACHIClimate::send_query_status_() {
  const uint32_t now = millis();
  if (now - this->last_status_ms_ < 900) return;
  this->last_status_ms_ = now;

  std::vector<uint8_t> q;
  this->build_legacy_query_status_(q);
  this->write_array(q.data(), q.size());
  this->flush();
  delayMicroseconds(1200);

  ESP_LOGV(TAG, "poll: status query sent");
}

// ---------- RX parsing ----------

bool ACHIClimate::extract_next_frame_(std::vector<uint8_t> &out) {
  // найти заголовок
  while (this->rx_start_ + 2 <= this->rx_.size()) {
    if (this->rx_[this->rx_start_] == HI_HDR0 && this->rx_[this->rx_start_ + 1] == HI_HDR1)
      break;
    this->rx_start_++;
  }
  if (this->rx_start_ + 2 > this->rx_.size()) {
    if (this->rx_start_ > 0) {
      this->rx_.erase(this->rx_.begin(), this->rx_.begin() + this->rx_start_);
      this->rx_start_ = 0;
    }
    return false;
  }

  // искать хвост
  size_t i = this->rx_start_ + 2;
  for (; i + 1 < this->rx_.size(); i++) {
    if (this->rx_[i] == HI_TAIL0 && this->rx_[i + 1] == HI_TAIL1) {
      size_t end = i + 2;
      out.assign(this->rx_.begin() + this->rx_start_, this->rx_.begin() + end);
      this->rx_.erase(this->rx_.begin(), this->rx_.begin() + end);
      this->rx_start_ = 0;
      return true;
    }
  }

  if (this->rx_start_ > 64) {
    this->rx_.erase(this->rx_.begin(), this->rx_.begin() + this->rx_start_);
    this->rx_start_ = 0;
  }
  return false;
}

void ACHIClimate::handle_frame_(const std::vector<uint8_t> &b) {
  if (b.size() < 16) return;

  const uint8_t cmd = b[13];

  // 1) Нормальный STATUS (0x66)
  if (cmd == CMD_STATUS) {
    ESP_LOGV(TAG, "RX frame: STATUS (0x66), len=%u", (unsigned) b.size());
    this->parse_status_legacy_(b);
    this->force_poll_at_ms_ = millis() + 1200;
    return;
  }

  // 2) Короткий ACK (0x65) — обычно ~18 байт
  if (cmd == CMD_WRITE && b.size() <= 20) {
    ESP_LOGV(TAG, "RX frame: ACK (0x65)");
    this->handle_ack_short_();
    return;
  }

  // 3) Длинный «STATUS-как-0x65» — встречается на части плат (ваш текущий случай)
  if (cmd == CMD_WRITE && b.size() > 40) {
    ESP_LOGV(TAG, "RX frame: cmd=0x65 len=%u → treating as STATUS", (unsigned) b.size());
    this->parse_status_legacy_(b);
    this->force_poll_at_ms_ = millis() + 1200;
    return;
  }

  // 4) NAK
  if (cmd == CMD_NAK) {
    ESP_LOGW(TAG, "RX frame: NAK (0xFD)");
    this->handle_nak_fd_();
    return;
  }

  ESP_LOGV(TAG, "RX frame: cmd=0x%02X len=%u (ignored)", cmd, (unsigned) b.size());
}

void ACHIClimate::handle_ack_short_() {
  if (this->writing_lock_) {
    this->writing_lock_ = false;
    this->dirty_ = false;
  }
}

void ACHIClimate::handle_nak_fd_() {
  this->writing_lock_ = false;
}

// Сопоставление полученного статуса с ожидаемым (для implicit-ACK)
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
  return rep_power == this->pending_power_
      && rep_mode  == this->pending_mode_
      && rep_set_c == this->pending_target_c_
      && rep_fan   == this->pending_fan_
      && rep_swing == this->pending_swing_
      && rep_turbo == this->pending_turbo_
      && rep_eco   == this->pending_eco_
      && rep_quiet == this->pending_quiet_
      && rep_led   == this->pending_led_;
}

void ACHIClimate::parse_status_legacy_(const std::vector<uint8_t> &b) {
  if ((int)b.size() <= IDX_LED) return;  // на всякий

  const uint8_t mode_lo = b[IDX_MODE_POWER] & 0x0F;
  const uint8_t mode_hi = b[IDX_MODE_POWER] & 0xF0;

  bool power = (mode_lo == 0x0C);
  climate::ClimateMode mode = climate::CLIMATE_MODE_COOL;
  switch (mode_hi) {
    case 0x00: mode = climate::CLIMATE_MODE_FAN_ONLY; break;
    case 0x10: mode = climate::CLIMATE_MODE_HEAT;     break;
    case 0x20: mode = climate::CLIMATE_MODE_COOL;     break;
    case 0x30: mode = climate::CLIMATE_MODE_DRY;      break;
    default:   mode = climate::CLIMATE_MODE_COOL;     break;
  }

  uint8_t set_c = (b[IDX_TARGET_TEMP] >> 1) & 0x3F;
  set_c = clamp16_30_(set_c);

  climate::ClimateFanMode fan = climate::CLIMATE_FAN_AUTO;
  switch (b[IDX_FAN]) {
    case 0x02: fan = climate::CLIMATE_FAN_AUTO;   break;
    case 0x0B: fan = climate::CLIMATE_FAN_AUTO;   break; // alias встречается
    case 0x0D: fan = climate::CLIMATE_FAN_LOW;    break;
    case 0x0F: fan = climate::CLIMATE_FAN_MEDIUM; break;
    case 0x11: fan = climate::CLIMATE_FAN_HIGH;   break;
    default:   fan = climate::CLIMATE_FAN_AUTO;   break;
  }

  climate::ClimateSwingMode swing = climate::CLIMATE_SWING_OFF;
  if (b[IDX_SWING] == 0x50 && (b[IDX_FLAGS2] & 0x20))
    swing = climate::CLIMATE_SWING_BOTH;
  else if (b[IDX_SWING] == 0x50 || (b[IDX_FLAGS2] & 0x20))
    swing = climate::CLIMATE_SWING_BOTH;

  bool turbo = (b[IDX_FLAGS] & 0x0C) == 0x0C;
  bool eco   = (b[IDX_FLAGS] & 0x10) == 0x10;
  bool quiet = (b[IDX_FLAGS2] & 0x10) == 0x10;
  bool led   = (b[IDX_LED] & 0x40) == 0x40;

  const int8_t air   = (int8_t) b[IDX_AIR_TEMP];
  const int8_t pipe  = (int8_t) b[IDX_PIPE_TEMP];

#ifdef USE_SENSOR
  if (this->pipe_sensor_ != nullptr)
    this->pipe_sensor_->publish_state(pipe);
#endif

  // implicit-ACK по совпадению статуса с тем, что отправляли
  if (this->writing_lock_ && this->reported_matches_pending_(power, mode, set_c, fan, swing, turbo, eco, quiet, led)) {
    ESP_LOGV(TAG, "STATUS received while waiting ACK → clearing lock");
    this->writing_lock_ = false;
    this->dirty_ = false;
  }

  // публикация фактического состояния
  this->mode_ = power ? mode : climate::CLIMATE_MODE_OFF;
  this->power_on_ = power;
  this->target_c_ = set_c;
  this->fan_ = fan;
  this->swing_ = swing;
  this->turbo_ = turbo;
  this->eco_ = eco;
  this->quiet_ = quiet;
  this->led_ = led;

  this->publish_state();
}

} // namespace ac_hi
} // namespace esphome
