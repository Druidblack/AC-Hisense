#include "ac_hi.h"
#include <cmath>
#include <algorithm>
#include <cstdio>

namespace esphome {
namespace ac_hi {

static const char *const TAG = "ac_hi.climate";

void ACHIClimate::setup() {
  ESP_LOGI(TAG, "Setup AC-Hi climate");
  this->mode = climate::CLIMATE_MODE_OFF;
  this->target_temperature = 24;
  this->fan_mode = climate::CLIMATE_FAN_AUTO;
  this->swing_mode = climate::CLIMATE_SWING_OFF;
  this->publish_state();
}

void ACHIClimate::update() {
  // Периодический опрос: короткий запрос статуса
  if (!this->writing_lock_) {
    this->send_query_status_();
  }
}

void ACHIClimate::loop() {
  // Накапливаем входящие байты
  uint8_t c;
  while (this->read_byte(&c)) {
    rx_.push_back(c);
    // чтобы буфер не рос бесконечно
    if (rx_.size() > 2048) {
      rx_.erase(rx_.begin(), rx_.begin() + 1024);
    }
  }

  // Пробуем извлечь все полные кадры из буфера
  this->try_parse_frames_from_buffer_();
}

climate::ClimateTraits ACHIClimate::traits() {
  climate::ClimateTraits t{};
  t.set_supports_action(false);
  t.set_supported_modes({climate::CLIMATE_MODE_OFF, climate::CLIMATE_MODE_COOL, climate::CLIMATE_MODE_HEAT,
                         climate::CLIMATE_MODE_DRY, climate::CLIMATE_MODE_FAN_ONLY, climate::CLIMATE_MODE_AUTO});
  t.set_supported_fan_modes({climate::CLIMATE_FAN_AUTO, climate::CLIMATE_FAN_LOW, climate::CLIMATE_FAN_MEDIUM,
                             climate::CLIMATE_FAN_HIGH, climate::CLIMATE_FAN_QUIET});
  t.set_supported_swing_modes({climate::CLIMATE_SWING_OFF, climate::CLIMATE_SWING_VERTICAL,
                               climate::CLIMATE_SWING_HORIZONTAL, climate::CLIMATE_SWING_BOTH});
  if (enable_presets_) {
    t.set_supported_presets({climate::CLIMATE_PRESET_NONE, climate::CLIMATE_PRESET_ECO, climate::CLIMATE_PRESET_BOOST,
                             climate::CLIMATE_PRESET_SLEEP});
  }
  t.set_visual_min_temperature(18);
  t.set_visual_max_temperature(28);
  t.set_visual_temperature_step(1.0f);
  t.set_supports_current_temperature(true);
  return t;
}

void ACHIClimate::control(const climate::ClimateCall &call) {
  bool need_write = false;

  if (call.get_mode().has_value()) {
    auto m = *call.get_mode();
    if (m == climate::CLIMATE_MODE_OFF) {
      this->power_on_ = false;
    } else {
      this->power_on_ = true;
      this->mode_ = m;
    }
    need_write = true;
  }

  if (call.get_target_temperature().has_value()) {
    auto t = *call.get_target_temperature();
    if (!std::isnan(t)) {
      uint8_t c = static_cast<uint8_t>(std::round(t));
      c = std::max<uint8_t>(18, std::min<uint8_t>(28, c));
      this->target_c_ = c;
      need_write = true;
    }
  }

  if (call.get_fan_mode().has_value()) {
    this->fan_ = *call.get_fan_mode();
    need_write = true;
  }

  if (call.get_swing_mode().has_value()) {
    this->swing_ = *call.get_swing_mode();
    need_write = true;
  }

  if (call.get_preset().has_value()) {
    auto p = *call.get_preset();
    this->eco_ = (p == climate::CLIMATE_PRESET_ECO);
    this->turbo_ = (p == climate::CLIMATE_PRESET_BOOST);
    this->sleep_stage_ = (p == climate::CLIMATE_PRESET_SLEEP) ? 1 : 0;
    need_write = true;
  }

  if (need_write) {
    // Сборка TX-кадра
    uint8_t power_bin = this->power_on_ ? 0b00001100 : 0b00000100;  // из YAML
    uint8_t mode_hi = encode_mode_hi_nibble_(this->mode_);
    tx_bytes_[18] = power_bin + mode_hi;

    tx_bytes_[19] = encode_temp_(this->target_c_);          // setpoint
    tx_bytes_[16] = encode_fan_byte_(this->fan_);           // fan
    tx_bytes_[17] = encode_sleep_byte_(this->sleep_stage_); // sleep

    bool v_swing = (this->swing_ == climate::CLIMATE_SWING_VERTICAL) || (this->swing_ == climate::CLIMATE_SWING_BOTH);
    bool h_swing = (this->swing_ == climate::CLIMATE_SWING_HORIZONTAL) || (this->swing_ == climate::CLIMATE_SWING_BOTH);
    uint8_t updown_bin = encode_swing_ud_(v_swing);
    uint8_t leftright_bin = encode_swing_lr_(h_swing);
    tx_bytes_[32] = updown_bin + leftright_bin;

    uint8_t turbo_bin = this->turbo_ ? 0b00001100 : 0b00000100;
    uint8_t eco_bin = this->eco_ ? 0b00110000 : 0b00000000;
    tx_bytes_[33] = (this->turbo_ ? turbo_bin : (eco_bin ? eco_bin : 0));

    this->quiet_ = (this->fan_ == climate::CLIMATE_FAN_QUIET);
    tx_bytes_[35] = this->quiet_ ? 0b00110000 : 0b00000000;

    tx_bytes_[36] = this->led_ ? 0b11000000 : 0b01000000;

    if (this->turbo_) {
      tx_bytes_[19] = 0;       // override temperature
      tx_bytes_[33] = turbo_bin;
      tx_bytes_[35] = 0;       // override quiet
    }

    this->writing_lock_ = true;
    this->pending_write_ = true;
    this->send_write_changes_();
  }

  // Оптимистично публикуем ожидаемое состояние
  this->mode = this->power_on_ ? this->mode_ : climate::CLIMATE_MODE_OFF;
  this->target_temperature = this->target_c_;
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

// ---- Кодирование полей ----

uint8_t ACHIClimate::encode_mode_hi_nibble_(climate::ClimateMode m) {
  // {fan_only,heat,cool,dry,auto} -> ((code<<1)|1) << 4
  uint8_t code = 4;
  switch (m) {
    case climate::CLIMATE_MODE_FAN_ONLY: code = 0; break;
    case climate::CLIMATE_MODE_HEAT:     code = 1; break;
    case climate::CLIMATE_MODE_COOL:     code = 2; break;
    case climate::CLIMATE_MODE_DRY:      code = 3; break;
    case climate::CLIMATE_MODE_AUTO:     code = 4; break;
    default:                             code = 4; break;
  }
  return static_cast<uint8_t>(((code << 1) | 0x01) << 4);
}

uint8_t ACHIClimate::encode_fan_byte_(climate::ClimateFanMode f) {
  // Соответствие: AUTO->1, QUIET->10, LOW->12, MED->14, HIGH->16; при записи +1
  uint8_t code = 1;
  switch (f) {
    case climate::CLIMATE_FAN_AUTO:   code = 1;  break;
    case climate::CLIMATE_FAN_LOW:    code = 12; break;
    case climate::CLIMATE_FAN_MEDIUM: code = 14; break;
    case climate::CLIMATE_FAN_HIGH:   code = 16; break;
    case climate::CLIMATE_FAN_QUIET:  code = 10; break;
    default:                          code = 1;  break;
  }
  return static_cast<uint8_t>(code + 1);
}

uint8_t ACHIClimate::encode_sleep_byte_(uint8_t stage) {
  // listix_to_sleep_codes {0,1,2,4,8} -> (code<<1)|1
  uint8_t code = 0;
  switch (stage) {
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

// ---- Транспорт/CRC/логирование ----

void ACHIClimate::send_query_status_() {
  // Короткий запрос — отправляем как есть (CRC уже в шаблоне), логируем
  this->log_hex_frame_("TX", this->query_, "query(0x66)");
  for (auto b : this->query_) this->write_byte(b);  // совместимо с UARTDevice
  this->flush();
}

void ACHIClimate::calc_and_patch_crc_(std::vector<uint8_t> &buf) {
  // сумма байтов с 2 по (len-4)
  const int n = static_cast<int>(buf.size());
  uint16_t csum = 0;
  for (int i = 2; i < n - 4; i++) csum = static_cast<uint16_t>(csum + buf[i]);
  buf[n - 4] = static_cast<uint8_t>((csum & 0xFF00) >> 8);
  buf[n - 3] = static_cast<uint8_t>(csum & 0x00FF);
}

bool ACHIClimate::validate_crc_(const std::vector<uint8_t> &buf, uint16_t *out_sum) const {
  if (buf.size() < 8) return false;
  const size_t n = buf.size();
  uint16_t csum = 0;
  for (size_t i = 2; i < n - 4; i++) csum = static_cast<uint16_t>(csum + buf[i]);
  uint8_t cr1 = static_cast<uint8_t>((csum & 0xFF00) >> 8);
  uint8_t cr2 = static_cast<uint8_t>(csum & 0x00FF);
  if (out_sum) *out_sum = csum;
  return (buf[n - 4] == cr1) && (buf[n - 3] == cr2);
}

void ACHIClimate::send_write_changes_() {
  auto frame = this->tx_bytes_;
  this->calc_and_patch_crc_(frame);
  this->log_hex_frame_("TX", frame, "write");
  for (auto b : frame) this->write_byte(b);  // по-байтовая отправка
  this->flush();
}

void ACHIClimate::log_hex_frame_(const char *dir, const std::vector<uint8_t> &data, const char *note) const {
  // Печатаем заголовок с длиной и суммой; без Arduino String
  size_t n = data.size();
  uint16_t crc = 0;
  for (size_t i = 2; i < (n >= 4 ? n - 4 : 0); i++) crc = static_cast<uint16_t>(crc + data[i]);

  char decl_buf[8];
  const char *decl_str = "-";
  if (n > 5) {
    std::snprintf(decl_buf, sizeof(decl_buf), "0x%02X", static_cast<unsigned>(data[4]));
    decl_str = decl_buf;
  }

  ESP_LOGD(TAG, "%s frame (%s): len=%u, decl_len=%s, crc_sum=0x%04X",
           dir, (note ? note : "-"),
           static_cast<unsigned>(n),
           decl_str,
           crc);

  // Печатаем покадрово (по 16 байт), чтобы не раздувать одну строку
  char line[3 * 16 + 64];
  for (size_t i = 0; i < n; i += 16) {
    size_t chunk = std::min(static_cast<size_t>(16), n - i);
    int pos = 0;
    pos += std::snprintf(line + pos, sizeof(line) - pos, "%s [%03u..%03u]: ",
                         dir, static_cast<unsigned>(i), static_cast<unsigned>(i + chunk - 1));
    for (size_t j = 0; j < chunk && pos < static_cast<int>(sizeof(line) - 4); j++) {
      pos += std::snprintf(line + pos, sizeof(line) - pos, "%02X ", static_cast<unsigned>(data[i + j]));
    }
    ESP_LOGV(TAG, "%s", line);
  }
}

// ---- RX сканер/парсер ----

void ACHIClimate::try_parse_frames_from_buffer_() {
  // Ищем и извлекаем все полные [F4 F5 ... F4 FB] из rx_
  std::vector<uint8_t> frame;
  size_t safety = 0;
  while (this->extract_next_frame_(frame)) {
    // Логируем принятый кадр
    this->log_hex_frame_("RX", frame, "raw");
    // Валидация CRC
    uint16_t sum = 0;
    if (!this->validate_crc_(frame, &sum)) {
      ESP_LOGW(TAG, "<< Drop: CRC mismatch (sum=0x%04X, got=%02X %02X)", sum,
               frame[frame.size() - 4], frame[frame.size() - 3]);
      continue;
    }
    // Небольшая диагностика по длине
    if (frame.size() > 5) {
      uint8_t decl = frame[4];
      ESP_LOGV(TAG, "<< Declared len=0x%02X, actual bytes=%u", decl, static_cast<unsigned>(frame.size()));
    }
    // Передаём на разбор
    this->handle_frame_(frame);

    // Против зацикливания на испорченном потоке
    if (++safety > 32) break;
  }
}

bool ACHIClimate::extract_next_frame_(std::vector<uint8_t> &frame) {
  frame.clear();
  if (rx_.size() < 6) return false;

  // 1) Ищем заголовок F4 F5
  size_t i = 0;
  bool found_header = false;
  for (; i + 1 < rx_.size(); i++) {
    if (rx_[i] == HI_HDR0 && rx_[i + 1] == HI_HDR1) {
      found_header = true;
      break;
    }
  }
  if (!found_header) {
    // Сохраняем последний байт, если он равен первому байту заголовка, на случай частичного совпадения
    if (!rx_.empty() && rx_.back() == HI_HDR0) {
      uint8_t keep = rx_.back();
      rx_.clear();
      rx_.push_back(keep);
    } else {
      rx_.clear();
    }
    return false;
  }
  // Отбрасываем мусор до заголовка
  if (i > 0) rx_.erase(rx_.begin(), rx_.begin() + i);

  // 2) Ищем хвост F4 FB после заголовка
  size_t j = 2;  // поиск начнём сразу после заголовка
  bool found_tail = false;
  for (; j + 1 < rx_.size(); j++) {
    if (rx_[j] == HI_TAIL0 && rx_[j + 1] == HI_TAIL1) {
      found_tail = true;
      break;
    }
  }
  if (!found_tail) {
    // Кадр ещё не полный
    return false;
  }

  // 3) Извлекаем кадр [0 .. j+1]
  frame.insert(frame.end(), rx_.begin(), rx_.begin() + j + 2);

  // 4) Удаляем его из буфера
  rx_.erase(rx_.begin(), rx_.begin() + j + 2);

  return true;
}

void ACHIClimate::handle_frame_(const std::vector<uint8_t> &b) {
  if (b.size() < 20) return;

  const uint8_t cmd = b[13];
  const uint8_t typ = b[2];  // разные блоки могут присылать 0/1 — логируем и принимаем

  ESP_LOGV(TAG, "<< Frame: cmd=%u (0x%02X), typ=%u, byte[4]=0x%02X", cmd, cmd, typ, (b.size() > 5 ? b[4] : 0));

  if (cmd == 102 /*0x66 status resp*/ ) {
    this->parse_status_102_(b);
  } else if (cmd == 101 /*0x65 ack*/) {
    this->handle_ack_101_();
  } else {
    ESP_LOGD(TAG, "<< Unknown/unsupported cmd=0x%02X", cmd);
  }
}

void ACHIClimate::parse_status_102_(const std::vector<uint8_t> &bytes) {
  // Дополнительная "антиповторная" проверка (по сумме)
  uint16_t crc = 0;
  for (size_t i = 2; i < bytes.size() - 4; i++) crc = static_cast<uint16_t>(crc + bytes[i]);
  if (crc == last_status_crc_) {
    ESP_LOGV(TAG, "<< Status 102: unchanged");
    return;
  }
  last_status_crc_ = crc;

  // Питание (байт 18, бит 3)
  bool power = (bytes[18] & 0b00001000) != 0;
  this->power_on_ = power;

  // Режим (старший полубайт байта 18)
  uint8_t mode_code = (bytes[18] >> 4) & 0x0F;
  climate::ClimateMode new_mode = climate::CLIMATE_MODE_AUTO;
  switch (mode_code) {
    case 0x01: new_mode = climate::CLIMATE_MODE_FAN_ONLY; break;  // (0<<1 |1) = 1
    case 0x03: new_mode = climate::CLIMATE_MODE_HEAT; break;      // (1<<1 |1) = 3
    case 0x05: new_mode = climate::CLIMATE_MODE_COOL; break;      // (2<<1 |1) = 5
    case 0x07: new_mode = climate::CLIMATE_MODE_DRY; break;       // (3<<1 |1) = 7
    case 0x09: new_mode = climate::CLIMATE_MODE_AUTO; break;      // (4<<1 |1) = 9
    default:   new_mode = climate::CLIMATE_MODE_AUTO; break;
  }
  this->mode_ = new_mode;

  // Скорость вентилятора (байт 16)
  uint8_t raw_wind = bytes[16];
  climate::ClimateFanMode new_fan = climate::CLIMATE_FAN_AUTO;
  if (raw_wind == 1 || raw_wind == 2) new_fan = climate::CLIMATE_FAN_AUTO;
  else if (raw_wind == 11) new_fan = climate::CLIMATE_FAN_QUIET;
  else if (raw_wind == 13) new_fan = climate::CLIMATE_FAN_LOW;
  else if (raw_wind == 15) new_fan = climate::CLIMATE_FAN_MEDIUM;
  else if (raw_wind == 17) new_fan = climate::CLIMATE_FAN_HIGH;
  this->fan_ = new_fan;

  // Sleep (байт 17)
  uint8_t raw_sleep = bytes[17];
  uint8_t code = (raw_sleep >> 1);
  if (code == 0) this->sleep_stage_ = 0;
  else if (code == 1) this->sleep_stage_ = 1;
  else if (code == 2) this->sleep_stage_ = 2;
  else if (code == 4) this->sleep_stage_ = 3;
  else if (code == 8) this->sleep_stage_ = 4;
  else this->sleep_stage_ = 0;

  // Целевая температура (байт 19, код 2*n+1)
  uint8_t raw_set = bytes[19];
  if (raw_set & 0x01) {
    uint8_t c = raw_set >> 1;
    if (c >= 18 && c <= 28) this->target_c_ = c;
  }
  this->target_temperature = this->target_c_;

  // Текущая температура воздуха (байт 20)
  uint8_t tair = bytes[20];
  this->current_temperature = tair;

  // Температура трубки (байт 21)
#ifdef USE_SENSOR
  if (this->pipe_sensor_ != nullptr) this->pipe_sensor_->publish_state(bytes[21]);
#endif

  // Turbo/Eco/Quiet/LED
  uint8_t b35 = bytes[35];
  this->turbo_ = (b35 & 0b00000010) != 0;       // turbo_mask = 0b00000010
  this->eco_   = (b35 & 0b00000100) != 0;       // eco_mask   = 0b00000100
  this->quiet_ = (bytes[36] & 0b00000100) != 0; // quiet      в 36-м
  this->led_   = (bytes[37] & 0b10000000) != 0; // LED        в 37-м

  // Swing (байт 35: updown bit7, leftright bit6)
  bool updown = (bytes[35] & 0b10000000) != 0;
  bool leftright = (bytes[35] & 0b01000000) != 0;
  if (updown && leftright) this->swing_ = climate::CLIMATE_SWING_BOTH;
  else if (updown) this->swing_ = climate::CLIMATE_SWING_VERTICAL;
  else if (leftright) this->swing_ = climate::CLIMATE_SWING_HORIZONTAL;
  else this->swing_ = climate::CLIMATE_SWING_OFF;

  // Публикуем
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

void ACHIClimate::handle_ack_101_() {
  ESP_LOGD(TAG, "<< ACK 0x101 received; unlock writes");
  this->writing_lock_ = false;
  this->pending_write_ = false;
}

} // namespace ac_hi
} // namespace esphome
