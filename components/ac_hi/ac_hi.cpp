#include "ac_hi.h"
#include <cmath>
#include <algorithm>
#ifdef USE_LOGGER
  #include "esphome/core/log.h"
  static const char *const TAG = "ac_hi";
  #if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_DEBUG
  // Быстрый formatter кадра в HEX ("AA BB CC")
  static inline std::string ac_hi_bytes_to_hex_(const std::vector<uint8_t> &data) {
    std::string out;
    out.reserve(data.size() * 3);
    static const char hexdig[] = "0123456789ABCDEF";
    for (size_t i = 0; i < data.size(); i++) {
      uint8_t v = data[i];
      out.push_back(hexdig[(v >> 4) & 0x0F]);
      out.push_back(hexdig[v & 0x0F]);
      if (i + 1 != data.size()) out.push_back(' ');
    }
    return out;
  }
  #endif
#endif

namespace esphome {
namespace ac_hi {

// ---- Локальные хелперы для (де)кодирования режима ----
// Целевая раскладка nibble (byte[18] >> 4):
// 0x00 = FAN_ONLY, 0x01 = HEAT, 0x02 = COOL, 0x03 = DRY.
// Иного здесь нет — AUTO не поддерживается и в HA не объявляется.
static inline climate::ClimateMode decode_mode_from_nibble(uint8_t nib) {
  switch (nib & 0x0F) {
    case 0x00: return climate::CLIMATE_MODE_FAN_ONLY;
    case 0x01: return climate::CLIMATE_MODE_HEAT;
    case 0x02: return climate::CLIMATE_MODE_COOL;
    case 0x03: return climate::CLIMATE_MODE_DRY;
    default:   return climate::CLIMATE_MODE_COOL;  // fallback
  }
}
static inline uint8_t encode_nibble_from_mode(climate::ClimateMode m) {
  switch (m) {
    case climate::CLIMATE_MODE_FAN_ONLY: return 0x00;
    case climate::CLIMATE_MODE_HEAT:     return 0x01;
    case climate::CLIMATE_MODE_COOL:     return 0x02;
    case climate::CLIMATE_MODE_DRY:      return 0x03;
    default:                             return 0x02; // AUTO нет, используем COOL
  }
}

void ACHIClimate::setup() {
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
  } else if (this->pending_write_ && (esphome::millis() - this->write_started_ms_ > 450)) {
    // Если запись «залипла», снимаем лок
    this->writing_lock_ = false;
    this->pending_write_ = false;
  }
}

void ACHIClimate::loop() {
  // Накапливаем входящие байты неблокирующе
  uint8_t c;
  while (this->read_byte(&c)) {
    rx_.push_back(c);
  }

  // Скользящее окно/компакция буфера
  if (rx_start_ > RX_COMPACT_THRESHOLD) {
    rx_.erase(rx_.begin(), rx_.begin() + static_cast<std::ptrdiff_t>(rx_start_));
    rx_start_ = 0;
  }
  if (rx_.size() - rx_start_ > 4096) {
    size_t remain = rx_.size() - rx_start_;
    if (remain >= 1 && rx_.back() == HI_HDR0) {
      uint8_t keep = rx_.back();
      rx_.clear();
      rx_.push_back(keep);
      rx_start_ = 0;
    } else {
      rx_.clear();
      rx_start_ = 0;
    }
  }

  // Пробуем извлечь кадры с ограничением по времени/количеству
  this->try_parse_frames_from_buffer_(MAX_PARSE_TIME_MS);
}

climate::ClimateTraits ACHIClimate::traits() {
  climate::ClimateTraits t{};
  t.set_supports_action(false);
  // Без AUTO
  t.set_supported_modes({
    climate::CLIMATE_MODE_OFF,
    climate::CLIMATE_MODE_COOL,
    climate::CLIMATE_MODE_HEAT,
    climate::CLIMATE_MODE_DRY,
    climate::CLIMATE_MODE_FAN_ONLY
  });
  t.set_supported_fan_modes({climate::CLIMATE_FAN_AUTO, climate::CLIMATE_FAN_LOW,
                             climate::CLIMATE_FAN_MEDIUM, climate::CLIMATE_FAN_HIGH,
                             climate::CLIMATE_FAN_QUIET});
  t.set_supported_swing_modes({
    climate::CLIMATE_SWING_OFF, climate::CLIMATE_SWING_VERTICAL,
    climate::CLIMATE_SWING_HORIZONTAL, climate::CLIMATE_SWING_BOTH
  });
  t.set_visual_min_temperature(16);
  t.set_visual_max_temperature(32);
  t.set_visual_temperature_step(1.0f);
  return t;
}

void ACHIClimate::control(const climate::ClimateCall &call) {
  if (call.get_mode().has_value()) {
    auto m = *call.get_mode();
    if (m == climate::CLIMATE_MODE_OFF) {
      this->power_on_ = false;
    } else {
      this->power_on_ = true;
      this->mode_ = m;
    }
  }
  if (call.get_target_temperature().has_value()) {
    float v = *call.get_target_temperature();
    if (v < 16) v = 16;
    if (v > 32) v = 32;
    this->target_c_ = static_cast<uint8_t>(std::round(v));
  }
  if (call.get_fan_mode().has_value()) {
    this->fan_ = *call.get_fan_mode();
  }
  if (call.get_swing_mode().has_value()) {
    this->swing_ = *call.get_swing_mode();
  }

  if (call.get_preset().has_value()) {
    if (!enable_presets_) {
      // игнорируем, если пресеты отключены
    } else {
      auto p = *call.get_preset();
      if (p == climate::CLIMATE_PRESET_SLEEP) {
        this->sleep_stage_ = 1;
        this->turbo_ = false;
        this->eco_ = false;
      } else if (p == climate::CLIMATE_PRESET_ECO) {
        this->eco_ = true;
        this->turbo_ = false;
        this->sleep_stage_ = 0;
      } else if (p == climate::CLIMATE_PRESET_BOOST) {
        this->turbo_ = true;
        this->eco_ = false;
        this->sleep_stage_ = 0;
      } else {
        this->sleep_stage_ = 0;
        this->turbo_ = false;
        this->eco_ = false;
      }
    }
  }

  // Обновляем буфер исходящих и отправляем
  this->patch_tx_bytes_from_state_();
  this->send_write_changes_();

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

// ---- Транспорт/CRC ----

void ACHIClimate::send_query_status_() {
  
#ifdef USE_LOGGER
  #if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_DEBUG
    ESP_LOGD(TAG, "TX query: %s", ac_hi_bytes_to_hex_(this->query_).c_str());
  #endif
#endif
for (auto b : this->query_) this->write_byte(b);
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
  
#ifdef USE_LOGGER
  #if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_DEBUG
    ESP_LOGD(TAG, "TX write: %s", ac_hi_bytes_to_hex_(frame).c_str());
  #endif
#endif
for (auto b : frame) this->write_byte(b);
  this->flush();
}

// ---- RX сканер/парсер ----

void ACHIClimate::try_parse_frames_from_buffer_(uint32_t budget_ms) {
  std::vector<uint8_t> frame;
  uint8_t handled = 0;
  const uint32_t start = esphome::millis();

  while (handled < MAX_FRAMES_PER_LOOP &&
         (esphome::millis() - start) < budget_ms &&
         this->extract_next_frame_(frame)) {

    // На входе используем сумму как «детектор изменений», а не жёсткий CRC-drop
    uint16_t sum = 0;
    for (size_t i = 2; i + 4 <= frame.size(); i++) sum = static_cast<uint16_t>(sum + frame[i]);

    // Разбор
    
#ifdef USE_LOGGER
  #if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_DEBUG
    ESP_LOGD(TAG, "RX: %s", ac_hi_bytes_to_hex_(frame).c_str());
  #endif
#endif
this->handle_frame_(frame);
    handled++;

    // Сохраняем последнюю сумму, чтобы подавлять повторы
    last_status_crc_ = sum;
  }
}

bool ACHIClimate::extract_next_frame_(std::vector<uint8_t> &frame) {
  frame.clear();

  if (rx_.size() <= rx_start_ + 5) return false;

  // 1) Ищем заголовок F4 F5 начиная с rx_start_
  size_t i = rx_start_;
  bool found_header = false;
  for (; i + 1 < rx_.size(); i++) {
    if (rx_[i] == HI_HDR0 && rx_[i + 1] == HI_HDR1) {
      found_header = true;
      break;
    }
  }
  if (!found_header) {
    if (!rx_.empty() && rx_.back() == HI_HDR0) {
      uint8_t keep = rx_.back();
      rx_.clear();
      rx_.push_back(keep);
      rx_start_ = 0;
    } else {
      rx_.clear();
      rx_start_ = 0;
    }
    return false;
  }

  // 2) Декларированная длина (байт 4): полный размер кадра = bytes[4] + 9
  if (i + 5 >= rx_.size()) return false;
  const uint8_t decl_len = rx_[i + 4];
  const size_t full_len = static_cast<size_t>(decl_len) + 9;

  // 3) Если полный кадр ещё не пришёл — ждём
  if (i + full_len > rx_.size()) return false;

  // 4) Ищем хвост F4 FB (от конца заявленного окна)
  size_t j = i + full_len - 2;
  bool found_tail = false;
  for (; j + 1 < rx_.size(); j++) {
    if (rx_[j] == HI_TAIL0 && rx_[j + 1] == HI_TAIL1) {
      found_tail = true;
      break;
    }
  }
  if (!found_tail) return false;

  frame.insert(frame.end(),
               rx_.begin() + static_cast<std::ptrdiff_t>(rx_start_),
               rx_.begin() + static_cast<std::ptrdiff_t>(j + 2));
  rx_start_ = j + 2;
  return true;
}

void ACHIClimate::handle_frame_(const std::vector<uint8_t> &b) {
  if (b.size() < 20) return;

  const uint8_t cmd = b[13];

  if (cmd == 0x10) {
    // ACK на запись настроек
    this->handle_ack_101_();
    return;
  }

  if (cmd != 0x01) return; // интересует только статус

  // Power (byte 8 bit7)
  this->power_on_ = ((b[8] & 0x80) != 0);

  // Mode (byte 18 >> 4)
  this->mode_ = decode_mode_from_nibble(static_cast<uint8_t>(b[18] >> 4));

  // Fan (byte 18 lower nibble pattern)
  uint8_t raw_fan = static_cast<uint8_t>(b[18] & 0x0F);
  switch (raw_fan) {
    case 0b0110: this->fan_ = climate::CLIMATE_FAN_AUTO; break;
    case 0b0100: this->fan_ = climate::CLIMATE_FAN_LOW; break;
    case 0b0010: this->fan_ = climate::CLIMATE_FAN_MEDIUM; break;
    case 0b0001: this->fan_ = climate::CLIMATE_FAN_HIGH; break;
    case 0b1000: this->fan_ = climate::CLIMATE_FAN_QUIET; break;
    default:     this->fan_ = climate::CLIMATE_FAN_AUTO; break;
  }

  // Swing (byte 16 bits 0..1)
  const bool ud = (b[16] & 0b00000001) != 0;
  const bool lr = (b[16] & 0b00000010) != 0;
  if (ud && lr) this->swing_ = climate::CLIMATE_SWING_BOTH;
  else if (ud)  this->swing_ = climate::CLIMATE_SWING_VERTICAL;
  else if (lr)  this->swing_ = climate::CLIMATE_SWING_HORIZONTAL;
  else          this->swing_ = climate::CLIMATE_SWING_OFF;

  // Sleep (байт 17)
  {
    uint8_t raw_sleep = b[17];
    uint8_t code = (raw_sleep >> 1);
    if (code == 0) this->sleep_stage_ = 0;
    else if (code == 1) this->sleep_stage_ = 1;
    else if (code == 2) this->sleep_stage_ = 2;
    else if (code == 4) this->sleep_stage_ = 3;
    else if (code == 8) this->sleep_stage_ = 4;
    else this->sleep_stage_ = 0;
  }

  // Целевая температура (байт 19) — °C напрямую
  uint8_t raw_set = b[19];
  if (raw_set >= 16 && raw_set <= 32) this->target_c_ = raw_set;
  this->target_temperature = this->target_c_;

  // Публикуем состояние
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

#ifdef USE_SENSOR
  if (this->pipe_sensor_ != nullptr) {
    // Заглушка: если понадобится парсить температуру трубки — публиковать здесь
  }
#endif
}

// ---- Построение исходящего кадра из желаемого состояния ----

void ACHIClimate::patch_tx_bytes_from_state_() {
  if (this->tx_bytes_.size() < 24) this->tx_bytes_.resize(24, 0x00);

  // Power
  if (this->power_on_) this->tx_bytes_[8] |= 0x80;
  else this->tx_bytes_[8] &= static_cast<uint8_t>(~0x80);

  // Mode (byte 18 high nibble)
  {
    uint8_t nib = encode_nibble_from_mode(this->mode_);
    this->tx_bytes_[18] &= 0x0F;
    this->tx_bytes_[18] |= static_cast<uint8_t>(nib << 4);
  }

  // Fan (byte 18 low nibble)
  {
    uint8_t fbits;
    switch (this->fan_) {
      case climate::CLIMATE_FAN_LOW:    fbits = 0b0100; break;
      case climate::CLIMATE_FAN_MEDIUM: fbits = 0b0010; break;
      case climate::CLIMATE_FAN_HIGH:   fbits = 0b0001; break;
      case climate::CLIMATE_FAN_QUIET:  fbits = 0b1000; break;
      case climate::CLIMATE_FAN_AUTO:
      default:                          fbits = 0b0110; break;
    }
    this->tx_bytes_[18] &= 0xF0;
    this->tx_bytes_[18] |= (fbits & 0x0F);
  }

  // Swing (byte 16 bits 0..1)
  switch (this->swing_) {
    case climate::CLIMATE_SWING_VERTICAL:   this->tx_bytes_[16] = 0b00000001; break;
    case climate::CLIMATE_SWING_HORIZONTAL: this->tx_bytes_[16] = 0b00000010; break;
    case climate::CLIMATE_SWING_BOTH:       this->tx_bytes_[16] = 0b00000011; break;
    case climate::CLIMATE_SWING_OFF:
    default:                                this->tx_bytes_[16] = 0b00000000; break;
  }

  // Presets -> байт 17 + флаги
  if (enable_presets_) {
    if (this->turbo_) {
      this->tx_bytes_[17] = 0; // turbo реализуется другими флагами, здесь примем 0
    } else if (this->eco_) {
      this->tx_bytes_[17] = 0; // eco — аналогично
    } else {
      this->tx_bytes_[17] = (this->sleep_stage_ == 0) ? 0 : static_cast<uint8_t>(this->sleep_stage_ << 1);
    }
  } else {
    this->tx_bytes_[17] = 0;
  }

  // Target temperature
  this->tx_bytes_[19] = this->target_c_;
}

// ---- Обёртки/декодеры для preset/sleep (опционально) ----

uint8_t ACHIClimate::encode_sleep_(uint8_t stage) {
  switch (stage) {
    case 0: return 0;     // off
    case 1: return 1 << 1;
    case 2: return 2 << 1;
    case 3: return 4 << 1;
    case 4: return 8 << 1;
    default: return 0;
  }
}

uint8_t ACHIClimate::encode_swing_ud_(bool on) {
  return on ? 0b11000000 : 0b01000000;
}
uint8_t ACHIClimate::encode_swing_lr_(bool on) {
  return on ? 0b00110000 : 0b00010000;
}

void ACHIClimate::handle_ack_101_() {
  this->writing_lock_ = false;
  this->pending_write_ = false;
}

} // namespace ac_hi
} // namespace esphome
