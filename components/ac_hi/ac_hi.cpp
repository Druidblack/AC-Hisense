#include "ac_hi.h"
#include <cmath>
#include <algorithm>
#include "esphome/core/helpers.h"  // for format_hex_pretty
// NOTE: Logging is VERBOSE only and does not change behavior.

namespace esphome {
namespace ac_hi {

static const char *const TAG = "ac_hi"; // Logger tag

// ---- Local helpers (logging only) ----
// Convert enums to readable strings for logs (no functional impact)
static inline const char *mode_to_str(climate::ClimateMode m) {
  switch (m) {
    case climate::CLIMATE_MODE_OFF:       return "OFF";
    case climate::CLIMATE_MODE_COOL:      return "COOL";
    case climate::CLIMATE_MODE_HEAT:      return "HEAT";
    case climate::CLIMATE_MODE_DRY:       return "DRY";
    case climate::CLIMATE_MODE_FAN_ONLY:  return "FAN_ONLY";
    case climate::CLIMATE_MODE_AUTO:      return "AUTO";
    default:                              return "UNKNOWN";
  }
}
static inline const char *fan_to_str(climate::ClimateFanMode f) {
  switch (f) {
    case climate::CLIMATE_FAN_AUTO:   return "AUTO";
    case climate::CLIMATE_FAN_LOW:    return "LOW";
    case climate::CLIMATE_FAN_MEDIUM: return "MED";
    case climate::CLIMATE_FAN_HIGH:   return "HIGH";
    case climate::CLIMATE_FAN_QUIET:  return "QUIET";
    default:                          return "UNKNOWN";
  }
}
static inline const char *swing_to_str(climate::ClimateSwingMode s) {
  switch (s) {
    case climate::CLIMATE_SWING_OFF:        return "OFF";
    case climate::CLIMATE_SWING_VERTICAL:   return "VERT";
    case climate::CLIMATE_SWING_HORIZONTAL: return "HORIZ";
    case climate::CLIMATE_SWING_BOTH:       return "BOTH";
    default:                                return "UNKNOWN";
  }
}

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
    case climate::CLIMATE_MODE_FAN_ONLY: return 0x01;
    case climate::CLIMATE_MODE_HEAT:     return 0x03;
    case climate::CLIMATE_MODE_COOL:     return 0x05;
    case climate::CLIMATE_MODE_DRY:      return 0x07;
    default:                             return 0x05; // AUTO нет, используем COOL
  }
}

void ACHIClimate::setup() {
  this->mode = climate::CLIMATE_MODE_OFF;
  this->target_temperature = 24;
  this->fan_mode = climate::CLIMATE_FAN_AUTO;
  this->swing_mode = climate::CLIMATE_SWING_OFF;
  this->publish_state();

  ESP_LOGV(TAG, "Setup complete. presets=%s, init target=%.1f, fan=%s, swing=%s",
           this->enable_presets_ ? "enabled" : "disabled",
           this->target_temperature, fan_to_str(this->fan_mode), swing_to_str(this->swing_mode));
}

void ACHIClimate::update() {
  // Периодический опрос: короткий запрос статуса
  if (!this->writing_lock_) {
    ESP_LOGV(TAG, "Update tick: requesting status (no pending write).");
    this->send_query_status_();
  } else {
    ESP_LOGV(TAG, "Update tick: write in progress, skipping status query.");
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
    ESP_LOGV(TAG, "RX buffer compacted.");
  }
  if (rx_.size() - rx_start_ > 4096) {
    size_t remain = rx_.size() - rx_start_;
    ESP_LOGV(TAG, "RX buffer large (%u), trimming. remain=%u", (unsigned) (rx_.size()), (unsigned) remain);
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
  t.set_supported_swing_modes({climate::CLIMATE_SWING_OFF, climate::CLIMATE_SWING_VERTICAL,
                               climate::CLIMATE_SWING_HORIZONTAL, climate::CLIMATE_SWING_BOTH});
  if (enable_presets_) {
    t.set_supported_presets({climate::CLIMATE_PRESET_NONE, climate::CLIMATE_PRESET_ECO,
                             climate::CLIMATE_PRESET_BOOST, climate::CLIMATE_PRESET_SLEEP});
  }
  t.set_visual_min_temperature(16);
  t.set_visual_max_temperature(30);
  t.set_visual_temperature_step(1.0f);
  t.set_supports_current_temperature(true);
  return t;
}

void ACHIClimate::control(const climate::ClimateCall &call) {
  bool need_write = false;

  if (call.get_mode().has_value()) {
    auto m = *call.get_mode();
    ESP_LOGV(TAG, "Control: requested mode=%s", mode_to_str(m));
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
    ESP_LOGV(TAG, "Control: requested target=%.1f", t);
    if (!std::isnan(t)) {
      uint8_t c = static_cast<uint8_t>(std::round(t));
      c = std::max<uint8_t>(16, std::min<uint8_t>(30, c));
      this->target_c_ = c;
      need_write = true;
    }
  }

  if (call.get_fan_mode().has_value()) {
    this->fan_ = *call.get_fan_mode();
    ESP_LOGV(TAG, "Control: requested fan=%s", fan_to_str(this->fan_));
    need_write = true;
  }

  if (call.get_swing_mode().has_value()) {
    this->swing_ = *call.get_swing_mode();
    ESP_LOGV(TAG, "Control: requested swing=%s", swing_to_str(this->swing_));
    need_write = true;
  }

  if (call.get_preset().has_value()) {
    auto p = *call.get_preset();
    ESP_LOGV(TAG, "Control: requested preset=%s",
             (p == climate::CLIMATE_PRESET_ECO ? "ECO" :
              p == climate::CLIMATE_PRESET_BOOST ? "BOOST" :
              p == climate::CLIMATE_PRESET_SLEEP ? "SLEEP" : "NONE"));
    this->eco_ = (p == climate::CLIMATE_PRESET_ECO);
    this->turbo_ = (p == climate::CLIMATE_PRESET_BOOST);
    this->sleep_stage_ = (p == climate::CLIMATE_PRESET_SLEEP) ? 1 : 0;
    need_write = true;
  }

  if (need_write) {
    // Сборка TX-кадра (no functional changes)
    uint8_t power_bin = this->power_on_ ? 0b00001100 : 0b00000100;  // low nibble
    uint8_t mode_hi   = static_cast<uint8_t>(encode_nibble_from_mode(this->mode_) << 4);
    tx_bytes_[18] = static_cast<uint8_t>(power_bin + mode_hi);

    // запись уставки напрямую
    tx_bytes_[19] = encode_temp_(this->target_c_);

    tx_bytes_[16] = encode_fan_byte_(this->fan_);           // fan
    tx_bytes_[17] = encode_sleep_byte_(this->sleep_stage_); // sleep

    bool v_swing = (this->swing_ == climate::CLIMATE_SWING_VERTICAL) || (this->swing_ == climate::CLIMATE_SWING_BOTH);
    bool h_swing = (this->swing_ == climate::CLIMATE_SWING_HORIZONTAL) || (this->swing_ == climate::CLIMATE_SWING_BOTH);
    uint8_t updown_bin = encode_swing_ud_(v_swing);
    uint8_t leftright_bin = encode_swing_lr_(h_swing);
    tx_bytes_[32] = static_cast<uint8_t>(updown_bin + leftright_bin);

    uint8_t turbo_bin = this->turbo_ ? 0b00001100 : 0b00000100;
    uint8_t eco_bin = this->eco_ ? 0b00110000 : 0b00000000;
    tx_bytes_[33] = (this->turbo_ ? turbo_bin : (eco_bin ? eco_bin : 0));

    this->quiet_ = (this->fan_ == climate::CLIMATE_FAN_QUIET);
    tx_bytes_[35] = this->quiet_ ? 0b00110000 : 0b00000000;

    tx_bytes_[36] = this->led_ ? 0b11000000 : 0b01000000;

    if (this->turbo_) {
      tx_bytes_[19] = this->target_c_; // turbo не трогаем уставку
      tx_bytes_[33] = turbo_bin;
      tx_bytes_[35] = 0;               // override quiet
    }

    ESP_LOGV(TAG,
             "Control: assemble write -> power=%s mode=%s set=%u fan=%s sleep=%u swing=%s (ud=%s lr=%s) turbo=%d eco=%d quiet=%d led=%d",
             this->power_on_ ? "ON" : "OFF",
             mode_to_str(this->mode_), this->target_c_, fan_to_str(this->fan_), this->sleep_stage_,
             swing_to_str(this->swing_), v_swing ? "on" : "off", h_swing ? "on" : "off",
             this->turbo_, this->eco_, this->quiet_, this->led_);

    this->writing_lock_ = true;
    this->pending_write_ = true;
    ESP_LOGV(TAG, "Write lock set. Pending write = true.");
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
  ESP_LOGV(TAG, "State published (optimistic). mode=%s, target=%.1f, fan=%s, swing=%s",
           mode_to_str(this->mode), this->target_temperature, fan_to_str(this->fan_mode), swing_to_str(this->swing_mode));
}

// ---- Кодирование полей ----

uint8_t ACHIClimate::encode_mode_hi_nibble_(climate::ClimateMode m) {
  // Сформировать «старший полубайт»: nibble {0..3} << 4
  return static_cast<uint8_t>(encode_nibble_from_mode(m) << 4);
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

// ---- Транспорт/CRC ----

void ACHIClimate::send_query_status_() {
  ESP_LOGV(TAG, "TX Query [0x66]: %s", format_hex_pretty(this->query_).c_str());
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
  ESP_LOGV(TAG, "CRC patched: sum=0x%04X -> bytes[%d]=0x%02X, bytes[%d]=0x%02X",
           csum, n - 4, buf[n - 4], n - 3, buf[n - 3]);
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
  ESP_LOGV(TAG, "TX Write [0x65]: %s", format_hex_pretty(frame).c_str());
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

    // Дополнительно — логируем валидность CRC (лог только, без влияния на логику)
    uint16_t crc_calc = 0;
    bool crc_ok = this->validate_crc_(frame, &crc_calc);
    ESP_LOGV(TAG, "RX Frame (%u bytes, sum=0x%04X, crc=%s): %s",
             (unsigned) frame.size(), sum, crc_ok ? "OK" : "MISMATCH",
             format_hex_pretty(frame).c_str());

    // Разбор
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

  rx_start_ = i;

  // 2) Попытка среза по объявленной длине: полный размер = bytes[4] + 9
  if (rx_.size() > rx_start_ + 5) {
    uint8_t decl = rx_[rx_start_ + 4];
    size_t expected_total = static_cast<size_t>(decl) + 9U;

    if (rx_.size() >= rx_start_ + expected_total) {
      frame.insert(frame.end(),
                   rx_.begin() + static_cast<std::ptrdiff_t>(rx_start_),
                   rx_.begin() + static_cast<std::ptrdiff_t>(rx_start_ + expected_total));
      rx_start_ += expected_total;
      // Do not log here to avoid double logging; we log after extraction in try_parse...
      return true;
    }
  }

  // 3) Иначе — до первого хвоста F4 FB
  size_t j = rx_start_ + 2;
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
  if (b.size() < 20) {
    ESP_LOGV(TAG, "RX frame too short: %u bytes, ignored.", (unsigned) b.size());
    return;
  }

  const uint8_t cmd = b[13];
  ESP_LOGV(TAG, "Handle frame: cmd=0x%02X", cmd);

  if (cmd == 102 /*0x66 status resp*/ ) {
    this->parse_status_102_(b);
  } else if (cmd == 101 /*0x65 ack*/) {
    this->handle_ack_101_();
  } else {
    // неизвестные кадры игнорируем
    ESP_LOGV(TAG, "Unknown cmd=0x%02X, ignored.", cmd);
  }
}

void ACHIClimate::parse_status_102_(const std::vector<uint8_t> &bytes) {
  // Питание (байт 18, бит 3)
  bool power = (bytes[18] & 0b00001000) != 0;
  this->power_on_ = power;

  // Режим — верхний полубайт по таблице FAN/HEAT/COOL/DRY
  uint8_t nib = static_cast<uint8_t>((bytes[18] >> 4) & 0x0F);
  this->mode_ = decode_mode_from_nibble(nib);

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

  // Целевая температура (байт 19) — значение в °C напрямую
  uint8_t raw_set = bytes[19];
  if (raw_set >= 16 && raw_set <= 30) this->target_c_ = raw_set;
  this->target_temperature = this->target_c_;

  // Текущая температура воздуха (байт 20)
  uint8_t tair = bytes[20];
  this->current_temperature = tair;

  // Температура трубки (байт 21)
#ifdef USE_SENSOR
  if (this->pipe_sensor_ != nullptr) this->pipe_sensor_->publish_state(bytes[21]);
#endif

  // === Публикация всех доп. сенсоров из Legacy ===
#ifdef USE_SENSOR
  // Уставка / комнатная / скорость вентилятора / сон / числовой код режима
  if (this->set_temp_sensor_ != nullptr) this->set_temp_sensor_->publish_state(this->target_c_);
  if (this->room_temp_sensor_ != nullptr) this->room_temp_sensor_->publish_state(tair);
  if (this->wind_sensor_ != nullptr) this->wind_sensor_->publish_state(raw_wind);
  if (this->sleep_sensor_ != nullptr) this->sleep_sensor_->publish_state(this->sleep_stage_);
  if (this->mode_sensor_ != nullptr) this->mode_sensor_->publish_state(nib & 0x0F);
#endif

#ifdef USE_TEXT_SENSOR
  // Текстовый статус питания
  if (this->power_status_text_sensor_ != nullptr) this->power_status_text_sensor_->publish_state(this->power_on_ ? "ON" : "OFF");
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

#ifdef USE_SENSOR
  // Флаги и качание как бинарные числовые сенсоры (0/1)
  if (this->quiet_sensor_ != nullptr) this->quiet_sensor_->publish_state(this->quiet_ ? 1 : 0);
  if (this->turbo_sensor_ != nullptr) this->turbo_sensor_->publish_state(this->turbo_ ? 1 : 0);
  if (this->led_sensor_ != nullptr)   this->led_sensor_->publish_state(this->led_ ? 1 : 0);
  if (this->eco_sensor_ != nullptr)   this->eco_sensor_->publish_state(this->eco_ ? 1 : 0);
  if (this->swing_updown_sensor_ != nullptr)    this->swing_updown_sensor_->publish_state(updown ? 1 : 0);
  if (this->swing_leftright_sensor_ != nullptr) this->swing_leftright_sensor_->publish_state(leftright ? 1 : 0);
#endif

#ifdef USE_SENSOR
  // Наружные температуры и частоты компрессора (байты 42..45)
  if (this->compr_freq_set_sensor_ != nullptr) this->compr_freq_set_sensor_->publish_state(bytes[42]);
  if (this->compr_freq_sensor_ != nullptr)     this->compr_freq_sensor_->publish_state(bytes[43]);
  if (this->outdoor_temp_sensor_ != nullptr)   this->outdoor_temp_sensor_->publish_state(bytes[44]);
  if (this->outdoor_cond_temp_sensor_ != nullptr) this->outdoor_cond_temp_sensor_->publish_state(bytes[45]);
#endif
  // === конец добавленных публикаций ===

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

  // Log a concise status summary for diagnostics
  ESP_LOGV(TAG,
           "Parsed status: power=%s mode=%s fan=%s set=%u room=%u sleep=%u swing=%s (ud=%d lr=%d) flags: turbo=%d eco=%d quiet=%d led=%d",
           this->power_on_ ? "ON" : "OFF",
           mode_to_str(this->mode_), fan_to_str(this->fan_),
           this->target_c_, tair, this->sleep_stage_, swing_to_str(this->swing_),
           updown ? 1 : 0, leftright ? 1 : 0,
           this->turbo_, this->eco_, this->quiet_, this->led_);
}

void ACHIClimate::handle_ack_101_() {
  this->writing_lock_ = false;
  this->pending_write_ = false;
  ESP_LOGV(TAG, "ACK[0x65] received. Write lock cleared. Pending write = false.");
}

} // namespace ac_hi
} // namespace esphome
