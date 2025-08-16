#include "ac_hi.h"
#include <cmath>
#include <algorithm>  // std::min/std::max

namespace esphome {
namespace ac_hi {

static const char *const TAG = "ac_hi.climate";

void ACHIClimate::setup() {
  ESP_LOGI(TAG, "Setup AC-Hi climate");
  // initial climate state
  this->mode = climate::CLIMATE_MODE_OFF;
  this->target_temperature = 24;
  this->fan_mode = climate::CLIMATE_FAN_AUTO;
  this->swing_mode = climate::CLIMATE_SWING_OFF;
  this->publish_state();
}

void ACHIClimate::update() {
  // poll status if not writing
  if (!this->writing_lock_) {
    this->send_query_status_();
  }
}

void ACHIClimate::loop() {
  while (this->available()) {
    uint8_t c = this->read();
    rx_.push_back(c);
    // try to detect tail 0xF4 0xFB
    size_t n = rx_.size();
    if (n >= 2 && rx_[n - 2] == 0xF4 && rx_[n - 1] == 0xFB) {
      // find header
      if (n >= 4 && rx_[0] == 0xF4 && rx_[1] == 0xF5) {
        this->handle_frame_(rx_);
      } else {
        ESP_LOGW(TAG, "Discarding frame: bad header");
      }
      rx_.clear();
    }
    // avoid run-away buffer
    if (rx_.size() > 128) rx_.clear();
  }
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
    if (!std::isnan(t)) {  // <cmath> std::isnan
      uint8_t c = static_cast<uint8_t>(std::round(t));  // <cmath> std::round
      c = std::max<uint8_t>(18, std::min<uint8_t>(28, c));  // <algorithm> std::min/max
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
    // Eco/Boost/Sleep mapping
    this->eco_ = (p == climate::CLIMATE_PRESET_ECO);
    this->turbo_ = (p == climate::CLIMATE_PRESET_BOOST);
    this->sleep_stage_ = (p == climate::CLIMATE_PRESET_SLEEP) ? 1 : 0;
    need_write = true;
  }

  if (need_write) {
    // Rebuild tx_bytes_ fields
    // Byte 18 = power + mode
    uint8_t power_bin = this->power_on_ ? 0b00001100 : 0b00000100;  // из YAML
    uint8_t mode_hi = encode_mode_hi_nibble_(this->mode_);
    tx_bytes_[18] = power_bin + mode_hi;

    // Byte 19 target temp encoded
    tx_bytes_[19] = encode_temp_(this->target_c_);

    // Byte 16 fan
    tx_bytes_[16] = encode_fan_byte_(this->fan_);

    // Byte 17 sleep
    tx_bytes_[17] = encode_sleep_byte_(this->sleep_stage_);

    // Byte 32 swing merge
    bool v_swing = (this->swing_ == climate::CLIMATE_SWING_VERTICAL) || (this->swing_ == climate::CLIMATE_SWING_BOTH);
    bool h_swing = (this->swing_ == climate::CLIMATE_SWING_HORIZONTAL) || (this->swing_ == climate::CLIMATE_SWING_BOTH);
    uint8_t updown_bin = encode_swing_ud_(v_swing);
    uint8_t leftright_bin = encode_swing_lr_(h_swing);
    tx_bytes_[32] = updown_bin + leftright_bin;

    // Byte 33 turbo/eco
    uint8_t turbo_bin = this->turbo_ ? 0b00001100 : 0b00000100;
    uint8_t eco_bin = this->eco_ ? 0b00110000 : 0b00000000;
    tx_bytes_[33] = (this->turbo_ ? turbo_bin : (eco_bin ? eco_bin : 0));

    // Byte 35 quiet (fan quiet => биты)
    this->quiet_ = (this->fan_ == climate::CLIMATE_FAN_QUIET);
    tx_bytes_[35] = this->quiet_ ? 0b00110000 : 0b00000000;

    // Byte 36 LED
    tx_bytes_[36] = this->led_ ? 0b11000000 : 0b01000000;

    // Turbo overrides eco & quiet (как в YAML)
    if (this->turbo_) {
      tx_bytes_[19] = 0;    // override temperature
      tx_bytes_[33] = turbo_bin;
      tx_bytes_[35] = 0;    // override quiet
    }

    // finalize + send
    this->writing_lock_ = true;
    this->pending_write_ = true;
    this->send_write_changes_();
  }

  // Оптимистично публикуем ожидаемое состояние — статус 0x66/0x102 его подтвердит/уточнит
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

uint8_t ACHIClimate::encode_mode_hi_nibble_(climate::ClimateMode m) {
  // YAML: listix_to_mode_codes {0,1,2,3,4}; mode = code<<1 | 1; затем <<4
  uint8_t code = 0;
  switch (m) {
    case climate::CLIMATE_MODE_FAN_ONLY: code = 0; break;
    case climate::CLIMATE_MODE_HEAT: code = 1; break;
    case climate::CLIMATE_MODE_COOL: code = 2; break;
    case climate::CLIMATE_MODE_DRY: code = 3; break;
    case climate::CLIMATE_MODE_AUTO: code = 4; break;
    default: code = 4; break;
  }
  uint8_t v = ((code << 1) | 0x01) << 4;
  return v;
}

uint8_t ACHIClimate::encode_fan_byte_(climate::ClimateFanMode f) {
  // YAML listix_to_wind_codes {0,1,10,12,14,16,18}; +1 при записи
  // Соответствие: AUTO->1, QUIET->10, LOW->12, MED->14, HIGH->16
  uint8_t code = 1;
  switch (f) {
    case climate::CLIMATE_FAN_AUTO: code = 1; break;
    case climate::CLIMATE_FAN_LOW: code = 12; break;
    case climate::CLIMATE_FAN_MEDIUM: code = 14; break;
    case climate::CLIMATE_FAN_HIGH: code = 16; break;
    case climate::CLIMATE_FAN_QUIET: code = 10; break;
    default: code = 1; break;
  }
  return code + 1;
}

uint8_t ACHIClimate::encode_sleep_byte_(uint8_t stage) {
  // YAML listix_to_sleep_codes {0,1,2,4,8}; mode = (code<<1) | 1
  uint8_t code = 0;
  switch (stage) {
    case 1: code = 1; break;
    case 2: code = 2; break;
    case 3: code = 4; break;
    case 4: code = 8; break;
    default: code = 0; break;
  }
  return (code << 1) | 0x01;
}

uint8_t ACHIClimate::encode_swing_ud_(bool on) {
  return on ? 0b11000000 : 0b01000000;
}
uint8_t ACHIClimate::encode_swing_lr_(bool on) {
  return on ? 0b00110000 : 0b00010000;
}

void ACHIClimate::send_query_status_() {
  ESP_LOGV(TAG, ">> Query status");
  for (auto b : this->query_) this->write_byte(b);
  this->flush();
}

void ACHIClimate::calc_and_patch_crc_(std::vector<uint8_t> &buf) {
  // сумма байтов с 2 по (len-4), затем два CRC-байта
  int arrlen = buf.size();
  uint16_t csum = 0;
  for (int i = 2; i < arrlen - 4; i++) csum += buf[i];
  uint8_t cr1 = (csum & 0xFF00) >> 8;
  uint8_t cr2 = (csum & 0x00FF);
  buf[arrlen - 4] = cr1;
  buf[arrlen - 3] = cr2;
}

void ACHIClimate::send_write_changes_() {
  ESP_LOGI(TAG, ">> Write changes");
  auto frame = this->tx_bytes_;
  this->calc_and_patch_crc_(frame);
  for (auto b : frame) this->write_byte(b);
  this->flush();
}

void ACHIClimate::handle_frame_(const std::vector<uint8_t> &b) {
  if (b.size() < 20) return;
  uint8_t cmd = b[13];
  uint8_t typ = b[2];

  if (cmd == 102 && typ == 1 && !this->writing_lock_) {
    // status stream
    this->parse_status_102_(b);
  } else if (cmd == 101 && typ == 1) {
    this->handle_ack_101_();
  } else if (cmd == 102 && typ == 1 && this->writing_lock_) {
    // при записи тоже принимаем обновления, но lock не снимаем до ACK 101
    this->parse_status_102_(b);
  }
}

void ACHIClimate::parse_status_102_(const std::vector<uint8_t> &bytes) {
  // CRC для детекта изменений
  uint16_t crc = 0;
  for (size_t i = 2; i < bytes.size() - 4; i++) crc += bytes[i];
  if (crc == last_status_crc_) {
    ESP_LOGV(TAG, "<< Status 102: unchanged");
    return;
  }
  last_status_crc_ = crc;

  // Power (байт 18, бит 3)
  bool power = (bytes[18] & 0b00001000) != 0;
  this->power_on_ = power;

  // Mode (старший полубайт байта 18: {fan_only,heat,cool,dry,auto})
  uint8_t mode_code = (bytes[18] >> 4) & 0x0F;
  climate::ClimateMode new_mode = climate::CLIMATE_MODE_AUTO;
  switch (mode_code) {
    case 0x01: new_mode = climate::CLIMATE_MODE_FAN_ONLY; break;  // (0<<1 |1) = 1
    case 0x03: new_mode = climate::CLIMATE_MODE_HEAT; break;      // (1<<1 |1) = 3
    case 0x05: new_mode = climate::CLIMATE_MODE_COOL; break;      // (2<<1 |1) = 5
    case 0x07: new_mode = climate::CLIMATE_MODE_DRY; break;       // (3<<1 |1) = 7
    case 0x09: new_mode = climate::CLIMATE_MODE_AUTO; break;      // (4<<1 |1) = 9
    default: new_mode = climate::CLIMATE_MODE_AUTO; break;
  }
  this->mode_ = new_mode;

  // Fan (байт 16)
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

  // Target temperature (байт 19, код 2*n+1)
  uint8_t raw_set = bytes[19];
  if (raw_set & 0x01) {
    uint8_t c = raw_set >> 1;
    if (c >= 18 && c <= 28) this->target_c_ = c;
  }
  this->target_temperature = this->target_c_;

  // Current air temperature (байт 20)
  uint8_t tair = bytes[20];
  this->current_temperature = tair;

  // Pipe temp (байт 21)
#ifdef USE_SENSOR
  if (this->pipe_sensor_ != nullptr) this->pipe_sensor_->publish_state(bytes[21]);
#endif

  // Turbo/Eco/Quiet/LED по YAML
  uint8_t b35 = bytes[35];
  this->turbo_ = (b35 & 0b00000010) != 0;       // turbo_mask = 0b00000010
  this->eco_ = (b35 & 0b00000100) != 0;         // eco_mask   = 0b00000100
  this->quiet_ = (bytes[36] & 0b00000100) != 0; // quiet из 36-го
  this->led_ = (bytes[37] & 0b10000000) != 0;   // LED из 37-го

  // Swing (байт 35: updown bit7, leftright bit6)
  bool updown = (bytes[35] & 0b10000000) != 0;
  bool leftright = (bytes[35] & 0b01000000) != 0;
  if (updown && leftright) this->swing_ = climate::CLIMATE_SWING_BOTH;
  else if (updown) this->swing_ = climate::CLIMATE_SWING_VERTICAL;
  else if (leftright) this->swing_ = climate::CLIMATE_SWING_HORIZONTAL;
  else this->swing_ = climate::CLIMATE_SWING_OFF;

  // Publish climate
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
  ESP_LOGD(TAG, "<< ACK 101 received; unlock");
  this->writing_lock_ = false;
  this->pending_write_ = false;
}

} // namespace ac_hi
} // namespace esphome
