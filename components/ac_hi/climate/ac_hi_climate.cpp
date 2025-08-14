// SPDX-License-Identifier: MIT
#include "ac_hi_climate.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ac_hi {

static const char *const TAG = "ac_hi.climate";

// ======================= Setup & Config =======================

void ACHiClimate::setup() {
  ESP_LOGI(TAG, "Init climate over RS-485");
  this->check_uart_settings(9600, 1, uart::UART_CONFIG_PARITY_NONE, 8);

  // Build outbound template once
  this->out_.assign(50, 0x00);
  // Header & footer per legacy
  this->out_[0] = 0xF4; this->out_[1] = 0xF5;
  this->out_[2] = 0x00;           // addr/flag как в legacy-трафике
  this->out_[3] = 0x40;
  this->out_[4] = 0x29;           // тип длинного пакета записи, используемый в legacy
  this->out_[8] = 0x01;           // служебные константы, встречающиеся в дампах
  this->out_[9] = 0xFE;
  this->out_[10] = 0x00;
  this->out_[11] = 0x00;
  this->out_[12] = 0x00;
  this->out_[13] = 0x65;          // 101 — команда записи/применения (ожидается unlock 101)
  // bytes [46],[47] — CRC, [48]=0xF4, [49]=0xFB
  this->out_[48] = 0xF4; this->out_[49] = 0xFB;

  // default visual bounds
  this->target_temperature = 24;

  // request first status soon
  this->set_timeout("init_status", 1000, [this]() { this->send_status_request_(); });
}

void ACHiClimate::dump_config() {
  ESP_LOGCONFIG(TAG, "AC-Hi Climate (UART RS-485)");
  ESP_LOGCONFIG(TAG, "  Poll interval: %u ms", this->update_interval_ms_);
}

climate::ClimateTraits ACHiClimate::traits() {
  climate::ClimateTraits t;
  t.set_supports_current_temperature(true);
  t.set_supported_modes({
      climate::CLIMATE_MODE_OFF,
      climate::CLIMATE_MODE_COOL,
      climate::CLIMATE_MODE_HEAT,
      climate::CLIMATE_MODE_DRY,
      climate::CLIMATE_MODE_AUTO,
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
  t.set_visual_min_temperature(16.0f);
  t.set_visual_max_temperature(30.0f);
  t.set_visual_temperature_step(1.0f);
  return t;
}

// ======================= Loop =======================

void ACHiClimate::loop() {
  // RX collect
  while (this->available()) {
    uint8_t b = this->read();
    this->rb_push_(b);
  }
  // Parse frames
  while (this->parse_next_frame_()) {}

  // Periodic status request
  const uint32_t now = millis();
  if (now - this->last_poll_ >= this->update_interval_ms_) {
    this->last_poll_ = now;
    this->send_status_request_();
  }
}

// ======================= Control =======================

void ACHiClimate::control(const climate::ClimateCall &call) {
  bool need_write = false;

  if (call.get_mode().has_value()) {
    auto m = *call.get_mode();
    if (m == climate::CLIMATE_MODE_OFF) {
      power_bin_ = 0b00000100; // держим bit2, чистим bit3
      this->mode = climate::CLIMATE_MODE_OFF;
      this->power_ = false;
    } else {
      power_bin_ = 0b00001100; // bit2 + bit3
      this->mode = m;
      this->power_ = true;

      uint8_t idx = 4; // auto
      if (m == climate::CLIMATE_MODE_HEAT) idx = 1;
      else if (m == climate::CLIMATE_MODE_COOL) idx = 2;
      else if (m == climate::CLIMATE_MODE_DRY) idx = 3;
      else if (m == climate::CLIMATE_MODE_FAN_ONLY) idx = 0;
      mode_bin_ = uint8_t((((idx << 1) | 0x01) << 4));
    }
    need_write = true;
  }

  if (call.get_target_temperature().has_value()) {
    float t = *call.get_target_temperature();
    if (t < 16.0f) t = 16.0f;
    if (t > 30.0f) t = 30.0f;
    this->target_temperature = t;
    temp_byte_ = (uint8_t(t) << 1) | 0x01;
    need_write = true;
  }

  if (call.get_fan_mode().has_value()) {
    auto f = *call.get_fan_mode();
    this->fan_mode = f;
    // map to код ветра
    if (f == climate::CLIMATE_FAN_AUTO)      wind_code_ = 1;
    else if (f == climate::CLIMATE_FAN_LOW)  wind_code_ = 12;
    else if (f == climate::CLIMATE_FAN_MEDIUM) wind_code_ = 14;
    else if (f == climate::CLIMATE_FAN_HIGH) wind_code_ = 16;
    need_write = true;
  }

  if (call.get_swing_mode().has_value()) {
    auto s = *call.get_swing_mode();
    swing_ud_ = (s == climate::CLIMATE_SWING_BOTH);
    swing_lr_ = (s == climate::CLIMATE_SWING_BOTH);
    need_write = true;
  }

  if (need_write) {
    this->send_write_frame_();
  }

  this->publish_state();
}

// ======================= Protocol I/O =======================

void ACHiClimate::send_status_request_() {
  // короткий запрос статуса (cmd 0x66 / 102): снят из legacy
  const uint8_t req[] = {0xF4,0xF5,0x00,0x40,0x0C,0x00,0x00,0x01,0x01,0xFE,0x01,0x00,0x00,0x66,0x00,0x00,0x00,0x01,0xB3,0xF4,0xFB};
  this->write_array(req, sizeof(req));
  this->flush();
  ESP_LOGVV(TAG, "Sent status request (102)");
}

void ACHiClimate::rebuild_write_frame_() {
  // переносим «намерения» в байты пакета
  // [16] wind (протокол просит +1)
  out_[16] = uint8_t(wind_code_ + 1);
  // [18] power + mode
  out_[18] = uint8_t(power_bin_ + mode_bin_);
  // [32] swing (up-down + left-right)
  updown_bin_    = swing_ud_ ? 0b00110000 : 0b00010000;
  leftright_bin_ = swing_lr_ ? 0b00001100 : 0b00000100;
  out_[32] = uint8_t(updown_bin_ + leftright_bin_);
  // [33] turbo + eco
  out_[33] = uint8_t(turbo_bin_ + eco_bin_);
  // [35] quiet
  out_[35] = quiet_bin_;
  // [19] целевая температура (0 — при turbo override)
  out_[19] = (turbo_bin_ == 0b00001100) ? 0x00 : temp_byte_;

  // Явно чистим бит питания (bit3) при OFF, как делалось в legacy
  if ((power_bin_ & 0b00001000) == 0) {
    out_[18] = uint8_t(out_[18] & (~(1U<<3)));
  }

  compute_crc_(out_);
}

void ACHiClimate::send_write_frame_() {
  this->rebuild_write_frame_();
  this->write_array(out_.data(), out_.size());
  this->flush();
  ESP_LOGD(TAG, "Sent write frame (ожидается unlock 101)");
}

void ACHiClimate::compute_crc_(std::vector<uint8_t> &buf) {
  // CRC как в legacy: сумма байт с 2 по size-5, в [46],[47]; хвост [48]=F4, [49]=FB
  if (buf.size() < 50) return;
  int csum = 0;
  for (size_t i = 2; i < buf.size() - 4; i++) csum += buf[i];
  uint8_t cr1 = (csum & 0xFF00) >> 8;
  uint8_t cr2 = (csum & 0x00FF);
  buf[46] = cr1;
  buf[47] = cr2;
  buf[48] = 0xF4;
  buf[49] = 0xFB;
}

// парсер: кадры начинаются F4 F5 и закрываются F4 FB
bool ACHiClimate::parse_next_frame_() {
  // sync to start
  size_t cnt = 0;
  uint8_t b;
  while (rb_pop_(b)) {
    if (b == 0xF4) {
      uint8_t b2;
      if (!rb_pop_(b2)) { rb_tail_ = (rb_tail_ + RB_SIZE - 1) % RB_SIZE; return false; }
      if (b2 == 0xF5) {
        std::vector<uint8_t> frame;
        frame.push_back(0xF4); frame.push_back(0xF5);
        int guard = 0;
        while (rb_pop_(b)) {
          frame.push_back(b);
          if (frame.size() >= 4 && frame[frame.size()-2] == 0xF4 && frame.back() == 0xFB) {
            if (frame.size() >= 20) handle_status_(frame);
            return true;
          }
          if (++guard > 256) break;
        }
        return false;
      } else {
        // откат b2
        rb_tail_ = (rb_tail_ + RB_SIZE - 1) % RB_SIZE;
      }
    }
    if (++cnt > RB_SIZE) break;
  }
  return false;
}

void ACHiClimate::handle_status_(const std::vector<uint8_t> &bytes) {
  if (bytes.size() < 46) return;

  // интересуют только пакеты cmd=102 (статус)
  if (bytes[13] != 102) {
    // 101 — unlock после записи
    return;
  }

  // Power (bit3 в [18])
  bool new_power = (bytes[18] & 0b00001000) != 0;
  // Mode — старший ниббл [18]
  uint8_t mode_raw = (bytes[18] >> 4) & 0x0F;
  climate::ClimateMode new_mode = climate::CLIMATE_MODE_AUTO;
  switch (mode_raw) {
    case 0: new_mode = climate::CLIMATE_MODE_FAN_ONLY; break;
    case 1: new_mode = climate::CLIMATE_MODE_HEAT;     break;
    case 2: new_mode = climate::CLIMATE_MODE_COOL;     break;
    case 3: new_mode = climate::CLIMATE_MODE_DRY;      break;
    default: new_mode = climate::CLIMATE_MODE_AUTO;    break;
  }

  // Вентилятор — [16]
  uint8_t wind_raw = bytes[16];
  climate::ClimateFanMode new_fan = climate::CLIMATE_FAN_AUTO;
  if (wind_raw == 10) new_fan = climate::CLIMATE_FAN_LOW;
  else if (wind_raw == 12) new_fan = climate::CLIMATE_FAN_MEDIUM;
  else if (wind_raw == 14) new_fan = climate::CLIMATE_FAN_HIGH;
  else if (wind_raw == 1)  new_fan = climate::CLIMATE_FAN_AUTO;

  // Температуры — уставка и текущая
  uint8_t tset = bytes[19];
  uint8_t tcur = bytes[20];

  // Флаги
  bool quiet = (bytes[36] & 0b00000100) != 0;
  bool turbo = (bytes[35] & 0b00000010) != 0;
  bool eco   = (bytes[35] & 0b00000100) != 0;
  bool led   = (bytes[37] & 0b10000000) != 0;
  bool ud    = (bytes[35] & 0b10000000) != 0;
  bool lr    = (bytes[35] & 0b01000000) != 0;

  // Публикуем в Climate
  this->current_temperature = float(tcur);
  this->target_temperature  = float(tset);
  this->mode      = new_power ? new_mode : climate::CLIMATE_MODE_OFF;
  this->fan_mode  = new_fan;
  this->swing_mode = (ud || lr) ? climate::CLIMATE_SWING_BOTH : climate::CLIMATE_SWING_OFF;

  // зеркалим во внутреннее состояние
  this->power_ = new_power;
  this->mode_  = new_mode;
  this->fan_   = new_fan;
  this->swing_ud_ = ud;
  this->swing_lr_ = lr;
  this->quiet_ = quiet;
  this->turbo_ = turbo;
  this->eco_   = eco;
  this->led_   = led;
  this->room_temp_   = float(tcur);
  this->target_temp_ = float(tset);

  this->publish_state();
}

}  // namespace ac_hi
}  // namespace esphome
