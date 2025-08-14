// SPDX-License-Identifier: MIT
#include "ac_hi_climate.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ac_hi {

static const char *const TAG = "ac_hi.climate";

// ======================= Setup & Config =======================

void ACHiClimate::setup() {
  ESP_LOGI(TAG, "Init climate over RS-485");
  // Большинство Hisense/Ballu работают 9600 8N1 (RS-485 полудуплекс через автодирекшн платы TXD/RXD)
  this->check_uart_settings(9600, 1, uart::UART_CONFIG_PARITY_NONE, 8);

  // Сформируем базовый длинный кадр (тип 0x29)
  this->build_base_long_frame_();

  // Стартовый опрос
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
      power_bin_ = 0b00000100; // keep bit2, clear bit3
      this->mode = climate::CLIMATE_MODE_OFF;
      this->power_ = false;
    } else {
      power_bin_ = 0b00001100; // bit2 + bit3
      this->mode = m;
      this->power_ = true;

      uint8_t idx = 4; // auto
      if (m == climate::CLIMATE_MODE_HEAT) idx = 1;
      else if (m == climate::CLIMATE_MODE_COOL) idx = 2;
      else if (m == climate::CLIMATE_MODE_DRY)  idx = 3;
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
    // map to wind code used by протокол статуса
    if (f == climate::CLIMATE_FAN_AUTO)        wind_code_ = 1;
    else if (f == climate::CLIMATE_FAN_LOW)    wind_code_ = 12;
    else if (f == climate::CLIMATE_FAN_MEDIUM) wind_code_ = 14;
    else if (f == climate::CLIMATE_FAN_HIGH)   wind_code_ = 16;
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

void ACHiClimate::build_base_long_frame_() {
  // Базовый “длинный” пакет 50 байт (тип 0x29) как в проверенных примерах Hisense/AirCon
  out_.assign(50, 0x00);
  out_[0]  = 0xF4;
  out_[1]  = 0xF5;
  out_[2]  = 0x00;   // адрес/флаг (оставим 0x00; при необходимости подменится устройством)
  out_[3]  = 0x40;
  out_[4]  = 0x29;   // длинный пакет
  out_[5]  = 0x00;
  out_[6]  = 0x00;
  out_[7]  = 0x01;
  out_[8]  = 0x01;
  out_[9]  = 0xFE;
  out_[10] = 0x01;   // важно: в реальных кадрах = 0x01
  out_[11] = 0x00;
  out_[12] = 0x00;
  // out_[13] — команда: 0x65 (write) / 0x66 (status)
  // остальное заполним нулями; поля ниже мы изменяем в apply_intent_to_frame_()
  out_[48] = 0xF4;
  out_[49] = 0xFB;
}

void ACHiClimate::apply_intent_to_frame_() {
  // Команда уже выставляется вызывающей стороной (write/status).
  // Ниже — поля по реверсу (см. комментарии в исходниках).
  // [16] скорость вентилятора (для записи требуется +1 к коду статуса)
  out_[16] = uint8_t(wind_code_ + 1);

  // [18] составной байт: питание + режим
  out_[18] = uint8_t(power_bin_ + mode_bin_);
  if ((power_bin_ & 0b00001000) == 0) {
    out_[18] = uint8_t(out_[18] & (~(1U<<3)));
  }

  // [19] уставка температуры (°C *2 | 1)
  out_[19] = temp_byte_;

  // [32] качание жалюзи (UD + LR)
  uint8_t updown    = swing_ud_ ? 0b00110000 : 0b00010000;
  uint8_t leftright = swing_lr_ ? 0b00001100 : 0b00000100;
  out_[32] = uint8_t(updown + leftright);

  // Прочие флаги (turbo/eco/quiet/LED) не трогаем — оставляем 0x00 по умолчанию.
  // Пересчёт CRC ниже.
}

void ACHiClimate::send_status_request_() {
  // Длинный запрос статуса: тот же базовый 0x29, но команда 0x66
  this->build_base_long_frame_();
  out_[13] = 0x66;
  // Для статуса не важно, что в полях — но выставим текущие значения, некоторые контроллеры это учитывают
  this->apply_intent_to_frame_();
  this->compute_crc_(out_);

  this->write_array(out_.data(), out_.size());
  this->flush();
  ESP_LOGVV(TAG, "TX status req(0x66, 0x29)");
}

void ACHiClimate::send_write_frame_() {
  this->build_base_long_frame_();
  out_[13] = 0x65;        // команда записи
  this->apply_intent_to_frame_();
  this->compute_crc_(out_);

  // Отправка
  this->write_array(out_.data(), out_.size());
  this->flush();

  // Дамп для диагностики
  std::string dump;
  for (size_t i = 0; i < out_.size(); i++) {
    char b[4];
    snprintf(b, sizeof(b), "%02X ", out_[i]);
    dump += b;
    if ((i + 1) % 16 == 0) dump += "\n";
  }
  ESP_LOGD(TAG, "TX write(0x65):\n%s", dump.c_str());

  // После записи сразу попросим статус, чтобы HA не “откатывал” значения
  this->set_timeout("post_write_status", 150, [this]() { this->send_status_request_(); });
}

void ACHiClimate::compute_crc_(std::vector<uint8_t> &buf) {
  // Контрольная сумма: сумма по [2..len-5] в 16-бит (big-endian), затем 0xF4 0xFB в хвосте
  if (buf.size() < 8) return;
  const size_t n = buf.size();
  int csum = 0;
  for (size_t i = 2; i < n - 4; i++) csum += buf[i];
  uint8_t cr1 = (csum & 0xFF00) >> 8;
  uint8_t cr2 = (csum & 0x00FF);
  buf[n - 4] = cr1;
  buf[n - 3] = cr2;
  buf[n - 2] = 0xF4;
  buf[n - 1] = 0xFB;
}

// Frames start with F4 F5 and end with F4 FB
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
          if (++guard > 512) break;
        }
        return false;
      } else {
        // put back b2
        rb_tail_ = (rb_tail_ + RB_SIZE - 1) % RB_SIZE;
      }
    }
    if (++cnt > RB_SIZE) break;
  }
  return false;
}

void ACHiClimate::handle_status_(const std::vector<uint8_t> &bytes) {
  if (bytes.size() < 20) return;

  last_status_ = bytes;

  // Интересен cmd=102 (0x66)
  if (bytes.size() > 13 && bytes[13] != 102) {
    // 101 — “unlock/ack” после записи; игнорируем для состояния
    return;
  }

  // Power (bit3 в [18])
  if (bytes.size() > 18) {
    bool new_power = (bytes[18] & 0b00001000) != 0;
    this->power_ = new_power;
  }

  // Mode: верхняя тетрада [18]
  if (bytes.size() > 18) {
    uint8_t mode_raw = (bytes[18] >> 4) & 0x0F;
    climate::ClimateMode new_mode = climate::CLIMATE_MODE_AUTO;
    switch (mode_raw) {
      case 0: new_mode = climate::CLIMATE_MODE_FAN_ONLY; break;
      case 1: new_mode = climate::CLIMATE_MODE_HEAT;     break;
      case 2: new_mode = climate::CLIMATE_MODE_COOL;     break;
      case 3: new_mode = climate::CLIMATE_MODE_DRY;      break;
      default: new_mode = climate::CLIMATE_MODE_AUTO;    break;
    }
    this->mode = this->power_ ? new_mode : climate::CLIMATE_MODE_OFF;
  }

  // Fan — [16] (в статусе коды: 1/12/14/16)
  if (bytes.size() > 16) {
    uint8_t wind_raw = bytes[16];
    climate::ClimateFanMode new_fan = climate::CLIMATE_FAN_AUTO;
    if (wind_raw == 12) new_fan = climate::CLIMATE_FAN_LOW;
    else if (wind_raw == 14) new_fan = climate::CLIMATE_FAN_MEDIUM;
    else if (wind_raw == 16) new_fan = climate::CLIMATE_FAN_HIGH;
    else if (wind_raw == 1)  new_fan = climate::CLIMATE_FAN_AUTO;
    this->fan_mode = new_fan;
  }

  // Температуры — [19] уставка, [20] текущая
  if (bytes.size() > 20) {
    this->target_temperature  = float(bytes[19]);
    this->current_temperature = float(bytes[20]);
  }

  // Качание — биты в [35]: bit7 UD, bit6 LR
  if (bytes.size() > 35) {
    bool ud = (bytes[35] & 0b10000000) != 0;
    bool lr = (bytes[35] & 0b01000000) != 0;
    this->swing_mode = (ud || lr) ? climate::CLIMATE_SWING_BOTH : climate::CLIMATE_SWING_OFF;
  }

  this->publish_state();
}

}  // namespace ac_hi
}  // namespace esphome
