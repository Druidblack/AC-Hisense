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

  // Базовая уставка, чтобы ползунок температуры был доступен сразу
  this->target_temperature = 24.0f;

  // Сформируем базовый длинный кадр (тип 0x29)
  this->build_base_long_frame_();

  // Первый опрос статуса (длинный, «чистый»)
  this->set_timeout("init_status", 500, [this]() { this->send_status_request_(); });
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

  // Периодический опрос статуса (длинный, «чистый», без звука)
  const uint32_t now = millis();
  if (now - this->last_poll_ >= this->update_interval_ms_) {
    this->last_poll_ = now;
    this->send_status_request_();
  }

  // если давно не получали RX — чуть ускорим следующий опрос
  if (now - this->last_rx_ms_ > 10000 && now - this->last_poll_ > 1200) {
    this->send_status_request_();
    this->last_poll_ = now;
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

      // индексы: 0=FAN_ONLY,1=HEAT,2=COOL,3=DRY,4=AUTO → в байте статуса видим нечётные nibble
      uint8_t idx = 4; // auto
      if (m == climate::CLIMATE_MODE_HEAT) idx = 1;
      else if (m == climate::CLIMATE_MODE_COOL) idx = 2;
      else if (m == climate::CLIMATE_MODE_DRY)  idx = 3;
      else if (m == climate::CLIMATE_MODE_FAN_ONLY) idx = 0;

      // код в старшем полубайте = ((idx<<1)|1)
      mode_bin_ = uint8_t((((idx << 1) | 0x01) << 4));
    }
    need_write = true;
  }

  if (call.get_target_temperature().has_value()) {
    float t = *call.get_target_temperature();
    if (t < 16.0f) t = 16.0f;
    if (t > 30.0f) t = 30.0f;
    this->target_temperature = t;             // чтобы HA сразу показывал значение
    this->target_temp_ = t;
    temp_byte_ = (uint8_t(t) << 1) | 0x01;
    need_write = true;
  }

  if (call.get_fan_mode().has_value()) {
    auto f = *call.get_fan_mode();
    this->fan_mode = f;
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
  // Длинный “базовый” пакет 50 байт (тип 0x29)
  out_.assign(50, 0x00);
  out_[0]  = 0xF4; out_[1]  = 0xF5;

  // заголовок [2..12] — из обученного, либо дефолтного массива
  for (int i = 0; i < 11; i++) out_[2 + i] = header_[i];

  // [13] — команда (0x65 write / 0x66 status)
  out_[48] = 0xF4; out_[49] = 0xFB;
}

void ACHiClimate::apply_intent_to_frame_() {
  // [16] скорость вентилятора (для записи требуется +1 к статусному коду)
  out_[16] = uint8_t(wind_code_ + 1);

  // [18] составной байт: питание + режим (odd nibble схема)
  out_[18] = uint8_t(power_bin_ + mode_bin_);
  if ((power_bin_ & 0b00001000) == 0) {
    out_[18] = uint8_t(out_[18] & (~(1U<<3)));
  }

  // [19] уставка температуры (°C*2 | 1)
  out_[19] = temp_byte_;

  // [32] качание: UD + LR
  uint8_t updown    = swing_ud_ ? 0b00110000 : 0b00010000;
  uint8_t leftright = swing_lr_ ? 0b00001100 : 0b00000100;
  out_[32] = uint8_t(updown + leftright);
}

void ACHiClimate::send_status_request_() {
  // ДЛИННЫЙ запрос статуса (0x29 + cmd 0x66), «чистый»: без проставления полей управления
  // ВАЖНО: используем обученный заголовок (иначе некоторые юниты не отвечают).
  this->build_base_long_frame_();
  out_[13] = 0x66;

  // Не трогаем [16],[18],[19],[32] — оставляем 0x00
  this->compute_crc_(out_);

  this->write_array(out_.data(), out_.size());
  this->flush();
  ESP_LOGVV(TAG, "TX status req (0x66 long, clean) with hdr: %02X %02X %02X ...",
            out_[2], out_[3], out_[4]);
}

void ACHiClimate::send_write_frame_() {
  this->build_base_long_frame_();
  out_[13] = 0x65;        // команда записи
  this->apply_intent_to_frame_();
  this->compute_crc_(out_);

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

  // После записи ожидаем ACK → выучим заголовок → запросим статус
  this->set_timeout("post_write_status", 220, [this]() { this->send_status_request_(); });
}

void ACHiClimate::compute_crc_(std::vector<uint8_t> &buf) {
  // Сумма по [2..len-5] → [len-4],[len-3], затем 0xF4 0xFB
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
  size_t scanned = 0;
  uint8_t b;
  while (rb_pop_(b)) {
    if (b == 0xF4) {
      uint8_t b2;
      if (!rb_pop_(b2)) { rb_tail_ = (rb_tail_ + RB_SIZE - 1) % RB_SIZE; return false; }
      if (b2 == 0xF5) {
        std::vector<uint8_t> frame;
        frame.reserve(96);
        frame.push_back(0xF4); frame.push_back(0xF5);
        int guard = 0;
        while (rb_pop_(b)) {
          frame.push_back(b);
          if (frame.size() >= 4 && frame[frame.size()-2] == 0xF4 && frame.back() == 0xFB) {
            // обучимся заголовку с ЛЮБОГО кадра
            learn_header_(frame);

            // обработка статуса
            if (frame.size() >= 20) handle_status_(frame);
            return true;
          }
          if (++guard > 1024) break; // защита от бесконечного потока без хвоста
        }
        return false;
      } else {
        // откат b2 назад
        rb_tail_ = (rb_tail_ + RB_SIZE - 1) % RB_SIZE;
      }
    }
    if (++scanned > RB_SIZE) break;
  }
  return false;
}

void ACHiClimate::learn_header_(const std::vector<uint8_t> &bytes) {
  if (bytes.size() <= 12) return;
  // запомним [2..12]
  for (int i = 0; i < 11; i++) header_[i] = bytes[2 + i];
  if (!header_learned_) {
    header_learned_ = true;
    ESP_LOGI(TAG, "Learned header: %02X %02X %02X %02X %02X  %02X %02X %02X %02X %02X %02X",
             header_[0],header_[1],header_[2],header_[3],header_[4],
             header_[5],header_[6],header_[7],header_[8],header_[9],header_[10]);
  }
  this->last_rx_ms_ = millis();
}

void ACHiClimate::handle_status_(const std::vector<uint8_t> &bytes) {
  if (bytes.size() < 20) return;

  // Команда (ACK=101, STATUS=102)
  uint8_t cmd = (bytes.size() > 13) ? bytes[13] : 0x00;

  if (cmd == 101) {
    // ACK после записи — ничего не публикуем, но заголовок уже выучен в learn_header_()
    ESP_LOGV(TAG, "RX ACK (101), len=%u", (unsigned)bytes.size());
    return;
  }
  if (cmd != 102) {
    // неизвестный тип — пропустим
    ESP_LOGVV(TAG, "RX non-status cmd=%u, len=%u", cmd, (unsigned)bytes.size());
    return;
  }

  // Power: bit3 в [18]
  if (bytes.size() > 18) {
    bool new_power = (bytes[18] & 0b00001000) != 0;
    this->power_ = new_power;
  }

  // Mode: старший полубайт [18] — НЕЧЁТНЫЕ значения!
  if (bytes.size() > 18) {
    uint8_t nibble = (bytes[18] >> 4) & 0x0F;  // ожидаем 1,3,5,7,9
    climate::ClimateMode new_mode = climate::CLIMATE_MODE_AUTO;
    switch (nibble) {
      case 1: new_mode = climate::CLIMATE_MODE_FAN_ONLY; break;
      case 3: new_mode = climate::CLIMATE_MODE_HEAT;     break;
      case 5: new_mode = climate::CLIMATE_MODE_COOL;     break;
      case 7: new_mode = climate::CLIMATE_MODE_DRY;      break;
      case 9: new_mode = climate::CLIMATE_MODE_AUTO;     break;
      default: /* оставить прежний */ break;
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

  // Температуры — типичный вариант: [19] уставка, [20] текущая (в °C).
  // Если получим явный мусор, не затираем разумные значения.
  if (bytes.size() > 20) {
    uint8_t tset = bytes[19];
    uint8_t tcur = bytes[20];
    if (tset >= 8 && tset <= 45) this->target_temperature  = float(tset);
    if (tcur >= 8 && tcur <= 45) this->current_temperature = float(tcur);
  }

  // Качание — биты в [35]: bit7 UD, bit6 LR (если поле присутствует)
  if (bytes.size() > 35) {
    bool ud = (bytes[35] & 0b10000000) != 0;
    bool lr = (bytes[35] & 0b01000000) != 0;
    this->swing_mode = (ud || lr) ? climate::CLIMATE_SWING_BOTH : climate::CLIMATE_SWING_OFF;
  }

  ESP_LOGV(TAG, "RX status: power=%d mode=%d fan=%d t=%.1f/%.1f",
           int(this->power_), int(this->mode), int(this->fan_mode),
           this->current_temperature, this->target_temperature);

  this->publish_state();
  this->last_rx_ms_ = millis();
}

}  // namespace ac_hi
}  // namespace esphome
