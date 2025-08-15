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

  // Стартовая уставка для доступности ползунка до первого статуса
  this->target_temperature = 24.0f;

  // Базовая заготовка длинного кадра (для записи и длинного статуса)
  this->build_base_long_frame_();

  // Первый опрос статуса (короткий, «тихий»)
  this->set_timeout("init_status", 500, [this]() { this->send_status_request_short_(); });
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
  // RX collect — накопим сырую порцию, сразу залогируем (чтобы понять, молчит ли линия вообще)
  std::vector<uint8_t> raw_rx;
  raw_rx.reserve(256);

  while (this->available()) {
    uint8_t b = this->read();
    raw_rx.push_back(b);
    this->rb_push_(b);
  }

  if (!raw_rx.empty()) {
    this->log_hex_dump_("RX raw", raw_rx);
  }

  // Parse frames
  while (this->parse_next_frame_()) {}

  const uint32_t now = millis();

  // Периодический короткий опрос
  if (now - this->last_poll_ >= this->update_interval_ms_) {
    this->last_poll_ = now;
    this->send_status_request_short_();
  }

  // Если давно не слышим ответов — редкий длинный «чистый» опрос (без пищалки-спама)
  if ((now - this->last_rx_ms_ > 10000) && (now - this->last_long_status_ms_ > 30000)) {
    this->last_long_status_ms_ = now;
    this->send_status_request_long_clean_();
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

      // индексы: 0=FAN_ONLY,1=HEAT,2=COOL,3=DRY,4=AUTO → odd-nibble при записи
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
    this->target_temperature = t;
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

  // заголовок [2..12] — дефолтный либо обученный
  for (int i = 0; i < 11; i++) out_[2 + i] = header_[i];

  // [13] — команда (0x65 write / 0x66 status)
  // [23] — в рабочем yaml был 0x04 (оставляем)
  out_[23] = 0x04;
  out_[48] = 0xF4; out_[49] = 0xFB;
}

void ACHiClimate::apply_intent_to_frame_() {
  // [16] скорость вентилятора (для записи требуется +1 к статусному коду)
  out_[16] = uint8_t(wind_code_ + 1);

  // [18] составной байт: питание + режим (odd-nibble схема)
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

  // [33] turbo + eco (держим off по умолчанию)
  out_[33] = uint8_t(turbo_bin_ + eco_bin_);

  // [35] quiet (держим off)
  out_[35] = quiet_bin_;
}

void ACHiClimate::send_status_request_short_() {
  // КОРОТКИЙ запрос статуса (cmd 0x66) — бесшумный и стабильный.
  // ВНИМАНИЕ: у этого формата однобайтный CRC (последний перед F4 FB). Поэтому оставляем фиксированный кадр.
  // F4 F5 00 40 0C 00 00 01 01 FE 01 00 00 66 00 00 00 01 B3 F4 FB
  const uint8_t req[] = {
    0xF4,0xF5,0x00,0x40,0x0C,0x00,0x00,0x01,0x01,0xFE,0x01,0x00,0x00,0x66,0x00,0x00,0x00,0x01,0xB3,0xF4,0xFB
  };
  this->write_array(req, sizeof(req));
  this->flush();
  ESP_LOGD(TAG, "TX status req (0x66 short)");
}

void ACHiClimate::send_status_request_long_clean_() {
  // ДЛИННЫЙ «чистый» запрос статуса 0x29/0x66 — без заполнения управляющих полей.
  // Используем ОБУЧЕННУЮ шапку [2..12], чтобы адреса соответствовали блоку.
  this->build_base_long_frame_();
  out_[13] = 0x66;
  // [16],[18],[19],[32] оставляем по нулям, только CRC/хвост
  this->compute_crc_(out_);
  this->write_array(out_.data(), out_.size());
  this->flush();
  ESP_LOGD(TAG, "TX status req (0x66 long, clean) hdr:%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
           out_[2],out_[3],out_[4],out_[5],out_[6],out_[7],out_[8],out_[9],out_[10],out_[11],out_[12]);
}

void ACHiClimate::send_write_frame_() {
  this->build_base_long_frame_();
  out_[13] = 0x65;        // команда записи
  this->apply_intent_to_frame_();
  this->compute_crc_(out_);

  this->write_array(out_.data(), out_.size());
  this->flush();

  // Дамп для диагностики
  this->log_hex_dump_("TX write(0x65)", out_);

  // После записи запросим статус: короткий сразу + длинный (через 150 мс) с обученной шапкой
  this->set_timeout("post_write_status_short", 120, [this]() { this->send_status_request_short_(); });
  this->set_timeout("post_write_status_long",  280, [this]() { this->send_status_request_long_clean_(); });
}

void ACHiClimate::compute_crc_(std::vector<uint8_t> &buf) {
  // ДЛИННЫЙ формат: сумма по [2..len-5] → [len-4],[len-3], затем 0xF4 0xFB
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
      // если второго байта ещё нет — вернём F4 в буфер и ждём
      if (!rb_pop_(b2)) { rb_tail_ = (rb_tail_ + RB_SIZE - 1) % RB_SIZE; return false; }
      if (b2 == 0xF5) {
        std::vector<uint8_t> frame;
        frame.reserve(160);
        frame.push_back(0xF4); frame.push_back(0xF5);
        int guard = 0;
        while (rb_pop_(b)) {
          frame.push_back(b);
          if (frame.size() >= 4 && frame[frame.size()-2] == 0xF4 && frame.back() == 0xFB) {
            // обучимся шапке с ЛЮБОГО кадра
            learn_header_(frame);

            // ЛОГ RX — дамп всего кадра
            this->log_hex_dump_("RX frame", frame);

            // обработка статуса
            if (frame.size() >= 20) handle_status_(frame);
            return true;
          }
          if (++guard > 4096) break; // защита от «каши»
        }
        // если мы тут — хвост не увидели, прерываем
        return false;
      } else {
        // откат b2 назад (мы его читали зря), F4 уже «съели», но это был не кадр
        rb_tail_ = (rb_tail_ + RB_SIZE - 1) % RB_SIZE;
      }
    }
    if (++scanned > RB_SIZE) break;
  }
  return false;
}

void ACHiClimate::learn_header_(const std::vector<uint8_t> &bytes) {
  if (bytes.size() <= 13) return;
  for (int i = 0; i < 11; i++) header_[i] = bytes[2 + i];
  if (!header_learned_) {
    header_learned_ = true;
    ESP_LOGI(TAG, "Learned header [2..12]: %02X %02X %02X %02X %02X  %02X %02X %02X %02X %02X %02X",
             header_[0],header_[1],header_[2],header_[3],header_[4],
             header_[5],header_[6],header_[7],header_[8],header_[9],header_[10]);
  }
  this->last_rx_ms_ = millis();
}

void ACHiClimate::handle_status_(const std::vector<uint8_t> &bytes) {
  if (bytes.size() < 20) return;

  // Команда (ACK=101, STATUS=102)
  uint8_t cmd = (bytes.size() > 13) ? bytes[13] : 0x00;
  ESP_LOGD(TAG, "RX summary: cmd=%u len=%u", cmd, (unsigned)bytes.size());

  if (cmd == 101) {
    // ACK после записи — не публикуем состояние, но шапку мы уже выучили.
    // Дополнительно триггерим длинный опрос с обученной шапкой (на случай, если очереди таймеров нет)
    this->set_timeout("after_ack_long_status", 120, [this]() { this->send_status_request_long_clean_(); });
    this->last_rx_ms_ = millis();
    return;
  }
  if (cmd != 102) {
    // неизвестный тип — пропустим, но отметим активность
    this->last_rx_ms_ = millis();
    return;
  }

  // Power: bit3 в [18]
  if (bytes.size() > 18) {
    bool new_power = (bytes[18] & 0b00001000) != 0;
    this->power_ = new_power;
  }

  // Mode: старший полубайт [18] — odd-nibble (1,3,5,7,9)
  if (bytes.size() > 18) {
    uint8_t nibble = (bytes[18] >> 4) & 0x0F;
    climate::ClimateMode new_mode = climate::CLIMATE_MODE_AUTO;
    switch (nibble) {
      case 1: new_mode = climate::CLIMATE_MODE_FAN_ONLY; break;
      case 3: new_mode = climate::CLIMATE_MODE_HEAT;     break;
      case 5: new_mode = climate::CLIMATE_MODE_COOL;     break;
      case 7: new_mode = climate::CLIMATE_MODE_DRY;      break;
      case 9: new_mode = climate::CLIMATE_MODE_AUTO;     break;
      default: /* no change */ break;
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

  // Температуры — [19] уставка, [20] текущая (в °C). Защита от явного мусора.
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

  this->publish_state();
  this->last_rx_ms_ = millis();
}

// ======================= Logging helpers =======================

void ACHiClimate::log_hex_dump_(const char *prefix, const std::vector<uint8_t> &data) {
  std::string dump;
  dump.reserve(data.size() * 3 + data.size() / 16 + 16);
  for (size_t i = 0; i < data.size(); i++) {
    char b[4];
    snprintf(b, sizeof(b), "%02X ", data[i]);
    dump += b;
    if ((i + 1) % 16 == 0) dump += "\n";
  }
  ESP_LOGD(TAG, "%s:\n%s", prefix, dump.c_str());
}

}  // namespace ac_hi
}  // namespace esphome
