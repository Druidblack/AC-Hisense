// SPDX-License-Identifier: MIT
#include "ac_hi_climate.h"
#include "esphome/core/log.h"
#include <algorithm>

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
    rx_buf_.push_back(b);
  }

  if (!raw_rx.empty()) {
    this->log_hex_dump_("RX raw", raw_rx);
  }

  // Parse any complete frames accumulated so far
  this->process_rx_buffer_();

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

  // После записи запросим статус: короткий сразу + длинный (чуть позже) с обученной шапкой
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

// ======================= RX processing =======================

void ACHiClimate::process_rx_buffer_() {
  // Защитный лимит, чтобы буфер не рос бесконечно
  const size_t MAX_BUF = 8192;
  if (rx_buf_.size() > MAX_BUF) {
    // обрежем до последних 512 байт
    rx_buf_.erase(rx_buf_.begin(), rx_buf_.end() - 512);
  }

  // Ищем полноценные кадры F4 F5 ... F4 FB, устойчиво к разрывам между итерациями
  while (true) {
    // старт кадра
    auto it_start = std::search(rx_buf_.begin(), rx_buf_.end(), std::begin("\xF4\xF5"), std::end("\xF4\xF5") - 1);
    if (it_start == rx_buf_.end()) {
      // нет начала — можно немного подчистить мусор, оставим последние 1 байт (вдруг это 0xF4)
      if (rx_buf_.size() > 1) rx_buf_.erase(rx_buf_.begin(), rx_buf_.end() - 1);
      break;
    }
    // удалим мусор до старта
    if (it_start != rx_buf_.begin()) rx_buf_.erase(rx_buf_.begin(), it_start);

    // конец кадра
    static const uint8_t tail[] = {0xF4, 0xFB};
    auto it_end = std::search(rx_buf_.begin() + 2, rx_buf_.end(), std::begin(tail), std::end(tail));
    if (it_end == rx_buf_.end()) {
      // хвоста ещё нет — ждём догрузки
      break;
    }

    // полный кадр: [begin, it_end+2)
    std::vector<uint8_t> frame(rx_buf_.begin(), it_end + 2);

    // обучимся шапке и залогируем
    learn_header_(frame);
    this->log_hex_dump_("RX frame", frame);

    // разбор
    if (frame.size() >= 20) handle_status_(frame);

    // удалим обработанную часть из буфера
    rx_buf_.erase(rx_buf_.begin(), it_end + 2);

    // цикл попробует вытащить следующий кадр, если он уже накопился
  }
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
    // ACK после записи — шапку уже выучили, состояние не публикуем.
    // Подстрахуемся и дёрнем длинный стат-опрос (если ещё не запланирован)
    this->set_timeout("after_ack_long_status", 120, [this]() { this->send_status_request_long_clean_(); });
    this->last_rx_ms_ = millis();
    return;
  }
  if (cmd != 102) {
    // неизвестный тип — пропустим, но отметим активность
    this->last_rx_ms_ = millis();
    return;
  }

  // ====== Поля из ваших 0x66-статусов ======
  // По логам: байты [19] и [20] стабильно выглядят как Tset(°C) и Tcur(°C) соответственно.
  if (bytes.size() > 20) {
    uint8_t tset = bytes[19];
    uint8_t tcur = bytes[20];
    if (tset >= 8 && tset <= 45) this->target_temperature  = float(tset);
    if (tcur >= 8 && tcur <= 45) this->current_temperature = float(tcur);
  }

  // Вентилятор: встречаются коды 0x01/0x12/0x14/0x16 — распознаём только их, иначе оставляем без изменений.
  if (bytes.size() > 16) {
    uint8_t w = bytes[16];
    if (w == 0x01)        this->fan_mode = climate::CLIMATE_FAN_AUTO;
    else if (w == 0x12)   this->fan_mode = climate::CLIMATE_FAN_LOW;
    else if (w == 0x14)   this->fan_mode = climate::CLIMATE_FAN_MEDIUM;
    else if (w == 0x16)   this->fan_mode = climate::CLIMATE_FAN_HIGH;
  }

  // Режим/питание в 0x66 на вашей модели кодируются не так, как в «записи».
  // Чтобы НЕ ломать отображение в HA, здесь их не трогаем, пока нет 100% карты полей.
  // (Режим меняется верно через control() и ACK, а статусное обновление температур уже работает.)

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
