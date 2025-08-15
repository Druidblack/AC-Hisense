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
  // протокол: 9600 8N1
  this->check_uart_settings(9600, 1, uart::UART_CONFIG_PARITY_NONE, 8);  // :contentReference[oaicite:1]{index=1}

  // дефолтная шапка (переобучится на первом реальном кадре)
  const uint8_t def_hdr[11] = {0x00,0x40,0x29,0x00,0x00,0x01,0x01,0xFE,0x01,0x00,0x00};
  memcpy(header_, def_hdr, sizeof(header_));

  // Опубликуем стартовое состояние
  this->target_temperature = target_temp_;
  this->mode = climate::CLIMATE_MODE_OFF;
  this->fan_mode = climate::CLIMATE_FAN_AUTO;
  this->swing_mode = climate::CLIMATE_SWING_OFF;
  this->publish_state();  // :contentReference[oaicite:2]{index=2}
}

void ACHiClimate::dump_config() {
  ESP_LOGCONFIG(TAG, "AC-Hi (Ballu/Hisense) Climate");
  ESP_LOGCONFIG(TAG, "  Update interval: %u ms", (unsigned)update_interval_ms_);
}

climate::ClimateTraits ACHiClimate::traits() {
  climate::ClimateTraits t;
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
      climate::CLIMATE_SWING_BOTH, // протокол даёт H/V — сводим к BOTH для стандартной карточки
  });
  t.set_visual_min_temperature(16.0f);
  t.set_visual_max_temperature(30.0f);
  t.set_visual_temperature_step(1.0f);
  return t;
}

// ======================= Loop =======================

void ACHiClimate::loop() {
  // RX: читаем всё доступное и накапливаем до хвоста 0xF4 0xFB
  while (this->available()) {
    uint8_t b; if (!this->read_byte(&b)) break;
    rx_buf_.push_back(b);
  }

  // разбираем полные кадры
  const uint8_t tail[2] = {0xF4, 0xFB};
  for (;;) {
    // ищем конец кадра (хвост)
    auto it_tail = std::search(rx_buf_.begin() + 2, rx_buf_.end(), std::begin(tail), std::end(tail));
    if (it_tail == rx_buf_.end()) break; // нет конца — ждём

    // собрать кадр от начала буфера до хвоста (включая хвост)
    std::vector<uint8_t> frame(rx_buf_.begin(), it_tail + 2);
    learn_header_(frame);
    this->log_hex_dump_("RX frame", frame);
    if (frame.size() >= 20) handle_status_(frame);

    // удалить обработанное
    rx_buf_.erase(rx_buf_.begin(), it_tail + 2);
  }

  // Периодический статус-опрос
  uint32_t now = millis();
  if (now - last_poll_ >= update_interval_ms_) {
    last_poll_ = now;
    this->send_status_request_short_();
  }
  // Если давно не слышим ответов — редкий длинный «чистый» опрос
  if ((now - last_rx_ms_ > 10000) && (now - last_long_status_ms_ > 30000)) {
    last_long_status_ms_ = now;
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
      mode_bin_ = 0x00;        // clear mode when powering off
      this->mode = climate::CLIMATE_MODE_OFF;
      this->power_ = false;
    } else {
      power_bin_ = 0b00001100; // bit2 + bit3
      this->mode = m;
      this->power_ = true;

      // индексы: 0=FAN_ONLY,1=HEAT,2=COOL,3=DRY,4=AUTO
      uint8_t idx = 4; // auto
      if (m == climate::CLIMATE_MODE_HEAT) idx = 1;
      else if (m == climate::CLIMATE_MODE_COOL) idx = 2;
      else if (m == climate::CLIMATE_MODE_DRY)  idx = 3;
      else if (m == climate::CLIMATE_MODE_FAN_ONLY) idx = 0;

      // код в старшем полубайте = idx<<4
      mode_bin_ = uint8_t(idx << 4);
    }
    need_write = true;
  }

  if (call.get_target_temperature().has_value()) {
    float t = *call.get_target_temperature();
    if (t < 16.0f) t = 16.0f;
    if (t > 30.0f) t = 30.0f;
    this->target_temperature = t;
    target_temp_ = t;
    temp_byte_ = (uint8_t(t) << 1) | 0x01; // протокол ожидает °C*2 | 1
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
    this->swing_mode = s;
    need_write = true;
  }

  if (need_write) {
    this->send_write_frame_();
    // анти-откат: блокируем обновление режима/питания из входящих статусов на 2.5с
    suppress_until_ms_ = millis() + 2500;
  }

  this->publish_state();  // уведомляем HA об обновлении состояния устройства :contentReference[oaicite:3]{index=3}
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

  // [18] составной байт: питание + режим
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

  // [33] turbo + eco (можно расширить при необходимости)
  out_[33] = uint8_t(turbo_bin_ + eco_bin_);

  // [35] quiet
  out_[35] = quiet_bin_;
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

void ACHiClimate::send_status_request_short_() {
  // КОРОТКИЙ запрос статуса (cmd 0x66) — фиксированный кадр (однобайтный CRC перед F4 FB)
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
  this->build_base_long_frame_();
  out_[13] = 0x66;
  this->compute_crc_(out_);
  this->write_array(out_.data(), out_.size());
  this->flush();
  ESP_LOGD(TAG, "TX status req (0x66 long, clean) hdr:%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
           out_[2],out_[3],out_[4],out_[5],out_[6],out_[7],out_[8],out_[9],out_[10],out_[11],out_[12]);
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
}

void ACHiClimate::handle_status_(const std::vector<uint8_t> &bytes) {
  if (bytes.size() < 20) return;

  // Команда (ACK=101, STATUS=102)
  uint8_t cmd = (bytes.size() > 13) ? bytes[13] : 0x00;
  ESP_LOGD(TAG, "RX summary: cmd=%u len=%u", cmd, (unsigned)bytes.size());

  if (cmd == 101) {
    // ACK после записи — запросим ещё и длинный статус.
    this->set_timeout("after_ack_long_status", 120, [this]() { this->send_status_request_long_clean_(); }); // :contentReference[oaicite:4]{index=4}
    this->last_rx_ms_ = millis();
    return;
  }
  if (cmd != 102) {
    // неизвестный тип — пропустим, но отметим активность
    this->last_rx_ms_ = millis();
    return;
  }

  // ====== Поля из 0x66-статусов ======
  // [19]=Tset (°C), [20]=Tcur (°C)
  if (bytes.size() > 20) {
    uint8_t tset = bytes[19];
    uint8_t tcur = bytes[20];
    this->target_temperature = tset;
    this->current_temperature = tcur;
    if (tset_s_)  tset_s_->publish_state(tset);
    if (tcur_s_)  tcur_s_->publish_state(tcur);
  }

  // [16]=скорость вентилятора (статусный код)
  if (bytes.size() > 16) {
    uint8_t wc = bytes[16];
    // сопоставим 1/12/14/16 → AUTO/LOW/MED/HIGH
    climate::ClimateFanMode f = climate::CLIMATE_FAN_AUTO;
    if (wc >= 16) f = climate::CLIMATE_FAN_HIGH;
    else if (wc >= 14) f = climate::CLIMATE_FAN_MEDIUM;
    else if (wc >= 12) f = climate::CLIMATE_FAN_LOW;
    this->fan_mode = f;
  }

  // [18] режим/питание — с учётом подавления анти-отката
  bool allow_mode_update = (int32_t)(millis() - suppress_until_ms_) >= 0;
  if (bytes.size() > 18 && allow_mode_update) {
    uint8_t b = bytes[18];
    bool rx_power = (b & 0x08) != 0;
    if (!rx_power) {
      this->mode = climate::CLIMATE_MODE_OFF;
      this->power_ = false;
    } else {
      this->power_ = true;
      uint8_t m = (b >> 4) & 0x07;
      if (m == 0)
        this->mode = climate::CLIMATE_MODE_FAN_ONLY;
      else if (m == 1)
        this->mode = climate::CLIMATE_MODE_HEAT;
      else if (m == 2)
        this->mode = climate::CLIMATE_MODE_COOL;
      else if (m == 3)
        this->mode = climate::CLIMATE_MODE_DRY;
      else
        this->mode = climate::CLIMATE_MODE_AUTO;
    }
  }

  // Свинг: по практике legacy читали биты в [35]; сведём к BOTH/ OFF для стандартной карточки
  if (bytes.size() > 35) {
    bool ud = (bytes[35] & 0x80) != 0;
    bool lr = (bytes[35] & 0x40) != 0;
    this->swing_mode = (ud || lr) ? climate::CLIMATE_SWING_BOTH : climate::CLIMATE_SWING_OFF;
  }

  // Доп-датчики — места под outdoor/pipe/частоту компрессора (если известны байты, можно раскомментировать)
  if (tout_s_)   {/* tout_s_->publish_state(...); */}
  if (tpipe_s_)  {/* tpipe_s_->publish_state(...); */}
  if (compfreq_s_) {/* compfreq_s_->publish_state(...); */}

  this->publish_state();
  this->last_rx_ms_ = millis();
}

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

// ======================= Write =======================

void ACHiClimate::send_write_frame_() {
  this->build_base_long_frame_();
  out_[13] = 0x65;     // запись
  this->apply_intent_to_frame_();
  this->compute_crc_(out_);
  this->write_array(out_.data(), out_.size());
  this->flush();
  this->log_hex_dump_("TX write(0x65)", out_);

  // Пост-опрос: короткий сразу + длинный с небольшой задержкой
  this->set_timeout("post_write_status_short", 120, [this]() { this->send_status_request_short_(); });  // :contentReference[oaicite:5]{index=5}
  this->set_timeout("post_write_status_long",  280, [this]() { this->send_status_request_long_clean_(); });
}

}  // namespace ac_hi
}  // namespace esphome
