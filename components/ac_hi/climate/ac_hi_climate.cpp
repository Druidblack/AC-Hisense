// SPDX-License-Identifier: MIT
#include "ac_hi_climate.h"
#include "esphome/core/log.h"
#include <algorithm>

namespace esphome {
namespace ac_hi {

static const char *const TAG = "ac_hi.climate";

void ACHiClimate::setup() {
  // проверка UART-конфигурации (9600 8N1)
  this->check_uart_settings(9600, 1, uart::UART_CONFIG_PARITY_NONE, 8);  // api docs: UARTDevice::check_uart_settings
  // стартовое состояние
  this->target_temperature = target_temp_;
  this->mode = climate::CLIMATE_MODE_OFF;
  this->fan_mode = climate::CLIMATE_FAN_AUTO;   // фан пока только авто (безопасно для приёма)
  this->swing_mode = climate::CLIMATE_SWING_OFF;
  this->publish_state(); // уведомить HA
}

void ACHiClimate::dump_config() {
  ESP_LOGCONFIG(TAG, "AC-Hi Climate (Ballu/Hisense) — mirror-write [18],[19] only");
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
  // чтобы не рисковать при записи, оставляем управляемую скорость только AUTO
  t.set_supported_fan_modes({ climate::CLIMATE_FAN_AUTO });
  t.set_supported_swing_modes({
      climate::CLIMATE_SWING_OFF,
      climate::CLIMATE_SWING_BOTH,
  });
  t.set_visual_min_temperature(16.0f);
  t.set_visual_max_temperature(30.0f);
  t.set_visual_temperature_step(1.0f);
  return t;
}

void ACHiClimate::loop() {
  // RX накопление
  while (this->available()) {
    uint8_t b; if (!this->read_byte(&b)) break;
    rx_buf_.push_back(b);
  }

  // парсим по хвосту F4 FB
  const uint8_t tail[2] = {0xF4, 0xFB};
  for (;;) {
    auto it_tail = std::search(rx_buf_.begin() + 2, rx_buf_.end(), std::begin(tail), std::end(tail));
    if (it_tail == rx_buf_.end()) break;
    std::vector<uint8_t> frame(rx_buf_.begin(), it_tail + 2);

    this->log_hex_dump_("RX frame", frame);
    if (frame.size() >= 20) {
      learn_from_status_(frame);
      handle_status_(frame);
    }
    rx_buf_.erase(rx_buf_.begin(), it_tail + 2);
  }

  // опрос
  uint32_t now = millis();
  if (now - last_poll_ >= update_interval_ms_) {
    last_poll_ = now;
    this->send_status_request_short_();
  }
}

void ACHiClimate::control(const climate::ClimateCall &call) {
  bool need_write = false;

  if (call.get_mode().has_value()) {
    auto m = *call.get_mode();
    if (m == climate::CLIMATE_MODE_OFF) {
      power_bin_ = 0x00;      // OFF
      mode_bin_  = 0x00;
      this->mode = climate::CLIMATE_MODE_OFF;
      this->power_ = false;
    } else {
      power_bin_ = 0x08;      // ON
      this->mode = m;
      this->power_ = true;
      uint8_t idx = 4; // AUTO
      if (m == climate::CLIMATE_MODE_HEAT) idx = 1;
      else if (m == climate::CLIMATE_MODE_COOL) idx = 2;
      else if (m == climate::CLIMATE_MODE_DRY)  idx = 3;
      else if (m == climate::CLIMATE_MODE_FAN_ONLY) idx = 0;
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
    temp_byte_ = static_cast<uint8_t>(t); // целые °C
    need_write = true;
  }

  // (на этом этапе мы сознательно НЕ меняем [16], [32..35] — см. send_write_frame_)
  if (need_write) {
    expected_mode_byte_     = uint8_t(mode_bin_ + power_bin_);
    guard_mode_until_match_ = true;
    guard_deadline_ms_      = millis() + 6000;   // ждём подтверждение до 6с
    suppress_until_ms_      = millis() + 2500;

    this->send_write_frame_();
    // чтобы не сдублировать ближайший периодический опрос, чуть сдвинем
    last_poll_ = millis() + 500;  // следующий короткий опрос не раньше чем через 0.5с
  }

  this->publish_state(); // обновить карточку HA
}

void ACHiClimate::send_status_request_short_() {
  // Короткий 0x66 (стандартный)
  static const uint8_t req[] = {
    0xF4,0xF5,0x00,0x40,0x0C,0x00,0x00,0x01,0x01,0xFE,0x01,0x00,0x00,0x66,0x00,0x00,0x00,0x01,0xB3,0xF4,0xFB
  };
  this->write_array(req, sizeof(req));
  this->flush();
  ESP_LOGD(TAG, "TX status req (0x66 short)");
}

// ======= MIRROR-WRITE: меняем ТОЛЬКО [18] и [19] =======
void ACHiClimate::send_write_frame_() {
  std::vector<uint8_t> out;

  if (!last_status_.empty()) {
    out = last_status_;              // копируем статус целиком (та же длина/поля)
  } else {
    // экстренно: заготовка (редкий случай старта без статуса)
    out.assign(50, 0x00);
    out[0] = 0xF4; out[1] = 0xF5;
    for (int i = 0; i < 11; i++) out[2 + i] = header_[i];
    out[48] = 0xF4; out[49] = 0xFB;
  }

  const size_t n = out.size();
  if (n < 24) return;

  // строго сохраняем «родные» служебные байты
  out[13] = 0x65;      // команда = запись
  out[14] = fld14_;    // как в статусе
  out[15] = fld15_;    // адрес/канал
  // [16] не трогаем — как в статусе
  // МЕНЯЕМ ТОЛЬКО:
  out[18] = expected_mode_byte_;  // режим+питание (power=0x08 / 0x00)
  out[19] = temp_byte_;           // уставка (целые °C)
  // [23], [32..35] и прочее — НЕ трогаем

  // CRC: сумма по [2..n-5] → [n-4]=hi, [n-3]=lo
  int csum = 0;
  for (size_t i = 2; i < n - 4; i++) csum += out[i];
  out[n - 4] = (csum >> 8) & 0xFF;
  out[n - 3] = (csum) & 0xFF;
  out[n - 2] = 0xF4;
  out[n - 1] = 0xFB;

  this->write_array(out.data(), out.size());
  this->flush();
  this->log_hex_dump_("TX write(0x65)", out);

  // пост-опрос: короткий статус (через ~180 мс)
  this->set_timeout("post_write_status_short", 180, [this]() { this->send_status_request_short_(); });
}

void ACHiClimate::learn_from_status_(const std::vector<uint8_t> &bytes) {
  if (bytes.size() <= 24) return;
  if (bytes[13] != 0x66) return; // только валидный статус

  // шапка [2..12]
  for (int i = 0; i < 11; i++) header_[i] = bytes[2 + i];
  fld14_ = bytes[14];
  fld15_ = bytes[15];
  fld23_ = bytes[23];

  if (!header_learned_) {
    header_learned_ = true;
    ESP_LOGI(TAG, "Learned header [2..12]: %02X %02X %02X %02X %02X  %02X %02X %02X %02X %02X %02X; [14]=%02X [15]=%02X [23]=%02X",
             header_[0],header_[1],header_[2],header_[3],header_[4],
             header_[5],header_[6],header_[7],header_[8],header_[9],header_[10],
             fld14_, fld15_, fld23_);
  }
  last_status_ = bytes; // сохранить для mirror-write
}

void ACHiClimate::handle_status_(const std::vector<uint8_t> &bytes) {
  if (bytes.size() < 20) return;
  uint8_t cmd = (bytes.size() > 13) ? bytes[13] : 0x00;
  if (cmd != 102) { this->last_rx_ms_ = millis(); return; }

  // [19]=Tset (°C), [20]=Tcur (°C)
  if (bytes.size() > 20) {
    uint8_t tset = bytes[19];
    uint8_t tcur = bytes[20];
    this->target_temperature = tset;
    this->current_temperature = tcur;
    if (tset_s_)  tset_s_->publish_state(tset);
    if (tcur_s_)  tcur_s_->publish_state(tcur);
  }

  // Режим/питание в [18], питание = бит 0x08
  bool allow_mode_update = (int32_t)(millis() - suppress_until_ms_) >= 0;

  if (bytes.size() > 18) {
    uint8_t b = bytes[18];

    if (guard_mode_until_match_) {
      bool timed_out = (int32_t)(millis() - guard_deadline_ms_) >= 0;
      if (b == expected_mode_byte_) {
        guard_mode_until_match_ = false;
        allow_mode_update = true;
        ESP_LOGD(TAG, "Guard matched: [18]=%02X", b);
      } else if (!timed_out) {
        allow_mode_update = false;
        ESP_LOGD(TAG, "Guard active: ignore mode/power [18]=%02X, expect %02X", b, expected_mode_byte_);
      } else {
        guard_mode_until_match_ = false;
        ESP_LOGW(TAG, "Guard timeout: accepting [18]=%02X", b);
      }
    }

    if (allow_mode_update) {
      bool rx_power = (b & 0x08) != 0;
      if (!rx_power) {
        this->mode = climate::CLIMATE_MODE_OFF;
        this->power_ = false;
      } else {
        this->power_ = true;
        uint8_t m = (b >> 4) & 0x07;
        if (m == 0)      this->mode = climate::CLIMATE_MODE_FAN_ONLY;
        else if (m == 1) this->mode = climate::CLIMATE_MODE_HEAT;
        else if (m == 2) this->mode = climate::CLIMATE_MODE_COOL;
        else if (m == 3) this->mode = climate::CLIMATE_MODE_DRY;
        else             this->mode = climate::CLIMATE_MODE_AUTO;
      }
    }
  }

  this->publish_state();
  this->last_rx_ms_ = millis();
}

void ACHiClimate::compute_crc_(std::vector<uint8_t> &buf) {
  if (buf.size() < 8) return;
  const size_t n = buf.size();
  int csum = 0;
  for (size_t i = 2; i < n - 4; i++) csum += buf[i];
  buf[n - 4] = (csum >> 8) & 0xFF;
  buf[n - 3] = (csum) & 0xFF;
  buf[n - 2] = 0xF4;
  buf[n - 1] = 0xFB;
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

}  // namespace ac_hi
}  // namespace esphome
