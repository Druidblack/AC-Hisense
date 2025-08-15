// SPDX-License-Identifier: MIT
#pragma once

#include "esphome/core/component.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include <vector>
#include <string>
#include <cstring>

namespace esphome {
namespace ac_hi {

/**
 * ACHiClimate — ESPHome climate для Ballu/Hisense RS-485.
 * Кадры: F4 F5 ... F4 FB, статус 0x66, запись 0x65.
 * MIRROR-WRITE: копируем последний 0x66 и изменяем ТОЛЬКО [18] (питание+режим) и [19] (уставка).
 */
class ACHiClimate : public climate::Climate, public Component, public uart::UARTDevice {
 public:
  void set_update_interval(uint32_t ms) { update_interval_ms_ = ms; }

  // опциональные сенсоры
  void set_tset_sensor(sensor::Sensor *s)     { tset_s_ = s; }
  void set_tcur_sensor(sensor::Sensor *s)     { tcur_s_ = s; }
  void set_tout_sensor(sensor::Sensor *s)     { tout_s_ = s; }
  void set_tpipe_sensor(sensor::Sensor *s)    { tpipe_s_ = s; }
  void set_compfreq_sensor(sensor::Sensor *s) { compfreq_s_ = s; }

  void setup() override;
  void loop() override;
  void dump_config() override;
  climate::ClimateTraits traits() override;

 protected:
  void control(const climate::ClimateCall &call) override;

  // ===== I/O helpers =====
  void learn_from_status_(const std::vector<uint8_t> &bytes); // учим заголовок/служебные поля
  void handle_status_(const std::vector<uint8_t> &bytes);
  void send_status_request_short_();                          // короткий 0x66
  void send_write_frame_();                                   // MIRROR-WRITE
  void compute_crc_(std::vector<uint8_t> &buf);
  void log_hex_dump_(const char *prefix, const std::vector<uint8_t> &data);

  // ===== runtime =====
  uint32_t update_interval_ms_{2000};
  uint32_t last_poll_{0};
  uint32_t last_rx_ms_{0};

  // анти-откат
  uint32_t suppress_until_ms_{0};
  bool     guard_mode_until_match_{false};
  uint8_t  expected_mode_byte_{0};
  uint32_t guard_deadline_ms_{0};

  // входной буфер
  std::vector<uint8_t> rx_buf_;

  // служебные поля, выученные из статуса
  uint8_t header_[11]{0x01,0x40,0x29,0x01,0x00,0xFE,0x01,0x01,0x01,0x01,0x00}; // дефолт, перезапишем
  uint8_t fld14_{0x00};     // [14]
  uint8_t fld15_{0x01};     // [15] — у тебя 0x01
  uint8_t fld23_{0x80};     // [23] — у тебя 0x80
  bool header_learned_{false};

  // последний валидный статус-кадр (для mirror-write)
  std::vector<uint8_t> last_status_;

  // намерение
  bool    power_{false};
  float   target_temp_{24};
  uint8_t temp_byte_{24};     // просто °C

  // ВАЖНО: power-бит = 0x08 (ON) / 0x00 (OFF)
  uint8_t power_bin_{0};
  uint8_t mode_bin_{0};       // 0=FAN,1=HEAT,2=COOL,3=DRY,4=AUTO

  // сенсоры (опц.)
  sensor::Sensor *tset_s_{nullptr};
  sensor::Sensor *tcur_s_{nullptr};
  sensor::Sensor *tout_s_{nullptr};
  sensor::Sensor *tpipe_s_{nullptr};
  sensor::Sensor *compfreq_s_{nullptr};
};

}  // namespace ac_hi
}  // namespace esphome
