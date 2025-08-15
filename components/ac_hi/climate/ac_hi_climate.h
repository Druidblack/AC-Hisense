// SPDX-License-Identifier: MIT
#pragma once

#include "esphome/core/component.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include <vector>
#include <string>
#include <cmath>
#include <cstring>  // memcpy

namespace esphome {
namespace ac_hi {

/**
 * ACHiClimate — нативный ESPHome climate для кондиционеров Ballu/Hisense по RS-485.
 * Протокол (по legacy):
 *   Кадр:   0xF4 0xF5 ... 0xF4 0xFB
 *   Команды: запись 0x65, статус 0x66, ACK 0x101
 *   Поля:   [16]=fan code, [18]=power/mode (бит3=питание, старшая тетрада=режим), [19]=Tset, [20]=Tcur
 *           свинг/эко/турбо/тихий — биты вокруг [32..37] (см. реализацию).
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
  void build_base_long_frame_();
  void apply_intent_to_frame_();
  void compute_crc_(std::vector<uint8_t> &buf);
  void send_status_request_short_();       // короткий статус (0x66) — фиксированный кадр
  void send_status_request_long_clean_();  // длинный «чистый» статус 0x66 с обученной шапкой
  void send_write_frame_();                // ЗАПИСЬ 0x65 + пост-опрос   <-- добавлено объявление
  void learn_header_(const std::vector<uint8_t> &bytes);
  void handle_status_(const std::vector<uint8_t> &bytes);
  void log_hex_dump_(const char *prefix, const std::vector<uint8_t> &data);

  // ===== runtime =====
  uint32_t update_interval_ms_{2000};
  uint32_t last_poll_{0};
  uint32_t last_rx_ms_{0};
  uint32_t last_long_status_ms_{0};
  uint32_t suppress_until_ms_{0};  // анти-откат окна после записи

  // входной буфер
  std::vector<uint8_t> rx_buf_;

  // «обученная» шапка [2..12] для длинных пакетов
  uint8_t header_[11]{0x00};
  bool header_learned_{false};

  // сформированный исходящий длинный кадр (~50 байт)
  std::vector<uint8_t> out_;

  // текущее намерение/состояние
  bool   power_{false};
  float  target_temp_{24};
  uint8_t temp_byte_{(24U << 1) | 1};
  uint8_t wind_code_{1}; // статусный код: auto=1, low~12, med~14, high~16
  bool swing_ud_{false};
  bool swing_lr_{false};
  uint8_t power_bin_{0}; // 0b00001100=ON, 0b00000100=OFF
  uint8_t mode_bin_{0};  // старшая тетрада: 0=FAN,1=HEAT,2=COOL,3=DRY,4=AUTO
  uint8_t turbo_bin_{0};
  uint8_t eco_bin_{0};
  uint8_t quiet_bin_{0};

  // сенсоры (опционально)
  sensor::Sensor *tset_s_{nullptr};
  sensor::Sensor *tcur_s_{nullptr};
  sensor::Sensor *tout_s_{nullptr};
  sensor::Sensor *tpipe_s_{nullptr};
  sensor::Sensor *compfreq_s_{nullptr};
};

}  // namespace ac_hi
}  // namespace esphome
