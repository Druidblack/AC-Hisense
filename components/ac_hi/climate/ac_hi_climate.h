// SPDX-License-Identifier: MIT
#pragma once

#include "esphome/core/component.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/uart/uart.h"
#include <vector>
#include <string>
#include <cmath>

namespace esphome {
namespace ac_hi {

/**
 * ACHiClimate — нативный ESPHome climate для кондиционеров Ballu/Hisense по RS-485.
 * Чтение статуса:
 *   - периодически шлём короткий кадр (cmd 0x66, фиксированный однобайтный CRC);
 *   - после первого же входящего кадра «обучаемся» заголовку [2..12] и
 *     дополнительно опрашиваем длинным «чистым» 0x29/0x66 с двухбайтным CRC.
 * Запись — длинным 0x29/0x65.
 */
class ACHiClimate : public climate::Climate, public Component, public uart::UARTDevice {
 public:
  void set_update_interval(uint32_t ms) { update_interval_ms_ = ms; }

  void setup() override;
  void loop() override;
  void dump_config() override;
  climate::ClimateTraits traits() override;

 protected:
  void control(const climate::ClimateCall &call) override;

  // ===== I/O helpers =====
  void send_status_request_short_();        // короткий статус (0x66) — фиксированный кадр
  void send_status_request_long_clean_();   // длинный «чистый» статус (0x29/0x66) — с обученной шапкой
  void send_write_frame_();                 // запись (0x29/0x65)

  void build_base_long_frame_();            // заполнить out_ базовым шаблоном (50 байт) + шапка
  void apply_intent_to_frame_();            // проставить power/mode/temp/fan/swing в out_
  void compute_crc_(std::vector<uint8_t> &buf);

  // разбор входящего потока (с накоплением между итерациями)
  void process_rx_buffer_();
  void handle_status_(const std::vector<uint8_t> &bytes);

  // обучение заголовка [2..12]
  void learn_header_(const std::vector<uint8_t> &bytes);

  // logging helpers
  void log_hex_dump_(const char *prefix, const std::vector<uint8_t> &data);

  // ===== Состояние =====
  bool power_{false};
  float room_temp_{NAN};
  float target_temp_{24.0f};

  climate::ClimateMode mode_{climate::CLIMATE_MODE_AUTO};
  climate::ClimateFanMode fan_{climate::CLIMATE_FAN_AUTO};
  bool swing_ud_{false};
  bool swing_lr_{false};

  // write-intent (по рабочему legacy yaml)
  uint8_t power_bin_{0x04};                 // база 0x04; ON добавляет bit3
  uint8_t mode_bin_{0x10};                  // ((idx<<1)|1)<<4  — odd-nibble схема
  uint8_t wind_code_{0x01};                 // 1=auto; 12/14/16=low/med/high
  uint8_t temp_byte_{(24u << 1) | 1u};      // ((°C)<<1)|1
  uint8_t updown_bin_{0x10};                // 0x30 on, 0x10 off -> [32]
  uint8_t leftright_bin_{0x04};             // 0x0C on, 0x04 off -> [32]
  uint8_t turbo_bin_{0x04};                 // [33] 0x0C on, 0x04 off (держим off)
  uint8_t eco_bin_{0x40};                   // [33] 0xC0 on, 0x40 off (держим off)
  uint8_t quiet_bin_{0x10};                 // [35] 0x30 on, 0x10 off (держим off)

  // рабочий буфер кадра (50 байт)
  std::vector<uint8_t> out_{50, 0x00};

  // «обученная» шапка [2..12]. По умолчанию дефолт, заменяем по первому RX.
  uint8_t header_[11] = {0x00,0x40, 0x29, 0x00,0x00,0x01, 0x01,0xFE,0x01,0x00,0x00};
  bool header_learned_{false};

  // НАКОПИТЕЛЬ ВХОДЯЩЕГО ПОТОКА (устойчив к разрывам кадра между итерациями)
  std::vector<uint8_t> rx_buf_;

  // timing
  uint32_t last_poll_{0};
  uint32_t last_rx_ms_{0};
  uint32_t last_long_status_ms_{0};
  uint32_t update_interval_ms_{2000};
};

}  // namespace ac_hi
}  // namespace esphome
