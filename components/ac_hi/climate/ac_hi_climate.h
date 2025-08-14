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
 * Статусы читаем командой 0x66, запись параметров — командой 0x65.
 * Кадры длинного формата (тип 0x29) формируются из корректного базового шаблона,
 * CRC — 16-битная сумма по [2..len-5], последние два байта — контрольная сумма,
 * затем хвост 0xF4 0xFB.
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
  void send_status_request_();          // длинный статус (0x29 + 0x66)
  void send_write_frame_();             // запись (0x29 + 0x65)
  void build_base_long_frame_();        // заполнить out_ базовым шаблоном 50 байт
  void apply_intent_to_frame_();        // проставить в out_ текущие power/mode/temp/fan/swing
  void compute_crc_(std::vector<uint8_t> &buf);
  bool parse_next_frame_();
  void handle_status_(const std::vector<uint8_t> &bytes);

  // ===== State =====
  bool power_{false};
  float room_temp_{NAN};
  float target_temp_{24.0f};

  climate::ClimateMode mode_{climate::CLIMATE_MODE_AUTO};
  climate::ClimateFanMode fan_{climate::CLIMATE_FAN_AUTO};
  bool swing_ud_{false};
  bool swing_lr_{false};

  // write-intent поля (соответствуют реверсу)
  uint8_t power_bin_{0x04};      // базовый 0x04; ON добавляет bit3 в составном байте
  uint8_t mode_bin_{0x10};       // ((idx<<1)|1)<<4
  uint8_t wind_code_{0x01};      // 1=auto; low/med/high -> 12/14/16
  uint8_t temp_byte_{(24 << 1) | 1}; // ((°C)<<1)|1
  uint8_t updown_bin_{0x10};     // 0x30 on, 0x10 off -> [32]
  uint8_t leftright_bin_{0x04};  // 0x0C on, 0x04 off -> [32]

  // Последний принятый статус (не обязателен для записи, но полезен для анализа)
  std::vector<uint8_t> last_status_;

  // Рабочий буфер исходящего кадра (50 байт)
  std::vector<uint8_t> out_{50, 0x00};

  // RX ring buffer
  static constexpr size_t RB_SIZE = 512;
  uint8_t rb_[RB_SIZE]{};
  size_t rb_head_{0}, rb_tail_{0};

  // timing
  uint32_t last_poll_{0};
  uint32_t update_interval_ms_{2000};

  // helpers
  void rb_push_(uint8_t b) { rb_[rb_head_] = b; rb_head_ = (rb_head_ + 1) % RB_SIZE; }
  bool rb_pop_(uint8_t &b) {
    if (rb_head_ == rb_tail_) return false;
    b = rb_[rb_tail_]; rb_tail_ = (rb_tail_ + 1) % RB_SIZE; return true;
  }
};

}  // namespace ac_hi
}  // namespace esphome
