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
 * Чтение статуса — длинным кадром (тип 0x29, cmd 0x66, «чистый»), запись — 0x29/0x65.
 * CRC: 16-битная сумма по [2..len-5] → [len-4],[len-3], затем хвост 0xF4 0xFB.
 * ВАЖНО: часть блоков игнорирует запросы с «чужим» заголовком. Мы обучаемся заголовку
 * по любому принятому кадру (в т.ч. ACK на запись) и используем его далее.
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
  void send_status_request_();          // длинный статус (0x29/0x66, clean, с обученным заголовком)
  void send_write_frame_();             // запись (0x29/0x65)
  void build_base_long_frame_();        // заполнить out_ базовым шаблоном (50 байт) + заголовок
  void apply_intent_to_frame_();        // проставить power/mode/temp/fan/swing в out_
  void compute_crc_(std::vector<uint8_t> &buf);
  bool parse_next_frame_();
  void handle_status_(const std::vector<uint8_t> &bytes);

  // обучение заголовка [2..12]
  void learn_header_(const std::vector<uint8_t> &bytes);

  // ===== Состояние =====
  bool power_{false};
  float room_temp_{NAN};
  float target_temp_{24.0f};

  climate::ClimateMode mode_{climate::CLIMATE_MODE_AUTO};
  climate::ClimateFanMode fan_{climate::CLIMATE_FAN_AUTO};
  bool swing_ud_{false};
  bool swing_lr_{false};

  // write-intent (как в рабочем legacy)
  uint8_t power_bin_{0x04};                 // база 0x04; ON добавляет bit3
  uint8_t mode_bin_{0x10};                  // ((idx<<1)|1)<<4  — см. декод ниже (odd nibble)
  uint8_t wind_code_{0x01};                 // 1=auto; 12/14/16=low/med/high
  uint8_t temp_byte_{(24u << 1) | 1u};      // ((°C)<<1)|1
  uint8_t updown_bin_{0x10};                // 0x30 on, 0x10 off -> [32]
  uint8_t leftright_bin_{0x04};             // 0x0C on, 0x04 off -> [32]

  // рабочий буфер кадра (50 байт)
  std::vector<uint8_t> out_{50, 0x00};

  // «обученный» заголовок [2..12]. По умолчанию универсальный, но будет заменён первым же RX.
  uint8_t header_[11] = {0x00,0x40, 0x29, 0x00,0x00,0x01, 0x01,0xFE,0x01,0x00,0x00};
  bool header_learned_{false};

  // RX ring buffer
  static constexpr size_t RB_SIZE = 768;
  uint8_t rb_[RB_SIZE]{};
  size_t rb_head_{0}, rb_tail_{0};

  // timing
  uint32_t last_poll_{0};
  uint32_t last_rx_ms_{0};
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
