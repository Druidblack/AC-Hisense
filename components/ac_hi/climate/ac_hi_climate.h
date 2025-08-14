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
 * Реализация основана на рабочем legacy-конфиге (ballu_legacy.yaml): читаем статус (cmd 0x66),
 * формируем запись изменяя поля последнего статус-кадра и отправляя cmd 0x65 с корректной CRC.
 * YAML-лямбды не нужны: всё управление через ClimateCall.
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
  void send_status_request_();
  void send_write_frame_();
  void rebuild_write_frame_from_last_status_();
  void compute_crc_(std::vector<uint8_t> &buf);
  bool parse_next_frame_();
  void handle_status_(const std::vector<uint8_t> &bytes);

  // ===== State =====
  // last known state (from status 102)
  bool power_{false};
  float room_temp_{NAN};
  float target_temp_{NAN};
  climate::ClimateMode mode_{climate::CLIMATE_MODE_AUTO};
  climate::ClimateFanMode fan_{climate::CLIMATE_FAN_AUTO};
  bool swing_ud_{false};
  bool swing_lr_{false};

  // write-intent fields (match legacy encoding)
  uint8_t power_bin_{0x04};      // base (bit2); ON adds bit3
  uint8_t mode_bin_{0x10};       // ((idx<<1)|1)<<4
  uint8_t wind_code_{0x01};      // 1=auto; low/med/high -> 12/14/16
  uint8_t temp_byte_{0x00};      // ((°C)<<1)|1, or 0 when turbo override
  uint8_t updown_bin_{0x10};     // 0x30 on, 0x10 off -> [32]
  uint8_t leftright_bin_{0x04};  // 0x0C on, 0x04 off -> [32]
  uint8_t turbo_bin_{0x04};      // 0x0C on, 0x04 off -> [33]
  uint8_t eco_bin_{0x40};        // 0xC0 on, 0x40 off -> [33]
  uint8_t quiet_bin_{0x10};      // 0x30 on, 0x10 off -> [35]

  // last received full status frame (used as template for writes)
  std::vector<uint8_t> last_status_;
  bool have_status_template_{false};

  // outbound frame buffer (same length as last_status_)
  std::vector<uint8_t> out_;

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
