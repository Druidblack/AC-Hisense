#pragma once
// SPDX-License-Identifier: MIT
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
 * Основан на реверсе legacy/ballu_legacy.yaml: индексы байт статуса и формирование команд.
 * YAML-лямбды не нужны: всё управление через ClimateCall.
 */
class ACHiClimate : public climate::Climate, public Component, public uart::UARTDevice {
 public:
  void set_status_update_interval(uint32_t ms) { update_interval_ms_ = ms; }

  void setup() override;
  void loop() override;
  void dump_config() override;
  climate::ClimateTraits traits() override;

 protected:
  void control(const climate::ClimateCall &call) override;

  // ===== I/O helpers =====
  void send_status_request_();
  void send_write_frame_();
  void rebuild_write_frame_();
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
  bool quiet_{false};
  bool turbo_{false};
  bool eco_{false};
  bool led_{false};

  // write-intent fields
  uint8_t power_bin_{0x04};      // база (bit2); включение добавляет bit3
  uint8_t mode_bin_{0x10};       // ((idx<<1)|1)<<4
  uint8_t wind_code_{0x00};      // 0..18, в запись уходит +1
  uint8_t temp_byte_{0x00};      // ((°C)<<1)|1  или 0 при turbo override
  uint8_t updown_bin_{0x10};     // 0x30 on, 0x10 off -> [32]
  uint8_t leftright_bin_{0x04};  // 0x0C on, 0x04 off -> [32]
  uint8_t turbo_bin_{0x04};      // 0x0C on, 0x04 off -> [33]
  uint8_t eco_bin_{0x40};        // 0xC0 on, 0x40 off -> [33]
  uint8_t quiet_bin_{0x10};      // 0x30 on, 0x10 off -> [35]

  // outbound long frame template (50 bytes)
  std::vector<uint8_t> out_{50, 0};

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
