#pragma once

#include "esphome/components/climate/climate.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#ifdef USE_SENSOR
  #include "esphome/components/sensor/sensor.h"
#endif

#include <vector>
#include <cstdint>
#include <cstddef>
#include <cmath>
#include <algorithm>

namespace esphome {
namespace ac_hi {

class ACHIClimate : public climate::Climate, public PollingComponent, public uart::UARTDevice {
 public:
  ACHIClimate() = default;

  void setup() override {}
  void loop() override;
  void update() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  climate::ClimateTraits traits() override;
  void control(const climate::ClimateCall &call) override;

#ifdef USE_SENSOR
  void set_pipe_sensor(sensor::Sensor *s) { this->pipe_sensor_ = s; }
#endif
  void set_enable_presets(bool v) { this->enable_presets_ = v; }

 protected:
  // Protocol constants
  static constexpr uint8_t HI_HDR0  = 0xF4;
  static constexpr uint8_t HI_HDR1  = 0xF5;
  static constexpr uint8_t HI_TAIL0 = 0xF4;
  static constexpr uint8_t HI_TAIL1 = 0xFB;

  static constexpr uint8_t CMD_WRITE  = 0x65;  // 101
  static constexpr uint8_t CMD_STATUS = 0x66;  // 102
  static constexpr uint8_t CMD_NAK    = 0xFD;  // negative ack on some boards

  // Field indices in long write/status frames (0-based)
  static constexpr int IDX_FAN          = 16;
  static constexpr int IDX_SLEEP        = 17;
  static constexpr int IDX_MODE_POWER   = 18;
  static constexpr int IDX_TARGET_TEMP  = 19;
  static constexpr int IDX_AIR_TEMP     = 20;
  static constexpr int IDX_PIPE_TEMP    = 21;
  static constexpr int IDX_SWING        = 32;  // в legacy кадре WRITE тут магическое 0x50
  static constexpr int IDX_FLAGS        = 33;
  static constexpr int IDX_FLAGS2       = 35;
  static constexpr int IDX_LED          = 36;

  // Desired state
  bool power_on_{false};
  climate::ClimateMode mode_{climate::CLIMATE_MODE_COOL};
  uint8_t target_c_{24};
  climate::ClimateFanMode fan_{climate::CLIMATE_FAN_AUTO};
  climate::ClimateSwingMode swing_{climate::CLIMATE_SWING_OFF};
  uint8_t sleep_stage_{0};
  bool turbo_{false};
  bool eco_{false};
  bool quiet_{false};
  bool led_{false};

  bool enable_presets_{true};

#ifdef USE_SENSOR
  sensor::Sensor *pipe_sensor_{nullptr};
#endif

  // RX buffering
  std::vector<uint8_t> rx_;
  size_t rx_start_{0};

  // Write scheduling / reliability
  bool writing_lock_{false};
  bool dirty_{false};
  uint32_t last_tx_ms_{0};
  uint32_t ack_deadline_ms_{0};
  uint32_t last_status_ms_{0};
  uint32_t force_poll_at_ms_{0};

  // Pending desired snapshot captured at send to allow implicit-ACK via STATUS comparison
  bool have_pending_{false};
  bool pending_power_{false};
  climate::ClimateMode pending_mode_{climate::CLIMATE_MODE_COOL};
  uint8_t pending_target_c_{24};
  climate::ClimateFanMode pending_fan_{climate::CLIMATE_FAN_AUTO};
  climate::ClimateSwingMode pending_swing_{climate::CLIMATE_SWING_OFF};
  bool pending_turbo_{false};
  bool pending_eco_{false};
  bool pending_quiet_{false};
  bool pending_led_{false};

  // Helpers
  void send_query_status_();
  void send_write_frame_(const std::vector<uint8_t> &frame);
  void send_now_();

  bool extract_next_frame_(std::vector<uint8_t> &out);
  void handle_frame_(const std::vector<uint8_t> &frame);
  void parse_status_102_(const std::vector<uint8_t> &b);
  void handle_ack_101_();
  void handle_nak_fd_();

  // Legacy builders
  void build_legacy_write_(std::vector<uint8_t> &frame);
  void build_legacy_query_status_(std::vector<uint8_t> &frame);

  // Legacy CRC16-SUM: sum over [2 .. size-4) -> write HI, then LO into [size-4],[size-3]
  static void crc16_sum_legacy_patch(std::vector<uint8_t> &buf);

  static uint8_t clamp16_30_(uint8_t c) { if (c < 16) return 16; if (c > 30) return 30; return c; }
  static uint8_t encode_mode_hi_write_legacy_(climate::ClimateMode m) {
    switch (m) {
      case climate::CLIMATE_MODE_FAN_ONLY: return 0x00;
      case climate::CLIMATE_MODE_HEAT:     return 0x10;
      case climate::CLIMATE_MODE_COOL:     return 0x20;
      case climate::CLIMATE_MODE_DRY:      return 0x30;
      default:                             return 0x20;
    }
  }
  static uint8_t encode_target_temp_write_legacy_(uint8_t c) {
    c = clamp16_30_(c);
    return static_cast<uint8_t>((c << 1) | 0x01);
  }

  // В WRITE используем те же коды, что видим в STATUS (AUTO=0x02, QUIET=0x0B, LOW=0x0D, MED=0x0F, HIGH=0x11)
  static uint8_t encode_fan_byte_(climate::ClimateFanMode f);
  static uint8_t encode_sleep_byte_(uint8_t stage);
  static uint8_t encode_swing_ud_(bool on);
  static uint8_t encode_swing_lr_(bool on);

  bool reported_matches_pending_(bool rep_power,
                                 climate::ClimateMode rep_mode,
                                 uint8_t rep_set_c,
                                 climate::ClimateFanMode rep_fan,
                                 climate::ClimateSwingMode rep_swing,
                                 bool rep_turbo,
                                 bool rep_eco,
                                 bool rep_quiet,
                                 bool rep_led) const;
};

} // namespace ac_hi
} // namespace esphome
