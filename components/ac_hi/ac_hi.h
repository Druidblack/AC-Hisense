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
  static constexpr int IDX_SWING        = 32;
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

  // RX/TX state
  std::vector<uint8_t> rx_;
  size_t rx_start_{0};

  // Templates for TX frames (WRITE and STATUS query)
  std::vector<uint8_t> write_template_{
    // 0..12 header
    0xF4,0xF5,0x00,0x40,0x20,0x00,0x00,0x01,0x01,0xFE,0x01,0x00,0x00,
    // 13 cmd
    0x65,
    // 14..16 misc (will be overwritten below as needed)
    0x00,0x00,0x00,
    // 17.. payload (we override the fields below)
    0x00, // sleep
    0x00, // mode|power
    0x00, // target
    0x00, // air temp (ignored on write)
    0x00, // pipe temp (ignored on write)
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // padding
    0x50,0x00,0x00,0x00, // reserved as in logs
    0x40, // placeholder near LED/flags area (we override indices below)
    // CRC (2 bytes) + tail (2 bytes) — will be set dynamically by builder
    0x00,0x00,0xF4,0xFB
  };

  std::vector<uint8_t> query_{
    0xF4,0xF5,0x00,0x40,0x11,0x00,0x00,0x01,0x01,0xFE,0x01,0x00,0x00,0x66,
    0x00,0x00,0x00,0x00, // padding
    0x00,0x00,0xF4,0xFB
  };

  // Write control
  bool writing_lock_{false};
  bool dirty_{false};
  uint32_t last_tx_ms_{0};
  uint32_t ack_deadline_ms_{0};
  uint32_t last_status_ms_{0};
  uint32_t force_poll_at_ms_{0};

  // Multi-variant write attempts
  uint8_t write_attempt_{0};       // 0..N-1
  static constexpr uint8_t kWriteAttemptsMax = 8;

  static constexpr uint32_t kMinGapMs         = 120;
  static constexpr uint32_t kAckTimeoutMs     = 1000;
  static constexpr uint32_t kForcePollDelayMs = 150;

  // Prefer 'len=TOTAL' first (legacy expects len byte equal to frame size, e.g. 0x29)
  static constexpr uint8_t kInitialAttempt = 4; // 4: len=total, CRC=SUM16

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

  void build_variant_(uint8_t attempt, std::vector<uint8_t> &frame);
  void calc_and_patch_crc1_(std::vector<uint8_t> &buf) const;
  void calc_and_patch_crc16_sum_(std::vector<uint8_t> &buf) const;
  void calc_and_patch_crc16_modbus_(std::vector<uint8_t> &buf) const;
  void calc_and_patch_crc16_ccitt_(std::vector<uint8_t> &buf) const;

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

  static uint8_t encode_power_lo_write_(bool on) { return on ? 0x0C : 0x04; } // write lo-nibble
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
