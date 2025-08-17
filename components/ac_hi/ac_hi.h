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

  // Component lifecycle
  void setup() override {}
  void loop() override;
  void update() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // Climate API
  climate::ClimateTraits traits() override;
  void control(const climate::ClimateCall &call) override;

#ifdef USE_SENSOR
  void set_pipe_sensor(sensor::Sensor *s) { this->pipe_sensor_ = s; }
#endif
  void set_enable_presets(bool v) { this->enable_presets_ = v; }

 protected:
  // ---- Protocol constants ----
  static constexpr uint8_t HI_HDR0  = 0xF4;
  static constexpr uint8_t HI_HDR1  = 0xF5;
  static constexpr uint8_t HI_TAIL0 = 0xF4;
  static constexpr uint8_t HI_TAIL1 = 0xFB;

  static constexpr uint8_t CMD_WRITE  = 0x65;  // 101
  static constexpr uint8_t CMD_STATUS = 0x66;  // 102

  // Known field indices (based on device frames)
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

  // ---- State (desired + reported) ----
  bool power_on_{false};
  climate::ClimateMode mode_{climate::CLIMATE_MODE_COOL};
  uint8_t target_c_{24};
  climate::ClimateFanMode fan_{climate::CLIMATE_FAN_AUTO};
  climate::ClimateSwingMode swing_{climate::CLIMATE_SWING_OFF};
  bool turbo_{false};
  bool eco_{false};
  bool quiet_{false};
  bool led_{true};
  uint8_t sleep_stage_{0};
  bool enable_presets_{true};

#ifdef USE_SENSOR
  sensor::Sensor *pipe_sensor_{nullptr};
#endif

  // ---- TX templates ----
  std::vector<uint8_t> tx_bytes_ = {
      0xF4,0xF5,0x00,0x40,0x29,0x00,0x00,0x01,0x01,
      0xFE,0x01,0x00,0x00, CMD_WRITE, 0x00,0x00,
      0x23, // [16] fan
      0x45, // [17] sleep
      0x00, // [18] mode|power
      0x00, // [19] target temp
      0x00, // [20] air temp (read)
      0x00, // [21] pipe temp (read)
      0x00,0x00,0x00,0x00,0x00,0x00, // [22..27]
      0x00,0x00,0x00,0x00,0x00,      // [28..32]
      0x00, // [33] flags (turbo/eco)
      0x00, // [34]
      0x00, // [35] quiet/swing report
      0x00, // [36] LED
      0x00, // [37]
      0x00, // [38] checksum (1 byte)
      0xF4,0xFB
  };

  const std::vector<uint8_t> query_ = {
      0xF4,0xF5,0x00,0x40,0x0C,0x00,0x00,0x01,0x01,
      0xFE,0x01,0x00,0x00, CMD_STATUS, 0x00,0x00,0x00,0x01,
      0xB3, 0xF4,0xFB
  };

  // ---- RX buffering ----
  std::vector<uint8_t> rx_;
  size_t rx_start_{0};

  // ---- Write scheduling / reliability ----
  bool writing_lock_{false};       // waiting for ACK of last write
  bool dirty_{false};              // new changes pending send
  uint32_t last_tx_ms_{0};         // last write timestamp
  uint32_t ack_deadline_ms_{0};    // ACK timeout deadline
  uint32_t last_status_ms_{0};     // last time we received a status (implicit ACK)
  uint32_t force_poll_at_ms_{0};   // when to force a status poll after write

  static constexpr uint32_t kMinGapMs     = 120;  // min gap between writes
  static constexpr uint32_t kAckTimeoutMs = 800;  // ACK timeout
  static constexpr uint32_t kForcePollDelayMs = 150; // after write, force one status poll

  // ---- Helpers ----
  void send_query_status_();
  void send_now_();                       // build & send snapshot (sets lock/timers)
  void send_write_frame_(const std::vector<uint8_t> &frame);
  void calc_and_patch_crc_(std::vector<uint8_t> &buf) const;

  bool extract_next_frame_(std::vector<uint8_t> &frame);
  void handle_frame_(const std::vector<uint8_t> &frame);
  void handle_ack_101_();
  void parse_status_102_(const std::vector<uint8_t> &frame);

  // Encoding helpers
  static uint8_t clamp16_30_(int v) { return v < 16 ? 16 : (v > 30 ? 30 : (uint8_t) v); }
  static uint8_t encode_mode_hi_direct_(climate::ClimateMode m) {
    // hi-nibble: 0..3 -> <<4
    uint8_t code = 2;
    switch (m) {
      case climate::CLIMATE_MODE_FAN_ONLY: code = 0; break;
      case climate::CLIMATE_MODE_HEAT:     code = 1; break;
      case climate::CLIMATE_MODE_COOL:     code = 2; break;
      case climate::CLIMATE_MODE_DRY:      code = 3; break;
      default: break;
    }
    return (uint8_t)(code << 4);
  }
  // ВАЖНО: устройство, судя по логам, ожидает LO-nibble=0x00 в статусе → не ставим биты питания в write
  static uint8_t encode_power_lo_neutral_() { return 0x00; }

  static uint8_t encode_target_temp_direct_(uint8_t c) { return clamp16_30_(c); }

  static uint8_t encode_fan_byte_(climate::ClimateFanMode f);
  static uint8_t encode_sleep_byte_(uint8_t stage);
  static uint8_t encode_swing_ud_(bool on);
  static uint8_t encode_swing_lr_(bool on);
};

} // namespace ac_hi
} // namespace esphome
