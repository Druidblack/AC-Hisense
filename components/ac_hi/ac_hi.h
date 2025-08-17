#pragma once

#include "esphome/components/climate/climate.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"

#ifdef USE_SENSOR
  #include "esphome/components/sensor/sensor.h"
#endif

#include <vector>
#include <cstdint>

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

  // Climate
  climate::ClimateTraits traits() override;
  void control(const climate::ClimateCall &call) override;

  // Configuration helpers (from YAML)
#ifdef USE_SENSOR
  void set_pipe_sensor(sensor::Sensor *s) { this->pipe_sensor_ = s; }
#endif
  void set_enable_presets(bool v) { this->enable_presets_ = v; }

 protected:
  // ---- Protocol parameters ----
  static constexpr uint8_t HI_HDR0 = 0xF4;
  static constexpr uint8_t HI_HDR1 = 0xF5;
  static constexpr uint8_t HI_TAIL0 = 0xF4;
  static constexpr uint8_t HI_TAIL1 = 0xFB;

  static constexpr uint8_t CMD_WRITE = 0x65;   // 101
  static constexpr uint8_t CMD_STATUS = 0x66;  // 102

  // TX/RX fixed indexes used by this protocol (empirically derived)
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
  static constexpr int IDX_MISC         = 37;

  // Incoming stream buffer
  std::vector<uint8_t> rx_;
  size_t rx_start_{0};

  // Transmission frame template. We will patch fields and checksum before send.
  std::vector<uint8_t> tx_bytes_ = {
      0xF4,0xF5,  // header
      0x00,0x40,  // proto markers
      0x29,0x00,0x00,0x01,0x01,  // addr/seq (kept constant)
      0xFE,0x01,0x00,0x00,
      CMD_WRITE,  // command
      0x00,0x00,  // payload len hi/lo (kept zero for this variant)
      0x23,       // [16] fan
      0x45,       // [17] sleep
      0x00,       // [18] mode|power
      0x00,       // [19] target temp
      0x00,       // [20] air temp (read-only)
      0x00,       // [21] pipe temp (read-only)
      0x00,0x00,0x00,0x00,0x00,0x00, // [22..27]
      0x00,0x00,0x00,0x00,0x00,      // [28..32] last is swing
      0x00,       // [33] flags (turbo/eco)
      0x00,       // [34]
      0x00,       // [35] quiet + swing report
      0x00,       // [36] LED
      0x00,       // [37] misc
      0x00,0x00,  // [38..39] reserved
      0x00,0x00,  // CRC hi/lo (patched)
      0xF4,0xFB   // tail
  };

  // Query frame for status
  const std::vector<uint8_t> query_ = {
      0xF4,0xF5,0x00,0x40,0x0C,0x00,0x00,0x01,0x01,
      0xFE,0x01,0x00,0x00, CMD_STATUS, 0x00,0x00,0x00,0x01,
      0xB3, 0xF4,0xFB
  };

  // ---- State and scheduling ----
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

  // write/ack orchestration
  bool writing_lock_{false};
  bool dirty_{false};
  uint32_t last_tx_ms_{0};
  uint32_t ack_deadline_ms_{0};
  static constexpr uint32_t kMinGapMs      = 120;
  static constexpr uint32_t kAckTimeoutMs  = 800;

  // ---- Transport helpers ----
  void send_query_status_();
  void send_write_frame_(const std::vector<uint8_t> &frame);
  void send_now_();
  void calc_and_patch_crc_(std::vector<uint8_t> &buf) const;
  bool extract_next_frame_(std::vector<uint8_t> &frame);
  void handle_frame_(const std::vector<uint8_t> &frame);
  void handle_ack_101_();
  void parse_status_102_(const std::vector<uint8_t> &frame);

  // ---- Encoding helpers (legacy scheme) ----
  static uint8_t encode_mode_hi_(climate::ClimateMode m);          // ( (code<<1)|1 ) << 4
  static uint8_t encode_power_lo_(bool on);                         // 0x0C (on) / 0x04 (off)
  static uint8_t encode_target_temp_(uint8_t c);                    // (2*c + 1)
  static climate::ClimateMode decode_mode_from_hi_(uint8_t b18_hi); // map 1,3,5,7,9 to modes
  static bool decode_power_from_lo_(uint8_t b18_lo);                // bit3
  static uint8_t clamp16_30_(int v) { return v < 16 ? 16 : (v > 30 ? 30 : (uint8_t)v); }

  // other field encoders used in this protocol
  static uint8_t encode_fan_byte_(climate::ClimateFanMode f);
  static uint8_t encode_sleep_byte_(uint8_t stage);
  static uint8_t encode_swing_ud_(bool on);
  static uint8_t encode_swing_lr_(bool on);
};

}  // namespace ac_hi
}  // namespace esphome
