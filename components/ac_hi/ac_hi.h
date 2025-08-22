#pragma once

#include "esphome/components/climate/climate.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"  // esphome::millis()
#include "esphome/components/switch/switch.h"  // LED target switch

// Include sensor header only if the sensor platform is actually present in the build
#ifdef USE_SENSOR
  #include "esphome/components/sensor/sensor.h"
#endif

// Include text_sensor header only if the platform is present
#ifdef USE_TEXT_SENSOR
  #include "esphome/components/text_sensor/text_sensor.h"
#endif

#include <vector>
#include <cstddef>
#include <cstdint>
#include <string>
#include <algorithm>

namespace esphome {
namespace sensor {
class Sensor;  // forward declaration if USE_SENSOR is not defined
}  // namespace sensor
namespace text_sensor {
class TextSensor; // forward declaration if USE_TEXT_SENSOR is not defined
}  // namespace text_sensor
}  // namespace esphome

namespace esphome {
namespace ac_hi {

class ACHIClimate;  // forward

// Simple switch entity that controls desired LED flag inside ACHIClimate
class ACHILEDTargetSwitch : public switch_::Switch {
 public:
  void set_parent(ACHIClimate *p) { parent_ = p; }

 protected:
  // Called by HA when the user toggles the switch
  void write_state(bool state) override;

 private:
  ACHIClimate *parent_{nullptr};
};

// Hisense frames:
// Header: 0xF4 0xF5
// Tail  : 0xF4 0xFB
// bytes[4] — "declared length", full frame size = bytes[4] + 9
static constexpr uint8_t HI_HDR0 = 0xF4;
static constexpr uint8_t HI_HDR1 = 0xF5;
static constexpr uint8_t HI_TAIL0 = 0xF4;
static constexpr uint8_t HI_TAIL1 = 0xFB;

// Limits to avoid blocking the app loop
static constexpr uint8_t  MAX_FRAMES_PER_LOOP = 2;    // at most 2 frames per loop()
static constexpr uint32_t MAX_PARSE_TIME_MS   = 20;   // and at most 20 ms parsing budget per loop
static constexpr size_t   RX_COMPACT_THRESHOLD = 512; // compact RX buffer after consuming >512 bytes

// Memory/diagnostic tuning (no protocol changes)
static constexpr size_t RX_BUFFER_RESERVE  = 2048; // pre-reserved capacity for RX stream buffer
static constexpr size_t MAX_FRAME_BYTES    = 96;   // generous upper bound (> typical ~50B)
static constexpr size_t LOG_BYTES_PER_LINE = 24;   // chunk size for hex dump logging

// Memory metrics publish interval (keep it low to reduce noise/overhead)
static constexpr uint32_t MEM_PUBLISH_INTERVAL_MS = 5000;

class ACHIClimate : public climate::Climate, public PollingComponent, public uart::UARTDevice {
 public:
  ACHIClimate() = default;

  // Config
  void set_enable_presets(bool v) { enable_presets_ = v; }
#ifdef USE_SENSOR
  void set_pipe_sensor(sensor::Sensor *s) { pipe_sensor_ = s; }
#else
  void set_pipe_sensor(void *) {}
#endif

  // Link LED target switch
  void set_led_switch(ACHILEDTargetSwitch *s) { led_switch_ = s; if (led_switch_) led_switch_->set_parent(this); }

  // Optional sensors setters (created only if present in YAML)
#ifdef USE_SENSOR
  void set_set_temperature_sensor(sensor::Sensor *s) { set_temp_sensor_ = s; }
  void set_room_temperature_sensor(sensor::Sensor *s) { room_temp_sensor_ = s; }
  void set_wind_sensor(sensor::Sensor *s) { wind_code_sensor_ = s; }
  void set_sleep_stage_sensor(sensor::Sensor *s) { sleep_code_sensor_ = s; }
  void set_mode_code_sensor(sensor::Sensor *s) { mode_code_sensor_ = s; }
  void set_quiet_sensor(sensor::Sensor *s) { quiet_code_sensor_ = s; }
  void set_turbo_sensor(sensor::Sensor *s) { turbo_code_sensor_ = s; }
  void set_economy_sensor(sensor::Sensor *s) { eco_code_sensor_ = s; }
  void set_swing_ud_sensor(sensor::Sensor *s) { swing_ud_sensor_ = s; }
  void set_swing_lr_sensor(sensor::Sensor *s) { swing_lr_sensor_ = s; }
  void set_compr_freq_set_sensor(sensor::Sensor *s) { compressor_freq_set_sensor_ = s; }
  void set_compr_freq_sensor(sensor::Sensor *s) { compressor_freq_sensor_ = s; }
  void set_outdoor_temp_sensor(sensor::Sensor *s) { outdoor_temp_sensor_ = s; }
  void set_outdoor_cond_temp_sensor(sensor::Sensor *s) { outdoor_cond_temp_sensor_ = s; }

  // New: memory diagnostics sensors
  void set_heap_free_sensor(sensor::Sensor *s) { heap_free_sensor_ = s; }
  void set_heap_total_sensor(sensor::Sensor *s) { heap_total_sensor_ = s; }
  void set_heap_used_sensor(sensor::Sensor *s) { heap_used_sensor_ = s; }
  void set_heap_min_free_sensor(sensor::Sensor *s) { heap_min_free_sensor_ = s; }
  void set_heap_max_alloc_sensor(sensor::Sensor *s) { heap_max_alloc_sensor_ = s; }
  void set_heap_fragmentation_sensor(sensor::Sensor *s) { heap_fragmentation_sensor_ = s; }
  void set_psram_total_sensor(sensor::Sensor *s) { psram_total_sensor_ = s; }
  void set_psram_free_sensor(sensor::Sensor *s) { psram_free_sensor_ = s; }
#endif
#ifdef USE_TEXT_SENSOR
  void set_power_status_text(text_sensor::TextSensor *t) { power_status_text_ = t; }
#endif

  void setup() override;
  void loop() override;
  void update() override;

  // Climate API
  void control(const climate::ClimateCall &call) override;
  climate::ClimateTraits traits() override;

  // Called by ACHILEDTargetSwitch to change desired LED
  void set_desired_led(bool on);

 protected:
  // ---- Protocol/transport ----
  void send_query_status_();    // short status request (cmd 0x66)
  void send_write_changes_();   // full state write with CRC
  void calc_and_patch_crc_(std::vector<uint8_t> &buf);
  bool validate_crc_(const std::vector<uint8_t> &buf, uint16_t *out_sum = nullptr) const;

  // RX framer/parser
  void try_parse_frames_from_buffer_(uint32_t budget_ms = MAX_PARSE_TIME_MS); // stream scanner with time budget
  bool extract_next_frame_(std::vector<uint8_t> &frame);                       // extracts [F4 F5 ... F4 FB]
  void handle_frame_(const std::vector<uint8_t> &frame);
  void parse_status_102_(const std::vector<uint8_t> &b);
  void handle_ack_101_();

  // --- Priority/authority helpers ---
  void build_tx_from_desired_();   // build tx_bytes_ from desired_* fields (no mapping/proto change)
  void publish_gated_state_();     // publish either desired or actual into HA, sensors always actual
  void update_led_switch_state_(); // publish LED switch state according to gating
  void maybe_force_to_target_();   // enforce desired state while HA priority is active

  // Control signature includes LED and excludes sensor-only bytes
  uint32_t compute_control_signature_(bool power, climate::ClimateMode mode,
                                      climate::ClimateFanMode fan, climate::ClimateSwingMode swing,
                                      bool eco, bool turbo, bool quiet, bool led,
                                      uint8_t sleep_stage, uint8_t target_c) const;
  void recalc_desired_sig_();
  void recalc_actual_sig_();
  void log_sig_diff_() const;      // verbose: which control fields differ (for debugging only)

  // Memory diagnostics publish helper (no protocol impact)
  void publish_memory_diagnostics_();

  // Incoming stream buffer (sliding window: rx_start_ — offset of data start)
  std::vector<uint8_t> rx_;
  size_t rx_start_{0};

  bool writing_lock_{false};
  bool pending_write_{false};

  // Base write frame (as in YAML initial vector)
  std::vector<uint8_t> tx_bytes_ = {
      0xF4, 0xF5, 0x00, 0x40, 0x29, 0x00, 0x00, 0x01, 0x01, 0xFE, 0x01, 0x00, 0x00,
      0x65, 0x00, 0x00, 0x00, // 0..16
      0x00, // [17] sleep
      0x00, // [18] power+mode
      0x00, // [19] set temp (°C, direct)
      0x00, // [20] current temp (RO)
      0x00, // [21] pipe temp (RO)
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 22..29
      0x00, 0x00, // 30..31
      0x00, // [32] swing UD/LR
      0x00, // [33] turbo/eco (tx)
      0x00, // [34]
      0x00, // [35] quiet
      0x00, // [36] LED + misc
      0x00, // [37]
      0x00, // [38]
      0x00, 0x00, 0x00, 0x00, 0x00, // 39..43
      0x00, 0x00, // 44..45
      0x00, 0x00, // 46..47 CRC (will be patched)
      0xF4, 0xFB   // tail
  };

  // Short status query (cmd 0x66) — CRC is already "correct" for this template
  const std::vector<uint8_t> query_ = {
      0xF4, 0xF5, 0x00, 0x40, 0x0C, 0x00, 0x00, 0x01, 0x01,
      0xFE, 0x01, 0x00, 0x00, 0x66, 0x00, 0x00, 0x00, 0x01,
      0xB3, 0xF4, 0xFB
  };

  // ---- Actual (parsed from status) state ----
  bool power_on_{false};
  uint8_t target_c_{24}; // 16..30
  climate::ClimateMode mode_{climate::CLIMATE_MODE_OFF};
  climate::ClimateFanMode fan_{climate::CLIMATE_FAN_AUTO};
  climate::ClimateSwingMode swing_{climate::CLIMATE_SWING_OFF};
  bool turbo_{false};
  bool eco_{false};
  bool quiet_{false};
  bool led_{true};
  uint8_t sleep_stage_{0}; // 0..4

  // ---- Desired (from HA) state ----
  bool d_power_on_{false};
  uint8_t d_target_c_{24};
  climate::ClimateMode d_mode_{climate::CLIMATE_MODE_OFF};
  climate::ClimateFanMode d_fan_{climate::CLIMATE_FAN_AUTO};
  climate::ClimateSwingMode d_swing_{climate::CLIMATE_SWING_OFF};
  bool d_turbo_{false};
  bool d_eco_{false};
  bool d_quiet_{false};
  bool d_led_{true};  // default ON
  uint8_t d_sleep_stage_{0};

  // Acceptance/priority flags
  bool accept_remote_changes_{true};  // when true: apply remote (status) mode changes to HA
  bool ha_priority_active_{false};    // when true: keep enforcing desired_* until matched

  // Signatures (control-only hash; LED included)
  uint32_t desired_sig_{0};
  uint32_t actual_sig_{0};

  // For suppression/logging
  uint16_t last_status_crc_{0};

  // Optional pipe sensor
#ifdef USE_SENSOR
  sensor::Sensor *pipe_sensor_{nullptr};
#else
  void *pipe_sensor_{nullptr};
#endif

  // Optional LED target switch
  ACHILEDTargetSwitch *led_switch_{nullptr};

  // Optional numeric sensors (published from status frames)
#ifdef USE_SENSOR
  sensor::Sensor *set_temp_sensor_{nullptr};
  sensor::Sensor *room_temp_sensor_{nullptr};
  sensor::Sensor *wind_code_sensor_{nullptr};
  sensor::Sensor *sleep_code_sensor_{nullptr};
  sensor::Sensor *mode_code_sensor_{nullptr};
  sensor::Sensor *quiet_code_sensor_{nullptr};
  sensor::Sensor *turbo_code_sensor_{nullptr};
  sensor::Sensor *eco_code_sensor_{nullptr};
  sensor::Sensor *swing_ud_sensor_{nullptr};
  sensor::Sensor *swing_lr_sensor_{nullptr};
  sensor::Sensor *compressor_freq_set_sensor_{nullptr};
  sensor::Sensor *compressor_freq_sensor_{nullptr};
  sensor::Sensor *outdoor_temp_sensor_{nullptr};
  sensor::Sensor *outdoor_cond_temp_sensor_{nullptr};

  // New: memory diagnostics sensors
  sensor::Sensor *heap_free_sensor_{nullptr};
  sensor::Sensor *heap_total_sensor_{nullptr};
  sensor::Sensor *heap_used_sensor_{nullptr};
  sensor::Sensor *heap_min_free_sensor_{nullptr};
  sensor::Sensor *heap_max_alloc_sensor_{nullptr};
  sensor::Sensor *heap_fragmentation_sensor_{nullptr};
  sensor::Sensor *psram_total_sensor_{nullptr};
  sensor::Sensor *psram_free_sensor_{nullptr};
#endif

  // Optional text sensor (power status ON/OFF)
#ifdef USE_TEXT_SENSOR
  text_sensor::TextSensor *power_status_text_{nullptr};
#endif

  // Flags
  bool enable_presets_{true};

  // For debugging/analysis
  std::vector<uint8_t> last_status_frame_;
  std::vector<uint8_t> last_tx_frame_;

  // ---- Field encoders ----
  uint8_t encode_temp_(uint8_t c) {
    return static_cast<uint8_t>(((std::max<uint8_t>(16, std::min<uint8_t>(30, c))) << 1) | 0x01);
  }
  uint8_t encode_mode_hi_nibble_(climate::ClimateMode m);
  uint8_t encode_fan_byte_(climate::ClimateFanMode f);
  uint8_t encode_sleep_byte_(uint8_t stage);
  uint8_t encode_swing_ud_(bool on);
  uint8_t encode_swing_lr_(bool on);

  // ---- Logging helpers ----
  // Hex dump without dynamic heap allocations (chunked lines)
  void log_frame_(const char *prefix, const std::vector<uint8_t> &b) const;
};

}  // namespace ac_hi
}  // namespace esphome
