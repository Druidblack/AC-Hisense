#pragma once

#include "esphome/components/climate/climate.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/remote_base/remote_base.h"
#include "esphome/core/automation.h"
#include "kelon168_protocol.h"

#ifdef USE_SENSOR
  #include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_TEXT_SENSOR
  #include "esphome/components/text_sensor/text_sensor.h"
#endif

#include <vector>
#include <cstdint>
#include <algorithm>
#include <string>

namespace esphome {
namespace ac_hi {

// Forward declarations
class ACHIClimate;

enum ACHIIFeelMqttPayloadFormat : uint8_t {
  IFEEL_MQTT_PAYLOAD_HEX = 0,
  IFEEL_MQTT_PAYLOAD_JSON = 1,
};

// Simple switch that controls desired LED flag
class ACHILEDTargetSwitch : public switch_::Switch {
 public:
  void set_parent(ACHIClimate *p) { parent_ = p; }
 protected:
  void write_state(bool state) override;
 private:
  ACHIClimate *parent_{nullptr};
};

// Simple switch that enables/disables audible confirmation for user commands
class ACHICommandSoundSwitch : public switch_::Switch {
 public:
  void set_parent(ACHIClimate *p) { parent_ = p; }
 protected:
  void write_state(bool state) override;
 private:
  ACHIClimate *parent_{nullptr};
};

// Simple switch that enables/disables restoring the last active mode on generic power-on.
class ACHIMemorySwitch : public switch_::Switch {
 public:
  void set_parent(ACHIClimate *p) { parent_ = p; }
 protected:
  void write_state(bool state) override;
 private:
  ACHIClimate *parent_{nullptr};
};

// Protocol constants
static constexpr uint8_t HI_HDR0 = 0xF4;
static constexpr uint8_t HI_HDR1 = 0xF5;
static constexpr uint8_t HI_TAIL0 = 0xF4;
static constexpr uint8_t HI_TAIL1 = 0xFB;
static constexpr uint8_t KELON168_FOLLOW_ME_ENABLED = 0x80;


// Indexes of interesting bytes in the frame (0-based)
enum FrameIndex : uint8_t {
  IDX_CMD = 13,
  IDX_WIND = 16,
  IDX_SLEEP = 17,
  IDX_POWER_MODE = 18,
  IDX_SET_TEMP = 19,
  IDX_CURRENT_TEMP = 20,
  IDX_PIPE_TEMP = 21,

  // Write-frame indexes.
  IDX_TX_BEEP = 23,
  IDX_TX_SWING = 32,
  IDX_TX_TURBO_ECO = 33,
  IDX_TX_QUIET = 35,
  IDX_TX_LED = 36,

  // Status-frame indexes.
  IDX_RX_SWING_TURBO_ECO = 35,
  IDX_RX_QUIET = 36,
  IDX_RX_LED = 37,

  IDX_COMP_FREQ_SET = 42,
  IDX_COMP_FREQ = 43,
  IDX_OUTDOOR_TEMP = 44,
  IDX_OUTDOOR_COND_TEMP = 45,
  IDX_COMPRESSOR_EXHAUST_TEMP = 46,
};

// Bit masks within specific bytes
enum BitMasks : uint8_t {
  POWER_MASK      = 0b00001000,
  MODE_NIBBLE_MASK = 0b11110000,
  TURBO_MASK      = 0b00000010,   // in byte 35
  ECO_MASK        = 0b00000100,   // in byte 35
  QUIET_MASK      = 0b00000100,   // in byte 36
  LED_MASK        = 0b10000000,   // in byte 37
  UPDOWN_MASK     = 0b10000000,   // in byte 35
  LEFTRIGHT_MASK  = 0b01000000,   // in byte 35
};

// Values for turbo/eco/quiet encoding (matching legacy YAML)
namespace TxValues {
  constexpr uint8_t BEEP_ON   = 0x04;
  constexpr uint8_t BEEP_OFF  = 0x00;
  constexpr uint8_t TURBO_ON  = 0b00001100;
  constexpr uint8_t TURBO_OFF = 0b00000100;
  constexpr uint8_t ECO_ON    = 0b00110000;
  constexpr uint8_t ECO_OFF   = 0b00010000;
  constexpr uint8_t QUIET_ON  = 0b00110000;
  constexpr uint8_t QUIET_OFF = 0b00010000;
  constexpr uint8_t UPDOWN_ON = 0b11000000;
  constexpr uint8_t UPDOWN_OFF = 0b01000000;
  constexpr uint8_t LEFTRIGHT_ON = 0b00110000;
  constexpr uint8_t LEFTRIGHT_OFF = 0b00010000;
  constexpr uint8_t LED_ON   = 0b11000000;
  constexpr uint8_t LED_OFF  = 0b01000000;
}

// Limits for non‑blocking operation
static constexpr uint8_t  MAX_FRAMES_PER_LOOP = 2;
static constexpr uint32_t MAX_PARSE_TIME_MS   = 20;
static constexpr size_t   RX_COMPACT_THRESHOLD = 512;
static constexpr size_t   RX_BUFFER_RESERVE    = 2048;
static constexpr size_t   MAX_FRAME_BYTES      = 96;
static constexpr uint32_t WRITE_LOCK_TIMEOUT   = 5000;   // ms
static constexpr uint32_t CONTROL_DEBOUNCE_MS  = 200;    // ms
static constexpr uint32_t MEM_PUBLISH_INTERVAL_MS = 5000; // for memory diagnostics
static constexpr uint32_t STARTUP_POLL_DELAY_MS = 10000;  // delay first AC query after boot
static constexpr uint16_t MAX_UART_BYTES_PER_LOOP = 128;  // keep API/Wi-Fi responsive during UART bursts

class ACHIClimate : public climate::Climate, public PollingComponent, public uart::UARTDevice {
 public:
  ACHIClimate() = default;

  // Configuration
  void set_enable_presets(bool v) { enable_presets_ = v; }
#ifdef USE_SENSOR
  void set_pipe_sensor(sensor::Sensor *s) { pipe_sensor_ = s; }
#else
  void set_pipe_sensor(void *) {}
#endif
  void set_led_switch(ACHILEDTargetSwitch *s) { led_switch_ = s; if (led_switch_) led_switch_->set_parent(this); }
  void set_sound_switch(ACHICommandSoundSwitch *s) { sound_switch_ = s; if (sound_switch_) sound_switch_->set_parent(this); }
  void set_memory_switch(ACHIMemorySwitch *s) { memory_switch_ = s; if (memory_switch_) memory_switch_->set_parent(this); }
  void set_ir_transmitter(remote_base::RemoteTransmitterBase *t) { ir_transmitter_ = t; }
  void set_ifeel_mqtt_topic(const std::string &topic) { ifeel_mqtt_topic_ = topic; }
  void set_ifeel_mqtt_payload_format(const std::string &format) {
    ifeel_mqtt_payload_format_ = (format == "json") ? IFEEL_MQTT_PAYLOAD_JSON : IFEEL_MQTT_PAYLOAD_HEX;
  }
  void set_ifeel_mqtt_qos(uint8_t qos) { ifeel_mqtt_qos_ = qos > 2 ? 0 : qos; }
  void set_ifeel_mqtt_retain(bool retain) { ifeel_mqtt_retain_ = retain; }

  // Send/clear iFeel (Follow Me) over Kelon168 IR while UART climate remains the source of truth.
  void send_ifeel(float temperature, bool enabled);

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
  void set_compressor_exhaust_temp_sensor(sensor::Sensor *s) { compressor_exhaust_temp_sensor_ = s; }

  // Memory diagnostics sensors (optional)
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

  void control(const climate::ClimateCall &call) override;
  climate::ClimateTraits traits() override;

  // Called by LED, sound and memory switches
  void set_desired_led(bool on);
  void set_command_sound_enabled(bool on);
  void set_memory_mode_enabled(bool on);

 protected:
  // Protocol I/O
  void send_query_status_();
  void send_write_changes_();

  // IR iFeel helpers
  Kelon168Data build_kelon_state_from_current_(uint8_t command) const;
  Kelon168Data build_ifeel_state_(uint8_t temperature, bool enabled, bool update) const;
  bool transmit_kelon_ir_(const Kelon168Data &data);
  bool publish_kelon_mqtt_(const Kelon168Data &data, const char *kind, bool enabled, uint8_t temperature);
  void emit_kelon_ifeel_(Kelon168Data data, const char *kind, bool enabled, uint8_t temperature);
  std::string kelon168_to_hex_(const Kelon168Data &data) const;
  std::string kelon168_to_json_(const Kelon168Data &data, const char *kind, bool enabled, uint8_t temperature) const;
  void set_kelon_fan_(Kelon168Data *data, climate::ClimateFanMode fan_mode, bool turbo_fan) const;
  uint8_t encode_kelon_mode_(climate::ClimateMode mode) const;
  void calc_and_patch_crc_(std::vector<uint8_t> &buf);
  bool validate_crc_(const std::vector<uint8_t> &buf, uint16_t *out_sum = nullptr) const;

  // RX parser
  void try_parse_frames_from_buffer_(uint32_t budget_ms = MAX_PARSE_TIME_MS);
  bool extract_next_frame_(std::vector<uint8_t> &frame);
  void handle_frame_(const std::vector<uint8_t> &frame);
  void parse_status_102_(const std::vector<uint8_t> &b);
  void handle_ack_101_();

  // State management
  void build_tx_from_desired_();
  void publish_gated_state_();
  void update_led_switch_state_();
  void update_sound_switch_state_();
  void update_memory_switch_state_();
  void publish_fan_state_(bool turbo_fan, climate::ClimateFanMode fan);
  void maybe_force_to_target_();                     // <-- добавлено объявление
  void maybe_send_pending_control_();                // (опционально, если используется)

  // Signatures for convergence detection
  uint32_t compute_control_signature_(bool power, climate::ClimateMode mode,
                                      climate::ClimateFanMode fan, bool fan_turbo,
                                      climate::ClimateSwingMode swing,
                                      bool eco, bool turbo, bool quiet, bool led,
                                      uint8_t sleep_stage, uint8_t target_c) const;
  void recalc_desired_sig_();
  void recalc_actual_sig_();
  void log_sig_diff_() const;

  // Diagnostics
  void publish_memory_diagnostics_();

  // Field encoders (TX)
  uint8_t encode_temp_(uint8_t c) {
    return static_cast<uint8_t>(((std::max<uint8_t>(16, std::min<uint8_t>(30, c))) << 1) | 0x01);
  }
  uint8_t encode_mode_hi_nibble_(climate::ClimateMode m);
  uint8_t encode_fan_byte_(climate::ClimateFanMode f, bool turbo_fan);
  uint8_t encode_sleep_byte_(uint8_t stage);

  // Logging helper
  void log_frame_(const char *prefix, const std::vector<uint8_t> &b) const;

  // ----- Buffers and state -----
  std::vector<uint8_t> rx_;
  size_t rx_start_{0};

  bool writing_lock_{false};
  uint32_t write_lock_time_{0};               // when lock was set

  // Pending control from HA (debounced)
  bool pending_control_{false};
  uint32_t last_control_ms_{0};

  // Boot guard: avoids querying the indoor AC controller while it is still starting.
  uint32_t boot_ms_{0};

  // Audible beep is requested only for real user commands when command sound is enabled.
  // Automatic HA-priority re-sends stay silent to avoid repeated beeping.
  bool beep_on_next_write_{false};

  // Marks a real user command. This is separate from beep_on_next_write_ because
  // display-off climate commands still need one LED_OFF action to keep the display off
  // even when audible confirmation is disabled.
  bool user_command_next_write_{false};

  // User-configurable local mute for command confirmation beeps.
  bool command_sound_enabled_{true};

  // When enabled, a generic climate.turn_on restores the last active HVAC mode
  // instead of accepting Home Assistant's first-supported-mode fallback.
  bool memory_mode_enabled_{false};

  // Optional Kelon168 IR/MQTT output for iFeel / Follow Me commands.
  remote_base::RemoteTransmitterBase *ir_transmitter_{nullptr};
  std::string ifeel_mqtt_topic_{};
  ACHIIFeelMqttPayloadFormat ifeel_mqtt_payload_format_{IFEEL_MQTT_PAYLOAD_HEX};
  uint8_t ifeel_mqtt_qos_{0};
  bool ifeel_mqtt_retain_{false};
  bool ifeel_enabled_{false};
  uint8_t ifeel_temperature_{0};

  // Byte 36 in TX is an action-style display command, not a stable state field.
  // Keep it neutral for normal climate writes so repeated silent retries do not
  // re-send the display OFF command and make the indoor unit beep.
  bool led_command_pending_{false};

  // Last raw fan/wind code from status frame. Some presets are acknowledged
  // only indirectly via this code when the display is off.
  uint8_t last_raw_wind_{0};

  // Base write frame (template)
  std::vector<uint8_t> tx_bytes_ = {
      0xF4, 0xF5, 0x00, 0x40, 0x29, 0x00, 0x00, 0x01, 0x01, 0xFE, 0x01, 0x00, 0x00,
      0x65, 0x00, 0x00, 0x00, // 0..16
      0x00, // [17] sleep
      0x00, // [18] power+mode
      0x00, // [19] set temp (encoded)
      0x00, // [20] current temp (RO)
      0x00, // [21] pipe temp (RO)
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 22..29
      0x00, 0x00, // 30..31
      0x00, // [32] swing UD/LR
      0x00, // [33] turbo/eco
      0x00, // [34]
      0x00, // [35] quiet
      0x00, // [36] LED + misc
      0x00, // [37]
      0x00, // [38]
      0x00, 0x00, 0x00, 0x00, 0x00, // 39..43
      0x00, 0x00, // 44..45
      0x00, 0x00, // 46..47 CRC (patched)
      0xF4, 0xFB
  };

  // Short status query (cmd 0x66) – CRC is correct for this template
  const std::vector<uint8_t> query_ = {
      0xF4, 0xF5, 0x00, 0x40, 0x0C, 0x00, 0x00, 0x01, 0x01,
      0xFE, 0x01, 0x00, 0x00, 0x66, 0x00, 0x00, 0x00, 0x01,
      0xB3, 0xF4, 0xFB
  };

  // ----- Actual (parsed from AC) state -----
  bool power_on_{false};
  uint8_t target_c_{24};                // 16..30 °C
  climate::ClimateMode mode_{climate::CLIMATE_MODE_OFF};
  climate::ClimateFanMode fan_{climate::CLIMATE_FAN_AUTO};
  bool fan_turbo_{false};
  climate::ClimateSwingMode swing_{climate::CLIMATE_SWING_OFF};
  bool turbo_{false};
  bool eco_{false};
  bool quiet_{false};
  bool led_{true};
  uint8_t sleep_stage_{0};              // 0..4

  // ----- Desired (from HA) state -----
  bool d_power_on_{false};
  uint8_t d_target_c_{24};
  climate::ClimateMode d_mode_{climate::CLIMATE_MODE_OFF};
  climate::ClimateMode last_active_mode_{climate::CLIMATE_MODE_COOL};
  climate::ClimateFanMode d_fan_{climate::CLIMATE_FAN_AUTO};
  bool d_fan_turbo_{false};
  climate::ClimateSwingMode d_swing_{climate::CLIMATE_SWING_OFF};
  bool d_turbo_{false};
  bool d_eco_{false};
  bool d_quiet_{false};
  bool d_led_{true};
  uint8_t d_sleep_stage_{0};

  // Priority flags
  bool accept_remote_changes_{true};    // when true: apply status changes to HA
  bool ha_priority_active_{false};      // when true: enforce desired until matched

  // Signatures
  uint32_t desired_sig_{0};
  uint32_t actual_sig_{0};

  // Last CRC for suppression (optional)
  uint16_t last_status_crc_{0};

  // Optional sensors and switches
#ifdef USE_SENSOR
  sensor::Sensor *pipe_sensor_{nullptr};
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
  sensor::Sensor *compressor_exhaust_temp_sensor_{nullptr};

  // Memory diagnostics
  sensor::Sensor *heap_free_sensor_{nullptr};
  sensor::Sensor *heap_total_sensor_{nullptr};
  sensor::Sensor *heap_used_sensor_{nullptr};
  sensor::Sensor *heap_min_free_sensor_{nullptr};
  sensor::Sensor *heap_max_alloc_sensor_{nullptr};
  sensor::Sensor *heap_fragmentation_sensor_{nullptr};
  sensor::Sensor *psram_total_sensor_{nullptr};
  sensor::Sensor *psram_free_sensor_{nullptr};
#endif
#ifdef USE_TEXT_SENSOR
  text_sensor::TextSensor *power_status_text_{nullptr};
#endif

  ACHILEDTargetSwitch *led_switch_{nullptr};
  ACHICommandSoundSwitch *sound_switch_{nullptr};
  ACHIMemorySwitch *memory_switch_{nullptr};

  bool enable_presets_{true};

  // For debugging (optional)
  std::vector<uint8_t> last_status_frame_;
  std::vector<uint8_t> last_tx_frame_;
};

template<typename... Ts> class ACHIIFeelAction : public Action<Ts...> {
 public:
  explicit ACHIIFeelAction(ACHIClimate *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(float, temperature)
  TEMPLATABLE_VALUE(bool, enabled)

 protected:
  void play(const Ts &...x) override {
    this->parent_->send_ifeel(this->temperature_.value(x...), this->enabled_.value_or(x..., true));
  }

  ACHIClimate *parent_;
};


}  // namespace ac_hi
}  // namespace esphome