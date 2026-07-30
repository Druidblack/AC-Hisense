#pragma once

#include "esphome/components/climate/climate.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/select/select.h"
#include "esphome/components/remote_base/remote_base.h"
#include "esphome/core/automation.h"
#include "kelon168_protocol.h"

#ifdef USE_SENSOR
  #include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_TEXT_SENSOR
  #include "esphome/components/text_sensor/text_sensor.h"
#endif
#ifdef USE_BINARY_SENSOR
  #include "esphome/components/binary_sensor/binary_sensor.h"
#endif

#include <vector>
#include <cstdint>
#include <algorithm>
#include <string>

namespace esphome {
namespace ac_hi {

// Forward declarations
class ACHIClimate;

enum QueryKind : uint8_t {
  QUERY_NONE = 0,
  QUERY_STATUS = 1,
  QUERY_CAPABILITIES = 2,
};

struct ACHIDeviceCapabilities {
  bool valid{false};
  bool cool_heat{false};
  bool ai{false};
  bool infinite_fan{false};
  bool power_save{false};
  bool fan_mute{false};
  bool swing_dir_8{false};
  bool swing_follow{false};
  uint8_t power_display{0};
  uint8_t demand_response{0};
  bool humidity{false};
  bool heat_8c{false};
  bool purify{false};
  bool ext_valid{false};
  bool q_display{false};
  bool enable_8heat{false};
  bool trans_102_64{false};
  uint8_t reply_len{0};
};

struct ACHITimerState {
  bool known{false};
  bool active{false};
  uint16_t initial_minutes{0};
  uint16_t last_published_remaining{0xFFFF};
  uint16_t last_event_signature{0};
  uint32_t started_ms{0};
  uint32_t last_event_ms{0};
};

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

// Selects which of the four Hisense Sleep programs is used by the standard
// Climate Sleep preset. Selecting an option alone does not enable Sleep.
class ACHISleepProgramSelect : public select::Select {
 public:
  void set_parent(ACHIClimate *p) { parent_ = p; }
 protected:
  void control(const std::string &value) override;
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
  IDX_INDOOR_HUMIDITY_SETTING = 22,
  IDX_INDOOR_HUMIDITY = 23,
  // Display temperature unit reported by the indoor unit. Bit 1: 0=C, 1=F.
  IDX_TEMP_UNIT = 26,

  // Relative onboard timers in the ordinary 0x66/0x00 status reply.
  // ON timer: hour in byte 30 bits 7..3, minute in byte 31 bits 7..2,
  // active flag in byte 31 bit 0. OFF timer uses bytes 32 and 33.
  IDX_ON_TIMER_HOUR = 30,
  IDX_ON_TIMER_MINUTE_STATUS = 31,
  IDX_OFF_TIMER_HOUR = 32,
  IDX_OFF_TIMER_MINUTE_STATUS = 33,

  // Fault groups in the ordinary long 0x66/0x00 status reply.
  IDX_FAULT_INDOOR = 39,
  IDX_FAULT_MODULE = 40,
  IDX_FAULT_OUTDOOR = 64,
  IDX_FAULT_PROTECT = 66,

  // Write-frame indexes.
  IDX_TX_BEEP = 23,
  IDX_TX_SWING = 32,
  IDX_TX_TURBO_ECO = 33,
  IDX_TX_QUIET = 35,
  IDX_TX_LED = 36,
  IDX_TX_HEAT_8C = 37,

  // Status-frame indexes.
  IDX_RX_SWING_TURBO_ECO = 35,
  IDX_RX_QUIET = 36,
  IDX_RX_LED = 37,

  // 8 °C frost-protection status. The stock firmware exposes the primary
  // t_8heat flag at byte 77 bit 0. Some revisions additionally mirror the
  // active mode at byte 66 bit 7.
  IDX_RX_HEAT_8C_COMPANION = 66,
  IDX_RX_HEAT_8C = 77,

  // Compressor frequency fields confirmed from transition logs:
  // 41 = measured/actual, 42 = controller target, 43 = command sent to inverter.
  IDX_COMP_FREQ_ACTUAL = 41,
  IDX_COMP_FREQ_SET = 42,
  IDX_COMP_FREQ_COMMAND = 43,
  IDX_OUTDOOR_TEMP = 44,
  IDX_OUTDOOR_COND_TEMP = 45,
  IDX_COMPRESSOR_EXHAUST_TEMP = 46,
};

// Command fields in the 0x65 write frame. A zero byte means "do not change"
// for the corresponding AC setting. Normal Home Assistant commands therefore
// queue only the fields that actually changed instead of retransmitting a full
// climate state and accidentally clearing action-style modes such as Sleep.
enum CommandFieldMask : uint16_t {
  CMD_FIELD_NONE       = 0,
  CMD_FIELD_WIND       = 1u << 0,
  CMD_FIELD_SLEEP      = 1u << 1,
  CMD_FIELD_POWER_MODE = 1u << 2,
  CMD_FIELD_TEMP       = 1u << 3,
  CMD_FIELD_SWING      = 1u << 4,
  CMD_FIELD_TURBO_ECO  = 1u << 5,
  CMD_FIELD_QUIET      = 1u << 6,
  CMD_FIELD_LED        = 1u << 7,
  CMD_FIELD_HEAT_8C    = 1u << 8,
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
  constexpr uint8_t HEAT_8C_ON  = 0x03;
  constexpr uint8_t HEAT_8C_OFF = 0x01;
}

// Limits for non‑blocking operation
static constexpr uint8_t  MAX_FRAMES_PER_LOOP = 2;
static constexpr uint32_t MAX_PARSE_TIME_MS   = 20;
static constexpr size_t   RX_COMPACT_THRESHOLD = 512;
static constexpr size_t   RX_BUFFER_RESERVE    = 2048;
// Status responses are substantially longer than control frames. This indoor
// unit reports byte[4] = 0x8D, i.e. 0x8D + 9 = 150 logical bytes. Other
// firmware variants are known to use similarly long responses, so keep a
// conservative limit that still protects the RX parser from corrupt lengths.
static constexpr size_t   MAX_FRAME_BYTES      = 256;  // logical (unescaped) frame size
static constexpr size_t   MAX_WIRE_FRAME_BYTES = MAX_FRAME_BYTES * 2;
static constexpr uint32_t WRITE_LOCK_TIMEOUT   = 5000;   // ms
static constexpr uint32_t STATUS_QUERY_TIMEOUT = 1500;   // ms; prevents 0x65/0x66 overlap
static constexpr uint8_t  CAPABILITIES_MAX_ATTEMPTS = 3;
static constexpr uint32_t CAPABILITIES_RETRY_MS = 10000;
static constexpr uint32_t CONTROL_DEBOUNCE_MS  = 200;    // ms
static constexpr uint32_t MEM_PUBLISH_INTERVAL_MS = 5000; // for memory diagnostics
static constexpr uint32_t TIMER_REPEAT_EVENT_WINDOW_MS = 15000;
// Active timer announcements are repeated by the indoor unit. If ordinary
// status replies keep arriving with 00/00 for longer than this interval, the
// timer was cancelled from the physical remote (or expired in the unit).
static constexpr uint32_t TIMER_STATUS_REFRESH_TIMEOUT_MS = 75000;
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
  void set_sleep_program_select(ACHISleepProgramSelect *s) {
    sleep_program_select_ = s;
    if (sleep_program_select_ != nullptr) sleep_program_select_->set_parent(this);
  }
  void set_sleep_program(const std::string &value);
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
  void set_compr_freq_actual_sensor(sensor::Sensor *s) { compressor_freq_actual_sensor_ = s; }
  void set_compr_freq_set_sensor(sensor::Sensor *s) { compressor_freq_set_sensor_ = s; }
  void set_compr_freq_command_sensor(sensor::Sensor *s) { compressor_freq_command_sensor_ = s; }
  // Legacy YAML key `compressor_frequency`; retained as an alias for byte 43.
  void set_compr_freq_sensor(sensor::Sensor *s) { compressor_freq_sensor_ = s; }
  void set_outdoor_temp_sensor(sensor::Sensor *s) { outdoor_temp_sensor_ = s; }
  void set_outdoor_cond_temp_sensor(sensor::Sensor *s) { outdoor_cond_temp_sensor_ = s; }
  void set_compressor_exhaust_temp_sensor(sensor::Sensor *s) { compressor_exhaust_temp_sensor_ = s; }
  void set_indoor_humidity_setting_sensor(sensor::Sensor *s) { indoor_humidity_setting_sensor_ = s; }
  void set_indoor_humidity_sensor(sensor::Sensor *s) { indoor_humidity_sensor_ = s; }
  void set_power_on_timer_remaining_sensor(sensor::Sensor *s) { power_on_timer_remaining_sensor_ = s; }
  void set_power_off_timer_remaining_sensor(sensor::Sensor *s) { power_off_timer_remaining_sensor_ = s; }

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

#ifdef USE_BINARY_SENSOR
  void set_ac_fault_binary(binary_sensor::BinarySensor *b) { ac_fault_binary_ = b; }
  void set_power_on_timer_active_binary(binary_sensor::BinarySensor *b) { power_on_timer_active_binary_ = b; }
  void set_power_off_timer_active_binary(binary_sensor::BinarySensor *b) { power_off_timer_active_binary_ = b; }
#endif
#ifdef USE_TEXT_SENSOR
  void set_power_status_text(text_sensor::TextSensor *t) { power_status_text_ = t; }
  void set_device_capabilities_text(text_sensor::TextSensor *t) { device_capabilities_text_ = t; }
  void set_ac_active_faults_text(text_sensor::TextSensor *t) { ac_active_faults_text_ = t; }
  void set_power_on_timer_text(text_sensor::TextSensor *t) { power_on_timer_text_ = t; }
  void set_power_off_timer_text(text_sensor::TextSensor *t) { power_off_timer_text_ = t; }
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
  void send_query_capabilities_();
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
  std::vector<uint8_t> encode_wire_frame_(const std::vector<uint8_t> &logical) const;
  void send_logical_frame_(const std::vector<uint8_t> &logical, const char *log_prefix);

  // RX parser
  void try_parse_frames_from_buffer_(uint32_t budget_ms = MAX_PARSE_TIME_MS);
  bool extract_next_frame_(std::vector<uint8_t> &frame);
  void handle_frame_(const std::vector<uint8_t> &frame);
  void parse_status_102_(const std::vector<uint8_t> &b);
  void parse_capabilities_102_64_(const std::vector<uint8_t> &b);
  void apply_capability_availability_();
  void publish_fault_state_(const std::vector<uint8_t> &b);
  void parse_timer_status_(const std::vector<uint8_t> &b);
  void process_timer_event_(ACHITimerState &state, const char *name, uint8_t raw_hour,
                            uint8_t raw_minute_status);
  void publish_timer_state_(ACHITimerState &state, bool power_on_timer);
  void clear_timer_after_silence_(ACHITimerState &state, const char *name,
                                  uint8_t raw_hour, uint8_t raw_minute_status);
  void update_timer_countdowns_();
  void handle_ack_101_();

  // State management
  void build_tx_from_pending_fields_(uint16_t fields);
  void queue_retry_fields_from_state_();
  void publish_gated_state_();
  void update_led_switch_state_();
  void update_sound_switch_state_();
  void update_memory_switch_state_();
  void update_sleep_program_select_state_();
  void publish_fan_state_(bool turbo_fan, climate::ClimateFanMode fan);
#ifdef USE_SENSOR
  void publish_sensor_if_changed_(sensor::Sensor *sensor, float value);
#endif
#ifdef USE_TEXT_SENSOR
  void publish_text_sensor_if_changed_(text_sensor::TextSensor *sensor, const char *value);
#endif
  void remember_target_for_mode_(climate::ClimateMode mode, uint8_t target_c);
  uint8_t target_for_mode_(climate::ClimateMode mode, uint8_t fallback) const;
  void maybe_force_to_target_();                     // <-- добавлено объявление
  void maybe_send_pending_control_();                // (опционально, если используется)

  // Signatures for convergence detection
  uint32_t compute_control_signature_(bool power, climate::ClimateMode mode,
                                      climate::ClimateFanMode fan, bool fan_turbo,
                                      climate::ClimateSwingMode swing,
                                      bool eco, bool turbo, bool quiet, bool heat_8c, bool led,
                                      uint8_t sleep_stage, uint8_t target_c) const;
  void recalc_desired_sig_();
  void recalc_actual_sig_();
  void log_sig_diff_() const;

  // Diagnostics
  void publish_memory_diagnostics_();

  // Field encoders (TX)
  uint8_t encode_temp_(uint8_t c) const;
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

  // A status request and a write command must never share the RS-485 bus window.
  // While waiting for the 0x66 response, debounced 0x65 writes remain pending.
  bool status_query_in_flight_{false};
  uint32_t status_query_time_{0};
  QueryKind query_kind_{QUERY_NONE};

  // Read-only 0x66/subtype 0x40 ProductType discovery. Existing controls are
  // intentionally not hidden or gated until this unit's reply is confirmed.
  ACHIDeviceCapabilities capabilities_{};
  uint8_t capabilities_attempts_{0};
  uint32_t capabilities_next_attempt_ms_{0};
#ifdef USE_API
  bool capability_api_refresh_pending_{false};
  uint32_t capability_api_refresh_at_ms_{0};
#endif

  // Pending control from HA (debounced). pending_command_fields_ contains
  // exactly the action fields that will be written; all other 0x65 payload
  // bytes stay zero/neutral.
  bool pending_control_{false};
  uint16_t pending_command_fields_{CMD_FIELD_NONE};
  uint32_t last_control_ms_{0};

  // True only after a UART write containing desired Sleep was actually sent.
  // The next status frame must report Sleep Mode Code > 0; otherwise the HA
  // Sleep preset is cleared and no automatic retry is performed.
  bool sleep_confirmation_pending_{false};
  uint8_t sleep_confirmation_target_stage_{0};

  // Fan state that was active immediately before an HA Sleep request. If the
  // indoor unit rejects Sleep (Sleep Mode Code remains 0), restore this fan
  // state with one silent normal command. The snapshot is retained through the
  // confirmed Sleep session and consumed when Sleep is disabled.
  bool sleep_restore_fan_valid_{false};
  climate::ClimateFanMode sleep_restore_fan_{climate::CLIMATE_FAN_AUTO};
  bool sleep_restore_fan_turbo_{false};
  bool sleep_restore_quiet_{false};

  // Display preference that was active immediately before an HA Sleep request.
  // Sleep itself temporarily turns the panel off; that temporary status must not
  // be converted into a delayed LED_OFF action when Sleep is later disabled.
  bool sleep_restore_led_valid_{false};
  bool sleep_restore_led_{true};
  bool sleep_led_restore_pending_{false};
  uint32_t sleep_led_restore_started_ms_{0};

  // True only after the user explicitly changes the fan while a confirmed
  // Sleep program is active. Without this flag, Sleep's automatic QUIET fan is
  // authoritative and must not be overwritten by stale desired HA fan state.
  bool sleep_fan_override_pending_{false};

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

  // State restored when the +8 °C frost-protection preset is disabled.
  bool heat_8c_restore_valid_{false};
  bool heat_8c_restore_power_{false};
  uint8_t heat_8c_restore_target_c_{24};
  climate::ClimateMode heat_8c_restore_mode_{climate::CLIMATE_MODE_HEAT};
  climate::ClimateFanMode heat_8c_restore_fan_{climate::CLIMATE_FAN_AUTO};
  bool heat_8c_restore_fan_turbo_{false};

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
  uint8_t target_c_{24};                // 16..30 °C (ESPHome internal unit)
  bool temp_unit_f_{false};              // display/wire unit reported in status byte 26 bit 1
  bool temp_unit_known_{false};
  climate::ClimateMode mode_{climate::CLIMATE_MODE_OFF};
  // Raw upper-nibble status value. SMART/AUTO uses 4/5/6/7 to expose the
  // internally selected idle/fan, heat, cool and dehumidification branches.
  uint8_t raw_mode_code_{0};
  climate::ClimateFanMode fan_{climate::CLIMATE_FAN_AUTO};
  bool fan_turbo_{false};
  climate::ClimateSwingMode swing_{climate::CLIMATE_SWING_OFF};
  bool turbo_{false};
  bool eco_{false};
  bool quiet_{false};
  bool heat_8c_{false};
  bool led_{true};
  uint8_t sleep_stage_{0};              // 0..4
  uint8_t selected_sleep_stage_{2};     // default program used by HA Sleep preset

  // ----- Desired (from HA) state -----
  bool d_power_on_{false};
  uint8_t d_target_c_{24};
  climate::ClimateMode d_mode_{climate::CLIMATE_MODE_OFF};
  climate::ClimateMode last_active_mode_{climate::CLIMATE_MODE_COOL};
  uint8_t last_cool_target_c_{24};
  uint8_t last_heat_target_c_{24};
  climate::ClimateFanMode d_fan_{climate::CLIMATE_FAN_AUTO};
  bool d_fan_turbo_{false};
  climate::ClimateSwingMode d_swing_{climate::CLIMATE_SWING_OFF};
  bool d_turbo_{false};
  bool d_eco_{false};
  bool d_quiet_{false};
  bool d_heat_8c_{false};
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
  sensor::Sensor *compressor_freq_actual_sensor_{nullptr};
  sensor::Sensor *compressor_freq_set_sensor_{nullptr};
  sensor::Sensor *compressor_freq_command_sensor_{nullptr};
  // Legacy byte-43 sensor configured through `compressor_frequency`.
  sensor::Sensor *compressor_freq_sensor_{nullptr};
  sensor::Sensor *outdoor_temp_sensor_{nullptr};
  sensor::Sensor *outdoor_cond_temp_sensor_{nullptr};
  sensor::Sensor *compressor_exhaust_temp_sensor_{nullptr};
  sensor::Sensor *indoor_humidity_setting_sensor_{nullptr};
  sensor::Sensor *indoor_humidity_sensor_{nullptr};
  sensor::Sensor *power_on_timer_remaining_sensor_{nullptr};
  sensor::Sensor *power_off_timer_remaining_sensor_{nullptr};

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
  text_sensor::TextSensor *device_capabilities_text_{nullptr};
  text_sensor::TextSensor *ac_active_faults_text_{nullptr};
  text_sensor::TextSensor *power_on_timer_text_{nullptr};
  text_sensor::TextSensor *power_off_timer_text_{nullptr};
#endif
#ifdef USE_BINARY_SENSOR
  binary_sensor::BinarySensor *ac_fault_binary_{nullptr};
  binary_sensor::BinarySensor *power_on_timer_active_binary_{nullptr};
  binary_sensor::BinarySensor *power_off_timer_active_binary_{nullptr};
#endif

  bool fault_state_valid_{false};
  bool last_fault_any_{false};
  uint32_t last_fault_signature_{0};

  ACHITimerState power_on_timer_state_{};
  ACHITimerState power_off_timer_state_{};

  ACHILEDTargetSwitch *led_switch_{nullptr};
  ACHICommandSoundSwitch *sound_switch_{nullptr};
  ACHIMemorySwitch *memory_switch_{nullptr};
  ACHISleepProgramSelect *sleep_program_select_{nullptr};

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