#pragma once

#include "esphome/components/climate/climate.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#ifdef USE_SENSOR
  #include "esphome/components/sensor/sensor.h"
#endif

#ifdef USE_TEXT_SENSOR
  #include "esphome/components/text_sensor/text_sensor.h"
#endif

#include <vector>
#include <cstddef>
#include <cstdint>
#include <algorithm>

namespace esphome {
namespace ac_hi {

// Framing
static constexpr uint8_t HI_HDR0 = 0xF4;
static constexpr uint8_t HI_HDR1 = 0xF5;
static constexpr uint8_t HI_TAIL0 = 0xF4;
static constexpr uint8_t HI_TAIL1 = 0xFB;

// Limits
static constexpr uint8_t  MAX_FRAMES_PER_LOOP = 2;
static constexpr uint32_t MAX_PARSE_TIME_MS   = 20;
static constexpr size_t   RX_COMPACT_THRESHOLD = 512;

class ACHIClimate : public climate::Climate, public PollingComponent, public uart::UARTDevice {
 public:
  ACHIClimate() = default;

  // Config
  void set_enable_presets(bool v) { enable_presets_ = v; }

#ifdef USE_SENSOR
  void set_pipe_sensor(sensor::Sensor *s) { pipe_sensor_ = s; }
  void set_set_temp_sensor(sensor::Sensor *s) { set_temp_sensor_ = s; }
  void set_room_temp_sensor(sensor::Sensor *s) { room_temp_sensor_ = s; }
  void set_wind_sensor(sensor::Sensor *s) { wind_sensor_ = s; }
  void set_sleep_sensor(sensor::Sensor *s) { sleep_sensor_ = s; }
  void set_mode_sensor(sensor::Sensor *s) { mode_sensor_ = s; }
  void set_quiet_sensor(sensor::Sensor *s) { quiet_sensor_ = s; }
  void set_turbo_sensor(sensor::Sensor *s) { turbo_sensor_ = s; }
  void set_led_sensor(sensor::Sensor *s) { led_sensor_ = s; }
  void set_eco_sensor(sensor::Sensor *s) { eco_sensor_ = s; }
  void set_swing_updown_sensor(sensor::Sensor *s) { swing_updown_sensor_ = s; }
  void set_swing_leftright_sensor(sensor::Sensor *s) { swing_leftright_sensor_ = s; }
  // Outdoor/Compressor
  void set_compr_freq_set_sensor(sensor::Sensor *s) { compr_freq_set_sensor_ = s; }
  void set_compr_freq_sensor(sensor::Sensor *s) { compr_freq_sensor_ = s; }
  void set_outdoor_temp_sensor(sensor::Sensor *s) { outdoor_temp_sensor_ = s; }
  void set_outdoor_cond_temp_sensor(sensor::Sensor *s) { outdoor_cond_temp_sensor_ = s; }
#else
  void set_pipe_sensor(void *) {}
  void set_set_temp_sensor(void *) {}
  void set_room_temp_sensor(void *) {}
  void set_wind_sensor(void *) {}
  void set_sleep_sensor(void *) {}
  void set_mode_sensor(void *) {}
  void set_quiet_sensor(void *) {}
  void set_turbo_sensor(void *) {}
  void set_led_sensor(void *) {}
  void set_eco_sensor(void *) {}
  void set_swing_updown_sensor(void *) {}
  void set_swing_leftright_sensor(void *) {}
  void set_compr_freq_set_sensor(void *) {}
  void set_compr_freq_sensor(void *) {}
  void set_outdoor_temp_sensor(void *) {}
  void set_outdoor_cond_temp_sensor(void *) {}
#endif

#ifdef USE_TEXT_SENSOR
  void set_power_status_text_sensor(text_sensor::TextSensor *s) { power_status_text_sensor_ = s; }
#else
  void set_power_status_text_sensor(void *) {}
#endif

  // Component
  void setup() override;
  void loop() override;
  void update() override;

  // Climate API
  void control(const climate::ClimateCall &call) override;
  climate::ClimateTraits traits() override;

 protected:
  // Protocol helpers
  void send_query_status_();    // cmd 0x66
  void send_write_changes_();   // cmd 0x65 (existing logic preserved)
  void calc_and_patch_crc_(std::vector<uint8_t> &buf);
  bool validate_crc_(const std::vector<uint8_t> &buf, uint16_t *out_sum = nullptr) const;

  // RX framing
  void try_parse_frames_from_buffer_(uint32_t budget_ms = MAX_PARSE_TIME_MS);
  bool extract_next_frame_(std::vector<uint8_t> &frame);
  void handle_frame_(const std::vector<uint8_t> &frame);
  void parse_status_102_(const std::vector<uint8_t> &b);
  void handle_ack_101_();

  // === HA priority/enforce logic (no protocol mapping changes) ===
  struct DesiredState {
    bool power_on;
    climate::ClimateMode mode;
    uint8_t target_c;
    climate::ClimateFanMode fan;
    climate::ClimateSwingMode swing;
    bool turbo;
    bool eco;
    bool quiet;
    bool led;
    uint8_t sleep_stage;
  };

  // Fingerprint helpers (hash only of control fields; no sensors)
  uint32_t make_control_fingerprint_from_fields_(bool power, climate::ClimateMode m, uint8_t t,
                                                 climate::ClimateFanMode f, climate::ClimateSwingMode s,
                                                 bool turbo, bool eco, bool quiet, bool led, uint8_t sleep);
  uint32_t actual_fingerprint_() const;
  uint32_t desired_fingerprint_() const;

  // Debug helpers
  void log_frame_hex_(const char *title, const std::vector<uint8_t> &buf, size_t max_len = 64) const;

  // RX stream buffer
  std::vector<uint8_t> rx_;
  size_t rx_start_{0};

  bool writing_lock_{false};
  bool pending_write_{false};

  // Write/Query templates (kept as in project)
  std::vector<uint8_t> tx_bytes_ = {
      0xF4, 0xF5, 0x00, 0x40, 0x29, 0x00, 0x00, 0x01, 0x01, 0xFE, 0x01, 0x00, 0x00,
      0x65, 0x00, 0x00, 0x00,
      0x00, // [17] sleep
      0x00, // [18] power+mode
      0x18, // [19] set temp (24°C)
      0x00, // [20] room temp (ignored)
      0x00, // [21] pipe temp (ignored)
      // ... payload bytes depend on model
      0x00, 0x00, 0x00, 0x00, // [22..25]
      0x00, 0x00, 0x00, 0x00, // [26..29]
      0x00, 0x00, 0x00, 0x00, // [30..33]
      0x00, 0x00, 0x00, 0x00, // [34..37]
      0x00, 0x00, 0x00, 0x00, // [38..41]
      0x00,                   // [42]
      0x00,                   // [43]
      0x00,                   // [44]
      0x00,                   // [45]
      0x00, 0x00, 0x00,       // [46..48]
      0xF4, 0xFB
  };

  const std::vector<uint8_t> query_ = {
      0xF4, 0xF5, 0x00, 0x40, 0x0C, 0x00, 0x00, 0x01, 0x01,
      0xFE, 0x01, 0x00, 0x00, 0x66, 0x00, 0x00, 0x00, 0x01,
      0xB3, 0xF4, 0xFB
  };

  // Internal state
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
  uint16_t last_status_crc_{0};

  // Encode helpers
  uint8_t encode_temp_(uint8_t c) {
    uint8_t clamped = std::max<uint8_t>(16, std::min<uint8_t>(30, c));
    return static_cast<uint8_t>((clamped << 1) | 0x01);
  }
  uint8_t encode_mode_hi_nibble_(climate::ClimateMode m);
  uint8_t encode_fan_byte_(climate::ClimateFanMode f);
  uint8_t encode_sleep_byte_(uint8_t stage);
  uint8_t encode_swing_ud_(bool on);
  uint8_t encode_swing_lr_(bool on);

  // Sensors
#ifdef USE_SENSOR
  sensor::Sensor *pipe_sensor_{nullptr};
  sensor::Sensor *set_temp_sensor_{nullptr};
  sensor::Sensor *room_temp_sensor_{nullptr};
  sensor::Sensor *wind_sensor_{nullptr};
  sensor::Sensor *sleep_sensor_{nullptr};
  sensor::Sensor *mode_sensor_{nullptr};
  sensor::Sensor *quiet_sensor_{nullptr};
  sensor::Sensor *turbo_sensor_{nullptr};
  sensor::Sensor *led_sensor_{nullptr};
  sensor::Sensor *eco_sensor_{nullptr};
  sensor::Sensor *swing_updown_sensor_{nullptr};
  sensor::Sensor *swing_leftright_sensor_{nullptr};
  // Outdoor / compressor
  sensor::Sensor *compr_freq_set_sensor_{nullptr};
  sensor::Sensor *compr_freq_sensor_{nullptr};
  sensor::Sensor *outdoor_temp_sensor_{nullptr};
  sensor::Sensor *outdoor_cond_temp_sensor_{nullptr};
#endif

#ifdef USE_TEXT_SENSOR
  text_sensor::TextSensor *power_status_text_sensor_{nullptr};
#endif

  bool enable_presets_{true};

  // === HA priority/enforce state ===
  bool enforce_from_ha_{false};     // мы дожимаем состояние до desired_
  bool accept_ir_changes_{true};    // можно ли учитывать изменения из статусов (ИК-пульт)
  DesiredState desired_{};          // целевое состояние из HA
  bool desired_valid_{false};       // есть ли валидная цель

  uint32_t desired_fp_{0};          // контрольные подписи (только управляющие поля)
  uint32_t actual_fp_{0};
  uint32_t last_applied_fp_{0};

  uint32_t next_enforce_tx_at_{0};  // план на следующий повтор команды
  uint32_t enforce_interval_ms_{700};
  uint8_t enforce_backoff_steps_{0};
  uint8_t enforce_retry_counter_{0};
};

}  // namespace ac_hi
}  // namespace esphome
