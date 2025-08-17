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

// Hisense/BECO legacy protocol over UART/RS485.
// Header: F4 F5, Tail: F4 FB
// Короткий запрос STATUS — 21 байт с SUM8, длинные ответы STATUS у некоторых плат приходят с cmd=0x65.

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

  static constexpr uint8_t CMD_WRITE  = 0x65;  // write / ack / иногда длинный status
  static constexpr uint8_t CMD_STATUS = 0x66;  // короткий/длинный status на некоторых моделях
  static constexpr uint8_t CMD_NAK    = 0xFD;

  // Полезные индексы в длинных кадрах (совпадают с ранними логами: b18, b19, ...)
  static constexpr int IDX_FAN          = 16;
  static constexpr int IDX_SLEEP        = 17;
  static constexpr int IDX_MODE_POWER   = 18;  // hi: режим, lo: питание (0x0C=ON)
  static constexpr int IDX_TARGET_TEMP  = 19;  // 2*C+1
  static constexpr int IDX_AIR_TEMP     = 20;  // int8
  static constexpr int IDX_PIPE_TEMP    = 21;  // int8
  static constexpr int IDX_SWING        = 32;  // 0x50 = UD swing ON
  static constexpr int IDX_FLAGS        = 33;  // turbo/eco
  static constexpr int IDX_FLAGS2       = 35;  // quiet + LR swing bit (0x20)
  static constexpr int IDX_LED          = 36;  // бит 0x40

  // Желаемое состояние (для построения WRITE)
  bool power_on_{false};
  climate::ClimateMode mode_desired_{climate::CLIMATE_MODE_COOL};
  uint8_t target_c_{24};
  climate::ClimateFanMode fan_desired_{climate::CLIMATE_FAN_AUTO};
  climate::ClimateSwingMode swing_desired_{climate::CLIMATE_SWING_OFF};
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

  // Pending snapshot для implicit-ACK через сравнение со STATUS
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
  void parse_status_legacy_(const std::vector<uint8_t> &b);
  void handle_ack_short_();
  void handle_nak_fd_();

  // Legacy builders
  void build_legacy_write_(std::vector<uint8_t> &frame);
  void build_legacy_query_status_(std::vector<uint8_t> &frame);

  // Checksums
  static void crc16_sum_legacy_patch(std::vector<uint8_t> &buf); // WRITE (HI, LO) в [46],[47]
  static void crc8_sum_legacy_patch(std::vector<uint8_t> &buf);  // короткий STATUS: байт в [18]

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
