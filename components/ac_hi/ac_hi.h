#pragma once

#include "esphome/components/climate/climate.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "esphome/core/log.h"

// Подключаем заголовок сенсоров только если платформа sensor реально присутствует в билде
#ifdef USE_SENSOR
  #include "esphome/components/sensor/sensor.h"
#endif

#include <vector>

// Форвард-декларация класса Sensor на случай, если USE_SENSOR не определён
namespace esphome {
namespace sensor {
class Sensor;
}  // namespace sensor
}  // namespace esphome

namespace esphome {
namespace ac_hi {

// Encodings from hisense.yaml
// Byte map (0-based):
// 16 - fan speed
// 17 - sleep
// 18 - mode (hi nibble) + power/toggle bits (lo nibble)
// 19 - target temp encoded (2*T + 1)
// 32 - swing: up-down (bits 7:6), left-right (bits 5:4)
// 33 - turbo/eco (writer), фактические флаги в статусе читаем с 35/36
// 35 - quiet (bits 5:4), + флаги swing read
// 36 - LED (bits 7:6) и quiet-флаг при чтении
// 20 - current temp, 21 - pipe temp (read-only)

class ACHIClimate : public climate::Climate, public PollingComponent, public uart::UARTDevice {
 public:
  ACHIClimate() = default;

  // Config
  void set_enable_presets(bool v) { enable_presets_ = v; }
  void set_pipe_sensor(sensor::Sensor *s) { pipe_sensor_ = s; }

  void setup() override;
  void loop() override;
  void update() override;

  // Climate API
  void control(const climate::ClimateCall &call) override;
  climate::ClimateTraits traits() override;

  // UART: метод set_uart_parent(...) унаследован из uart::UARTDevice

 protected:
  // Protocol helpers
  void send_query_status_();    // short request (cmd 0x66)
  void send_write_changes_();   // full packet with CRC
  void calc_and_patch_crc_(std::vector<uint8_t> &buf);
  void handle_frame_(const std::vector<uint8_t> &frame);
  void parse_status_102_(const std::vector<uint8_t> &b);
  void handle_ack_101_();

  // Buffering
  std::vector<uint8_t> rx_;
  bool writing_lock_{false};
  bool pending_write_{false};
  uint32_t last_rx_ms_{0};

  // Cached "bytearray" to be sent (из YAML initial vector)
  std::vector<uint8_t> tx_bytes_ = {
      0xF4, 0xF5, 0x00, 0x40, 0x29, 0x00, 0x00, 0x01, 0x01, 0xFE, 0x01, 0x00, 0x00,
      0x65, 0x00, 0x00, 0x00, // 0..16
      0x00, // [17]
      0x00, // [18] power+mode
      0x00, // [19] temp
      0x00, // [20] cur temp (filled by AC)
      0x00, // [21] pipe temp
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
      0x00, 0x00, // 46..47 CRC
      0xF4, 0xFB    // tail
  };

  // Command bytes for query (cmd 0x66)
  const std::vector<uint8_t> query_ = {0xF4, 0xF5, 0x00, 0x40, 0x0C, 0x00, 0x00, 0x01, 0x01,
                                       0xFE, 0x01, 0x00, 0x00, 0x66, 0x00, 0x00, 0x00, 0x01,
                                       0xB3, 0xF4, 0xFB};

  // State mirrors
  bool power_on_{false};
  uint8_t target_c_{24}; // 18..28
  climate::ClimateMode mode_{climate::CLIMATE_MODE_OFF};
  climate::ClimateFanMode fan_{climate::CLIMATE_FAN_AUTO};
  climate::ClimateSwingMode swing_{climate::CLIMATE_SWING_OFF};
  bool turbo_{false};
  bool eco_{false};
  bool quiet_{false};
  bool led_{true};
  uint8_t sleep_stage_{0}; // 0..4

  // CRC cache
  uint16_t last_status_crc_{0};

  // Helpers mapping
  uint8_t encode_temp_(uint8_t c) { return static_cast<uint8_t>((c << 1) | 0x01); }
  uint8_t encode_mode_hi_nibble_(climate::ClimateMode m);
  uint8_t encode_fan_byte_(climate::ClimateFanMode f);
  uint8_t encode_sleep_byte_(uint8_t stage);
  uint8_t encode_swing_ud_(bool on);
  uint8_t encode_swing_lr_(bool on);

  // Sensors (указатель может быть нулевым)
  sensor::Sensor *pipe_sensor_{nullptr};

  // Flags
  bool enable_presets_{true};
};

}  // namespace ac_hi
}  // namespace esphome
