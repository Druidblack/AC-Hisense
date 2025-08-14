#pragma once

#include <string>
#include <vector>
#include <cstdint>

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/number/number.h"
#include "esphome/components/select/select.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/climate/climate.h"

namespace esphome {
namespace ac_hi {

class ACHi;

class ACHi : public PollingComponent, public uart::UARTDevice {
 public:
  ACHi(uart::UARTComponent *parent);

  void setup() override;
  void loop() override;
  void update() override;

  // Control methods
  void set_power(bool power);
  void set_temperature(float temperature);
  void set_mode(const std::string &mode);
  void set_fan_speed(const std::string &speed);
  void set_sleep_mode(const std::string &sleep_mode);
  void set_quiet_mode(bool quiet);
  void set_turbo_mode(bool turbo);
  void set_eco_mode(bool eco);
  void set_led(bool led);
  void set_swing_up_down(bool swing);
  void set_swing_left_right(bool swing);

    // Sensor and other methods

  void set_mode_select(select::Select *s) { ac_mode_select = s; }
  void set_wind_select(select::Select *s) { ac_wind_select = s; }
  void set_sleep_select(select::Select *s) { ac_sleep_select = s; }
  void set_temperature_number(number::Number *n) { my_temperature = n; }
  void set_power_switch(switch_::Switch *s) { power_switch = s; }
  void set_quiet_switch(switch_::Switch *s) { quiet_switch = s; }
  void set_turbo_switch(switch_::Switch *s) { turbo_switch = s; }
  void set_eco_switch(switch_::Switch *s) { eco_switch = s; }
  void set_led_switch(switch_::Switch *s) { led_switch = s; }
  void set_swing_up_down_switch(switch_::Switch *s) { swing_up_down_switch = s; }
  void set_swing_left_right_switch(switch_::Switch *s) { swing_left_right_switch = s; }

  void set_compr_freq_sensor(sensor::Sensor *s) { compr_freq = s; }
  void set_compr_freq_set_sensor(sensor::Sensor *s) { compr_freq_set = s; }
  void set_temp_current_sensor(sensor::Sensor *s) { temp_current = s; }
  void set_temp_outdoor_sensor(sensor::Sensor *s) { temp_outdoor = s; }
  void set_temp_outdoor_condenser_sensor(sensor::Sensor *s) { temp_outdoor_condenser = s; }
  void set_temp_pipe_current_sensor(sensor::Sensor *s) { temp_pipe_current = s; }
  void set_temp_set_sensor(sensor::Sensor *s) { temp_set = s; }
  void set_sensor_eco_sensor(sensor::Sensor *s) { sensor_eco = s; }
  void set_sensor_led_sensor(sensor::Sensor *s) { sensor_led = s; }
  void set_sensor_mode_sensor(sensor::Sensor *s) { sensor_mode = s; }
  void set_power_status_sensor(text_sensor::TextSensor *s) { power_status = s; }

    // Sensors
  sensor::Sensor *sensor_wind;
  sensor::Sensor *sensor_sleep;
  sensor::Sensor *sensor_mode;
  sensor::Sensor *temp_set;
  sensor::Sensor *temp_current;
  sensor::Sensor *compr_freq_set;
  sensor::Sensor *compr_freq;
  sensor::Sensor *temp_outdoor;
  sensor::Sensor *temp_outdoor_condenser;
  sensor::Sensor *sensor_quiet;
  sensor::Sensor *sensor_turbo;
  sensor::Sensor *sensor_led;
  sensor::Sensor *sensor_eco;
  sensor::Sensor *temp_pipe_current;
  sensor::Sensor *sensor_left_right;
  sensor::Sensor *sensor_up_down;

  // Text Sensors
  text_sensor::TextSensor *power_status;

  // Number (Temperature Control)
  number::Number *my_temperature;

  // Selects
  select::Select *ac_mode_select;
  select::Select *ac_wind_select;
  select::Select *ac_sleep_select;

  // Switches
  switch_::Switch *power_switch;
  switch_::Switch *quiet_switch;
  switch_::Switch *turbo_switch;
  switch_::Switch *eco_switch;
  switch_::Switch *led_switch;
  switch_::Switch *swing_up_down_switch;
  switch_::Switch *swing_left_right_switch;

 private:
  // Variables previously defined as globals
  bool current_power_;
  uint8_t current_set_temp_;
  std::string current_ac_mode_;
  std::string current_wind_;
  std::string current_sleep_;

  bool lock_update_;
  bool write_changes_;

  uint8_t power_bin_;
  uint8_t mode_bin_;
  uint8_t updown_bin_;
  uint8_t leftright_bin_;
  uint8_t turbo_bin_;
  uint8_t eco_bin_;
  uint8_t quiet_bin_;
  uint8_t led_bin_;

  std::vector<uint8_t> bytearray_;

  // Helper methods
  void send_read_command();
  void process_incoming_data(const std::vector<uint8_t> &data);
  void write_changes();
  void schedule_write_changes();

  // Timer for periodic actions
  uint32_t last_write_time_;
  uint32_t last_read_time_;

  // Other necessary variables
  int status_crc_;

  // Decoding arrays
  const std::string decode_acmode_codes_[8] = {"fan_only", "heat", "cool", "dry", "auto", "auto", "auto", "auto"};
  const std::string decode_wind_codes_[19] = {"off", "auto", "auto", "", "", "", "", "", "", "", "lowest", "", "low", "", "medium", "", "high", "", "highest"};
  const std::string decode_sleep_codes_[5] = {"off", "sleep_1", "sleep_2", "sleep_3", "sleep_4"};

  // Encoding arrays
  const uint8_t mode_codes_[5] = {0, 1, 2, 3, 4};
  const uint8_t wind_codes_[7] = {0, 1, 10, 12, 14, 16, 18};
  const uint8_t sleep_codes_[5] = {0, 1, 2, 4, 8};

  // Helper variables
  bool pending_write_;
};

}  // namespace ac_hi
}  // namespace esphome