#pragma once

#include "esphome/core/component.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"
#include "esphome/components/uart/uart.h"

#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/number/number.h"
#include "esphome/components/select/select.h"

#include <vector>
#include <string>

namespace esphome {
namespace ac_hi {

static const char *const TAG = "ac_hi";

enum class State : uint8_t {
  IDLE,
  QUERY_SENT,
  WAIT_STATUS,
  WRITE_SENT,
};

enum ControlType : uint8_t {
  CTRL_POWER,
  CTRL_QUIET,
  CTRL_TURBO,
  CTRL_LED,
  CTRL_ECO,
  CTRL_UPDOWN,
  CTRL_LEFTRIGHT,
  CTRL_TEMP,
  CTRL_MODE,
  CTRL_WIND,
  CTRL_SLEEP
};

class ACHIComponent;

// ------ Child controls ------
class ACHISwitch : public switch_::Switch {
 public:
  void set_parent(ACHIComponent *p) { parent_ = p; }
  void set_type(ControlType t) { type_ = t; }
 protected:
  void write_state(bool state) override;
  ACHIComponent *parent_{nullptr};
  ControlType type_{CTRL_POWER};
};

class ACHINumber : public number::Number {
 public:
  void set_parent(ACHIComponent *p) { parent_ = p; }
  void set_type(ControlType t) { type_ = t; }
 protected:
  void control(float value) override;
  ACHIComponent *parent_{nullptr};
  ControlType type_{CTRL_TEMP};
};

class ACHISelect : public select::Select {
 public:
  void set_parent(ACHIComponent *p) { parent_ = p; }
  void set_type(ControlType t) { type_ = t; }
 protected:
  void control(const std::string &value) override;
  ACHIComponent *parent_{nullptr};
  ControlType type_{CTRL_MODE};
};

// ------ Main component ------
class ACHIComponent : public PollingComponent, public uart::UARTDevice {
 public:
  ACHIComponent() = default;

  // lifecycle
  void setup() override;
  void loop() override;
  void update() override;

  // configuration from python
  void set_name_prefix(const std::string &p) { name_prefix_ = p; }

  // control entry points
  void on_switch(ControlType t, bool state);
  void on_number(ControlType t, float value);
  void on_select(ControlType t, const std::string &value);

 protected:
  // protocol helpers
  void send_query_();
  void handle_frame_(const std::vector<uint8_t> &bytes);
  void schedule_write_();
  void maybe_send_write_();
  void build_write_frame_();
  void publish_states_from_102_(const std::vector<uint8_t> &bytes);
  void unlock_on_101_(const std::vector<uint8_t> &bytes);
  static void put_crc_(std::vector<uint8_t> &buf);

  // fsm & timers
  State state_{State::IDLE};
  std::vector<uint8_t> rx_;
  bool write_changes_{false};
  bool lock_update_{false};
  uint32_t write_deadline_ms_{0};
  int last_status_crc_{-1};

  // перенос публикации в update()
  std::vector<uint8_t> pending_status_;
  bool has_pending_status_{false};

  // encoding tables
  const char* decode_mode_[8] = { "fan_only","heat","cool","dry","auto","auto","auto","auto" };
  const char* decode_wind_[19] = { "off","auto","auto","","","","","","","", "lowest","","low","","medium","","high","","highest" };
  const char* decode_sleep_[5] = { "off","sleep_1","sleep_2","sleep_3","sleep_4" };

  // outgoing frame template
  std::vector<uint8_t> frame_ = {0xF4,0xF5,0x00,0x40,0x29,0x00,0x00,0x01,0x01,0xFE,0x01,0x00,0x00,0x65,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF4,0xFB};

  // pending bit fields
  uint8_t power_bin_{0};
  uint8_t mode_bin_{0};
  uint8_t updown_bin_{0};
  uint8_t leftright_bin_{0};
  uint8_t turbo_bin_{0};
  uint8_t eco_bin_{0};
  uint8_t quiet_bin_{0};

  // current state mirrors
  bool current_power_{false};
  uint8_t current_set_temp_{0};
  std::string current_ac_mode_{};
  std::string current_wind_{};
  std::string current_sleep_{};

  // name prefix
  std::string name_prefix_{"Hisense AC"};

  // children
  text_sensor::TextSensor *power_text_{nullptr};

  sensor::Sensor *wind_s_{nullptr};
  sensor::Sensor *sleep_s_{nullptr};
  sensor::Sensor *mode_s_{nullptr};

  sensor::Sensor *t_set_{nullptr};
  sensor::Sensor *t_cur_{nullptr};
  sensor::Sensor *t_pipe_{nullptr};

  sensor::Sensor *quiet_s_{nullptr};
  sensor::Sensor *turbo_s_{nullptr};
  sensor::Sensor *led_s_{nullptr};
  sensor::Sensor *eco_s_{nullptr};

  sensor::Sensor *lr_s_{nullptr};
  sensor::Sensor *ud_s_{nullptr};

  ACHISwitch *power_sw_{nullptr};
  ACHISwitch *quiet_sw_{nullptr};
  ACHISwitch *turbo_sw_{nullptr};
  ACHISwitch *led_sw_{nullptr};
  ACHISwitch *eco_sw_{nullptr};
  ACHISwitch *ud_sw_{nullptr};
  ACHISwitch *lr_sw_{nullptr};

  ACHINumber *temp_num_{nullptr};

  ACHISelect *mode_sel_{nullptr};
  ACHISelect *wind_sel_{nullptr};
  ACHISelect *sleep_sel_{nullptr};
};

}  // namespace ac_hi
}  // namespace esphome
