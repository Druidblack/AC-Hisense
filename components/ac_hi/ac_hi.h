#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/uart/uart.h"

#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/number/number.h"
#include "esphome/components/select/select.h"

#include <vector>

namespace esphome {
namespace ac_hi {

static const char *const TAG = "ac_hi";

enum class State : uint8_t {
  IDLE,
  QUERY_SENT,
  WAIT_STATUS,
  READY_WRITE,
  WRITE_SENT,
  WAIT_ACK
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

// Forward
class ACHIComponent;

/** Generic Switch that delegates to parent */
class ACHISwitch : public switch_::Switch {
 public:
  void set_parent(ACHIComponent *p) { parent_ = p; }
  void set_type(ControlType t) { type_ = t; }
 protected:
  void write_state(bool state) override;
  ACHIComponent *parent_ {nullptr};
  ControlType type_{CTRL_POWER};
};

/** Number for setpoint */
class ACHINumber : public number::Number {
 public:
  void set_parent(ACHIComponent *p) { parent_ = p; }
  void set_type(ControlType t) { type_ = t; }
 protected:
  void control(float value) override;
  ACHIComponent *parent_{nullptr};
  ControlType type_{CTRL_TEMP};
};

/** Selects (mode/wind/sleep) */
class ACHISelect : public select::Select {
 public:
  void set_parent(ACHIComponent *p) { parent_ = p; }
  void set_type(ControlType t) {
    type_ = t;
    // set options (traits)
    if (t == CTRL_MODE) {
      traits_.set_options({"fan_only","heat","cool","dry","auto"});
    } else if (t == CTRL_WIND) {
      traits_.set_options({"off","auto","lowest","low","medium","high","highest"});
    } else if (t == CTRL_SLEEP) {
      traits_.set_options({"off","sleep_1","sleep_2","sleep_3","sleep_4"});
    }
  }
  select::SelectTraits get_traits() override { return traits_; }
 protected:
  void control(const std::string &value) override;
  ACHIComponent *parent_{nullptr};
  ControlType type_{CTRL_MODE};
  select::SelectTraits traits_;
};

class ACHIComponent : public PollingComponent, public uart::UARTDevice {
 public:
  explicit ACHIComponent() {}

  // ----- wiring from codegen -----
  void set_entities(
    text_sensor::TextSensor *power_text,
    sensor::Sensor *wind_s, sensor::Sensor *sleep_s, sensor::Sensor *mode_s,
    sensor::Sensor *t_set, sensor::Sensor *t_cur, sensor::Sensor *t_pipe,
    sensor::Sensor *quiet_s, sensor::Sensor *turbo_s, sensor::Sensor *led_s, sensor::Sensor *eco_s,
    sensor::Sensor *lr_s, sensor::Sensor *ud_s,
    ACHISwitch *power_sw, ACHISwitch *quiet_sw, ACHISwitch *turbo_sw, ACHISwitch *led_sw, ACHISwitch *eco_sw, ACHISwitch *ud_sw, ACHISwitch *lr_sw,
    ACHINumber *temp_num,
    ACHISelect *mode_sel, ACHISelect *wind_sel, ACHISelect *sleep_sel
  ) {
    power_text_ = power_text;
    wind_s_ = wind_s; sleep_s_ = sleep_s; mode_s_ = mode_s;
    t_set_ = t_set; t_cur_ = t_cur; t_pipe_ = t_pipe;
    quiet_s_ = quiet_s; turbo_s_ = turbo_s; led_s_ = led_s; eco_s_ = eco_s;
    lr_s_ = lr_s; ud_s_ = ud_s;

    power_sw_ = power_sw; quiet_sw_ = quiet_sw; turbo_sw_ = turbo_sw;
    led_sw_ = led_sw; eco_sw_ = eco_sw; ud_sw_ = ud_sw; lr_sw_ = lr_sw;

    temp_num_ = temp_num;

    mode_sel_ = mode_sel; wind_sel_ = wind_sel; sleep_sel_ = sleep_sel;

    // back-links for controls done in Python
  }

  // ----- lifecycle -----
  void setup() override;
  void loop() override;
  void update() override;

  // ----- control entry points from child entities -----
  void on_switch(ControlType t, bool state);
  void on_number(ControlType t, float value);
  void on_select(ControlType t, const std::string &value);

 protected:
  // ----- protocol helpers -----
  void send_query_();          // short request to read status
  void handle_frame_(const std::vector<uint8_t> &bytes);
  void schedule_write_();      // start/extend sliding window
  void maybe_send_write_();    // when window elapsed, build & send
  void build_write_frame_();   // merge fields and recalc CRC

  void publish_states_from_102_(const std::vector<uint8_t> &bytes);
  void unlock_on_101_(const std::vector<uint8_t> &bytes);

  // CRC: sum bytes [2..len-5], place at [46]=hi, [47]=lo
  static void put_crc_(std::vector<uint8_t> &buf);

  // ----- state -----
  State state_{State::IDLE};

  // RX framing
  std::vector<uint8_t> rx_;
  uint32_t last_rx_ms_{0};

  // Sliding write window
  bool write_changes_{false};
  bool lock_update_{false};
  uint32_t write_deadline_ms_{0};

  // cached last status crc to avoid noisy updates
  int last_status_crc_{-1};

  // decode tables (as in YAML)
  const char* decode_mode_[8] = { "fan_only","heat","cool","dry","auto","auto","auto","auto" };
  const char* decode_wind_[19] = { "off","auto","auto","","","","","","","", "lowest","","low","","medium","","high","","highest" };
  const char* decode_sleep_[5] = { "off","sleep_1","sleep_2","sleep_3","sleep_4" };

  // outgoing long frame template (from YAML)
  std::vector<uint8_t> frame_ = {0xF4,0xF5,0x00,0x40,0x29,0x00,0x00,0x01,0x01,0xFE,0x01,0x00,0x00,0x65,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF4,0xFB};

  // pending bit fields (same semantics as YAML)
  uint8_t power_bin_{0};
  uint8_t mode_bin_{0};
  uint8_t updown_bin_{0};
  uint8_t leftright_bin_{0};
  uint8_t turbo_bin_{0};
  uint8_t eco_bin_{0};
  uint8_t quiet_bin_{0};

  // current state (mirrors YAML globals)
  bool current_power_{false};
  uint8_t current_set_temp_{0};
  std::string current_ac_mode_{};
  std::string current_wind_{};
  std::string current_sleep_{};

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