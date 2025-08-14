#include "ac_hi.h"

namespace esphome {
namespace ac_hi {

using esphome::uart::UARTDevice;

static const uint8_t START0 = 0xF4;
static const uint8_t START1 = 0xF5;
static const uint8_t END0   = 0xF4;
static const uint8_t END1   = 0xFB;

static const uint8_t CMD_IDX = 13;  // per protocol

// ------------ ACHISwitch ------------
void ACHISwitch::write_state(bool state) {
  if (this->parent_ == nullptr) return;
  this->parent_->on_switch(this->type_, state);
}

// ------------ ACHINumber ------------
void ACHINumber::control(float value) {
  if (this->parent_ == nullptr) return;
  this->parent_->on_number(this->type_, value);
}

// ------------ ACHISelect ------------
void ACHISelect::control(const std::string &value) {
  if (this->parent_ == nullptr) return;
  this->parent_->on_select(this->type_, value);
}

// ------------ ACHIComponent ------------

void ACHIComponent::setup() {
  ESP_LOGI(TAG, "AC-Hi init, creating entities");
  rx_.reserve(256);  // уменьшает аллокации и джиттер

  // ----- Text sensor -----
  this->power_text_ = new text_sensor::TextSensor();
  this->power_text_->set_name((this->name_prefix_ + " Power Status").c_str());
  App.register_text_sensor(this->power_text_);
  this->power_text_->publish_state("OFF");

  // ----- Numeric sensors -----
  auto mk_sensor = [this](sensor::Sensor *&ptr, const std::string &name, int8_t acc = 0, const char *uom = nullptr, const char *dev_class = nullptr, sensor::StateClass st = sensor::STATE_CLASS_MEASUREMENT) {
    ptr = new sensor::Sensor();
    ptr->set_name((this->name_prefix_ + " " + name).c_str());
    ptr->set_accuracy_decimals(acc);
    if (uom != nullptr) ptr->set_unit_of_measurement(uom);
    if (dev_class != nullptr) ptr->set_device_class(dev_class);
    ptr->set_state_class(st);
    App.register_sensor(ptr);
  };

  mk_sensor(this->wind_s_, "Wind", 0, nullptr, nullptr, sensor::STATE_CLASS_MEASUREMENT);
  mk_sensor(this->sleep_s_, "Sleep", 0);
  mk_sensor(this->mode_s_, "Mode", 0);

  mk_sensor(this->t_set_, "Temperature Set", 0, "°C", "temperature", sensor::STATE_CLASS_MEASUREMENT);
  mk_sensor(this->t_cur_, "Temperature Current", 0, "°C", "temperature", sensor::STATE_CLASS_MEASUREMENT);
  mk_sensor(this->t_pipe_, "Pipe Temperature Current", 0, "°C", "temperature", sensor::STATE_CLASS_MEASUREMENT);

  mk_sensor(this->quiet_s_, "Quiet", 0);
  mk_sensor(this->turbo_s_, "Turbo", 0);
  mk_sensor(this->led_s_, "LED", 0);
  mk_sensor(this->eco_s_, "Economy", 0);
  mk_sensor(this->lr_s_, "Left-Right", 0);
  mk_sensor(this->ud_s_, "Up-Down", 0);

  // ----- Switches -----
  auto mk_switch = [this](ACHISwitch *&ptr, ControlType type, const std::string &name) {
    ptr = new ACHISwitch();
    ptr->set_parent(this);
    ptr->set_type(type);
    ptr->set_name((this->name_prefix_ + " " + name).c_str());
    App.register_switch(ptr);
  };

  mk_switch(this->power_sw_, CTRL_POWER, "Power");
  mk_switch(this->quiet_sw_, CTRL_QUIET, "Quiet Mode");
  mk_switch(this->turbo_sw_, CTRL_TURBO, "Turbo Mode");
  mk_switch(this->led_sw_,   CTRL_LED,   "LED");
  mk_switch(this->eco_sw_,   CTRL_ECO,   "ECO Mode");
  mk_switch(this->ud_sw_,    CTRL_UPDOWN,"Up-Down Swing");
  mk_switch(this->lr_sw_,    CTRL_LEFTRIGHT,"Left-Right Swing");

  // ----- Number (setpoint) -----
  this->temp_num_ = new ACHINumber();
  this->temp_num_->set_parent(this);
  this->temp_num_->set_type(CTRL_TEMP);
  this->temp_num_->traits.set_min_value(18.0f);
  this->temp_num_->traits.set_max_value(28.0f);
  this->temp_num_->traits.set_step(1.0f);
  this->temp_num_->set_name((this->name_prefix_ + " Temperature").c_str());
  App.register_number(this->temp_num_);

  // ----- Selects -----
  auto mk_select = [this](ACHISelect *&ptr, ControlType type, const std::string &name, const std::vector<std::string> &options) {
    ptr = new ACHISelect();
    ptr->set_parent(this);
    ptr->set_type(type);
    ptr->traits.set_options(options);
    ptr->set_name((this->name_prefix_ + " " + name).c_str());
    App.register_select(ptr);
  };

  mk_select(this->mode_sel_,  CTRL_MODE,  "Mode", {"fan_only","heat","cool","dry","auto"});
  mk_select(this->wind_sel_,  CTRL_WIND,  "Wind", {"off","auto","lowest","low","medium","high","highest"});
  mk_select(this->sleep_sel_, CTRL_SLEEP, "Sleep", {"off","sleep_1","sleep_2","sleep_3","sleep_4"});

  // initial query
  this->send_query_();
  this->state_ = State::QUERY_SENT;
}

void ACHIComponent::update() {
  // FSM tick each update interval
  if (!this->lock_update_ && this->state_ == State::IDLE) {
    this->send_query_();
    this->state_ = State::QUERY_SENT;
  }
  // sliding write window
  if (this->write_changes_ && (millis() >= this->write_deadline_ms_)) {
    this->maybe_send_write_();
  }
}

// --- НОВЫЙ неблокирующий loop() ---
void ACHIComponent::loop() {
  // читаем максимум 128 байт за вызов — без ожидания прихода следующих
  uint8_t buf[128];
  size_t n = this->read_array(buf, sizeof(buf));  // non-blocking
  if (n == 0) return;

  // добавляем в RX-буфер
  rx_.insert(rx_.end(), buf, buf + n);

  // быстрый разбор всех полных кадров в буфере
  // допускаем несколько кадров подряд; обрабатываем и удаляем из начала буфера
  for (;;) {
    // ищем начало
    size_t start = 0;
    while (start + 1 < rx_.size() && !(rx_[start] == START0 && rx_[start + 1] == START1)) start++;
    if (start + 1 >= rx_.size()) {
      // нет начала — очищаем мусор слева
      if (start > 0) rx_.erase(rx_.begin(), rx_.begin() + start);
      break;
    }

    // ищем конец
    size_t end = start + 2;
    while (end + 1 < rx_.size() && !(rx_[end] == END0 && rx_[end + 1] == END1)) end++;
    if (end + 1 >= rx_.size()) {
      // полного кадра ещё нет — оставляем хвост начиная с start
      if (start > 0) rx_.erase(rx_.begin(), rx_.begin() + start);
      break;
    }

    // есть полный кадр [start..end+1]
    std::vector<uint8_t> frame(rx_.begin() + start, rx_.begin() + end + 2);
    this->handle_frame_(frame);

    // удаляем обработанный кадр и продолжаем искать следующий с начала
    rx_.erase(rx_.begin(), rx_.begin() + end + 2);
  }
}
