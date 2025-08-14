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
  rx_.reserve(256);  // уменьшает аллокации

  // ----- Text sensor -----
  this->power_text_ = new text_sensor::TextSensor();
  this->power_text_->set_internal(false);
  this->power_text_->set_name((this->name_prefix_ + " Power Status").c_str());
  App.register_text_sensor(this->power_text_);
  this->power_text_->publish_state("OFF");

  // ----- Numeric sensors -----
  auto mk_sensor = [this](sensor::Sensor *&ptr, const std::string &name, int8_t acc = 0,
                          const char *uom = nullptr, const char *dev_class = nullptr,
                          sensor::StateClass st = sensor::STATE_CLASS_MEASUREMENT) {
    ptr = new sensor::Sensor();
    ptr->set_internal(false);
    ptr->set_name((this->name_prefix_ + " " + name).c_str());
    ptr->set_accuracy_decimals(acc);
    if (uom != nullptr) ptr->set_unit_of_measurement(uom);
    if (dev_class != nullptr) ptr->set_device_class(dev_class);
    ptr->set_state_class(st);
    App.register_sensor(ptr);
  };

  mk_sensor(this->wind_s_,  "Wind", 0);
  mk_sensor(this->sleep_s_, "Sleep", 0);
  mk_sensor(this->mode_s_,  "Mode", 0);

  mk_sensor(this->t_set_,  "Temperature Set",     0, "°C", "temperature");
  mk_sensor(this->t_cur_,  "Temperature Current", 0, "°C", "temperature");
  mk_sensor(this->t_pipe_, "Pipe Temperature Current", 0, "°C", "temperature");

  mk_sensor(this->quiet_s_, "Quiet", 0);
  mk_sensor(this->turbo_s_, "Turbo", 0);
  mk_sensor(this->led_s_,   "LED",   0);
  mk_sensor(this->eco_s_,   "Economy", 0);
  mk_sensor(this->lr_s_,    "Left-Right", 0);
  mk_sensor(this->ud_s_,    "Up-Down", 0);

  // ----- Switches -----
  auto mk_switch = [this](ACHISwitch *&ptr, ControlType type, const std::string &name) {
    ptr = new ACHISwitch();
    ptr->set_internal(false);
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
  mk_switch(this->ud_sw_,    CTRL_UPDOWN, "Up-Down Swing");
  mk_switch(this->lr_sw_,    CTRL_LEFTRIGHT, "Left-Right Swing");

  // ----- Number (setpoint) -----
  this->temp_num_ = new ACHINumber();
  this->temp_num_->set_internal(false);
  this->temp_num_->set_parent(this);
  this->temp_num_->set_type(CTRL_TEMP);
  this->temp_num_->traits.set_min_value(18.0f);
  this->temp_num_->traits.set_max_value(28.0f);
  this->temp_num_->traits.set_step(1.0f);
  this->temp_num_->set_name((this->name_prefix_ + " Temperature").c_str());
  App.register_number(this->temp_num_);

  // ----- Selects -----
  auto mk_select = [this](ACHISelect *&ptr, ControlType type, const std::string &name,
                          const std::vector<std::string> &options) {
    ptr = new ACHISelect();
    ptr->set_internal(false);
    ptr->set_parent(this);
    ptr->set_type(type);
    ptr->traits.set_options(options);
    ptr->set_name((this->name_prefix_ + " " + name).c_str());
    App.register_select(ptr);
  };

  mk_select(this->mode_sel_,  CTRL_MODE,  "Mode",  {"fan_only","heat","cool","dry","auto"});
  mk_select(this->wind_sel_,  CTRL_WIND,  "Wind",  {"off","auto","lowest","low","medium","high","highest"});
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

  // публикуем состояние ТОЛЬКО здесь (а не в loop)
  if (this->has_pending_status_) {
    this->publish_states_from_102_(this->pending_status_);
    this->has_pending_status_ = false;
  }
}

// --- неблокирующий loop(): читаем только доступное, обрабатываем 1 кадр ---
void ACHIComponent::loop() {
  size_t avail = this->available();
  if (avail == 0) return;

  size_t chunk = avail > 32 ? 32 : avail;  // жёстче ограничим время цикла
  for (size_t i = 0; i < chunk; i++) {
    uint8_t b;
    if (!this->read_byte(&b)) break;  // non-blocking, т.к. avail > 0
    rx_.push_back(b);
  }

  // найти начало
  size_t start = 0;
  while (start + 1 < rx_.size() && !(rx_[start] == START0 && rx_[start + 1] == START1)) start++;
  if (start + 1 >= rx_.size()) {
    if (start > 0) rx_.erase(rx_.begin(), rx_.begin() + start);
    return;
  }

  // найти конец
  size_t end = start + 2;
  while (end + 1 < rx_.size() && !(rx_[end] == END0 && rx_[end + 1] == END1)) end++;
  if (end + 1 >= rx_.size()) {
    if (start > 0) rx_.erase(rx_.begin(), rx_.begin() + start);
    return;
  }

  // 1 полный кадр [start..end+1] — проверим и отдадим в update()
  std::vector<uint8_t> frame(rx_.begin() + start, rx_.begin() + end + 2);
  rx_.erase(rx_.begin(), rx_.begin() + end + 2);

  if (frame.size() > 20 && frame[CMD_IDX] == 102 && !this->lock_update_) {
    int crc = 0;
    for (size_t i = 2; i < frame.size() - 4; i++) crc += frame[i];
    if (crc != this->last_status_crc_) {
      this->last_status_crc_ = crc;
      this->pending_status_ = std::move(frame);
      this->has_pending_status_ = true;   // публикация произойдёт в update()
    }
    this->state_ = State::IDLE;
    return;
  }

  if (frame.size() > 20 && frame[CMD_IDX] == 101) {
    this->unlock_on_101_(frame);
    this->state_ = State::IDLE;
    return;
  }
}

void ACHIComponent::send_query_() {
  std::vector<uint8_t> req{0xF4,0xF5,0x00,0x40,0x0C,0x00,0x00,0x01,0x01,0xFE,0x01,0x00,0x00,0x66,0x00,0x00,0x00,0x01,0xB3,0xF4,0xFB};
  this->write_array(req.data(), req.size());
  this->state_ = State::WAIT_STATUS;
  ESP_LOGV(TAG, "Query status sent");
}

void ACHIComponent::handle_frame_(const std::vector<uint8_t> &bytes) {
  // (не используется для публикации — оставлено для совместимости, если понадобится)
  if (bytes.size() <= 20) return;
  if (!(bytes[0]==START0 && bytes[1]==START1)) return;

  if (bytes[CMD_IDX] == 101) {
    this->unlock_on_101_(bytes);
    this->state_ = State::IDLE;
  }
}

void ACHIComponent::publish_states_from_102_(const std::vector<uint8_t> &bytes) {
  // power
  bool new_power = (bytes[18] & 0x08) != 0;
  if (new_power != this->current_power_) this->current_power_ = new_power;
  if (this->power_text_) this->power_text_->publish_state(new_power ? "ON" : "OFF");
  if (this->power_sw_)   this->power_sw_->publish_state(new_power);

  // wind
  const char* wind = (bytes[16] < 19) ? this->decode_wind_[bytes[16]] : "off";
  if (this->current_wind_ != wind) {
    this->current_wind_ = wind;
    if (this->wind_s_)   this->wind_s_->publish_state(bytes[16]);
    if (this->wind_sel_) this->wind_sel_->publish_state(this->current_wind_);
  }

  // sleep
  const char* sleep = (bytes[17] < 5) ? this->decode_sleep_[bytes[17]] : "off";
  if (this->current_sleep_ != sleep) {
    this->current_sleep_ = sleep;
    if (this->sleep_s_)   this->sleep_s_->publish_state(bytes[17]);
    if (this->sleep_sel_) this->sleep_sel_->publish_state(this->current_sleep_);
  }

  // mode
  const char* mode = this->decode_mode_[(bytes[18] >> 4) & 0x07];
  if (this->current_ac_mode_ != mode) {
    this->current_ac_mode_ = mode;
    if (this->mode_s_)   this->mode_s_->publish_state(bytes[18] >> 4);
    if (this->mode_sel_) this->mode_sel_->publish_state(this->current_ac_mode_);
  }

  // temperatures
  if (this->current_set_temp_ != bytes[19]) {
    this->current_set_temp_ = bytes[19];
    if (this->t_set_)    this->t_set_->publish_state(bytes[19]);
    if (this->temp_num_) this->temp_num_->publish_state(this->current_set_temp_);
  }
  if (this->t_cur_)  this->t_cur_->publish_state(bytes[20]);
  if (this->t_pipe_) this->t_pipe_->publish_state(bytes[21]);

  // flags
  if (this->quiet_s_) {
    bool q = (bytes[36] & 0x04) != 0;
    this->quiet_s_->publish_state(q);
    if (this->quiet_sw_) this->quiet_sw_->publish_state(q);
  }
  if (this->eco_s_) {
    bool e = (bytes[35] & 0x04) != 0;
    this->eco_s_->publish_state(e);
    if (this->eco_sw_) this->eco_sw_->publish_state(e);
  }
  if (this->turbo_s_) {
    bool t = (bytes[35] & 0x02) != 0;
    this->turbo_s_->publish_state(t);
    if (this->turbo_sw_) this->turbo_sw_->publish_state(t);
  }
  if (this->led_s_) {
    bool l = (bytes[37] & 0x80) != 0;
    this->led_s_->publish_state(l);
    if (this->led_sw_) this->led_sw_->publish_state(l);
  }
  if (this->ud_s_) {
    bool ud = (bytes[35] & 0x80) != 0;
    this->ud_s_->publish_state(ud);
    if (this->ud_sw_) this->ud_sw_->publish_state(ud);
  }
  if (this->lr_s_) {
    bool lr = (bytes[35] & 0x40) != 0;
    this->lr_s_->publish_state(lr);
    if (this->lr_sw_) this->lr_sw_->publish_state(lr);
  }
}

void ACHIComponent::unlock_on_101_(const std::vector<uint8_t> &/*bytes*/) {
  this->lock_update_ = false;
  ESP_LOGV(TAG, "Update lock released (0x101)");
}

void ACHIComponent::on_switch(ControlType t, bool state) {
  switch (t) {
    case CTRL_POWER:
      this->power_bin_ = state ? 0b00001100 : 0b00000100;
      this->lock_update_ = true;
      this->schedule_write_();
      break;
    case CTRL_QUIET:
      this->quiet_bin_ = state ? 48 /*0b00110000*/ : 16 /*0b00010000*/;
      this->lock_update_ = true;
      this->schedule_write_();
      break;
    case CTRL_TURBO:
      this->turbo_bin_ = state ? 0b00001100 : 0b00000100;
      this->lock_update_ = true;
      this->schedule_write_();
      break;
    case CTRL_LED:
      this->frame_[36] = state ? 0b11000000 : 0b01000000;
      this->lock_update_ = true;
      this->write_changes_ = true;
      this->write_deadline_ms_ = millis() + 1500;
      break;
    case CTRL_ECO:
      this->eco_bin_ = state ? 0b00110000 : 0b00010000;
      this->lock_update_ = true;
      this->schedule_write_();
      break;
    case CTRL_UPDOWN:
      this->updown_bin_ = state ? 0b11000000 : 0b01000000;
      this->lock_update_ = true;
      this->schedule_write_();
      break;
    case CTRL_LEFTRIGHT:
      this->leftright_bin_ = state ? 0b00110000 : 0b00010000;
      this->lock_update_ = true;
      this->schedule_write_();
      break;
    default:
      break;
  }
}

void ACHIComponent::on_number(ControlType t, float value) {
  if (t != CTRL_TEMP) return;
  uint8_t c = static_cast<uint8_t>(value);
  if (c < 17 || c > 30) return;
  uint8_t tempX = (c << 1) | 0x01;
  if (this->current_set_temp_ != c) {
    this->frame_[19] = tempX;
    this->lock_update_ = true;
    this->schedule_write_();
    ESP_LOGD(TAG, "Target temp set to %d", c);
  }
}

void ACHIComponent::on_select(ControlType t, const std::string &value) {
  if (t == CTRL_MODE) {
    uint8_t idx = 0;
    if (value == "heat") idx = 1;
    else if (value == "cool") idx = 2;
    else if (value == "dry") idx = 3;
    else if (value == "auto") idx = 4;
    uint8_t mode = ((idx << 1) | 0x01) << 4;
    this->mode_bin_ = mode;
    this->lock_update_ = true;
    this->schedule_write_();
    ESP_LOGD(TAG, "AC mode -> %s", value.c_str());
  } else if (t == CTRL_WIND) {
    static const uint8_t codes[7] = {0,1,10,12,14,16,18};
    uint8_t listix = 0;
    if (value == "auto") listix = 1;
    else if (value == "lowest") listix = 2;
    else if (value == "low") listix = 3;
    else if (value == "medium") listix = 4;
    else if (value == "high") listix = 5;
    else if (value == "highest") listix = 6;
    uint8_t mode = codes[listix] + 1;
    this->frame_[16] = mode;
    this->lock_update_ = true;
    this->schedule_write_();
    ESP_LOGD(TAG, "Wind -> %s", value.c_str());
  } else if (t == CTRL_SLEEP) {
    static const uint8_t codes[5] = {0,1,2,4,8};
    uint8_t listix = 0;
    if (value == "sleep_1") listix = 1;
    else if (value == "sleep_2") listix = 2;
    else if (value == "sleep_3") listix = 3;
    else if (value == "sleep_4") listix = 4;
    uint8_t mode = (codes[listix] << 1) | 0x01;
    this->frame_[17] = mode;
    this->lock_update_ = true;
    this->schedule_write_();
    ESP_LOGD(TAG, "Sleep -> %s", value.c_str());
  }
}

void ACHIComponent::schedule_write_() {
  this->write_changes_ = true;
  this->write_deadline_ms_ = millis() + 1500; // sliding window
  ESP_LOGV(TAG, "Collecting changes for 1500ms");
}

void ACHIComponent::maybe_send_write_() {
  if (!this->write_changes_) return;
  this->write_changes_ = false;

  this->build_write_frame_();

  this->write_array(this->frame_.data(), this->frame_.size());
  this->state_ = State::WRITE_SENT;
  ESP_LOGD(TAG, "Changes written to AC");
}

void ACHIComponent::build_write_frame_() {
  this->frame_[18] = this->power_bin_ + this->mode_bin_;
  this->frame_[32] = this->updown_bin_ + this->leftright_bin_;
  this->frame_[33] = this->turbo_bin_ + this->eco_bin_;
  this->frame_[35] = this->quiet_bin_;

  // Turbo overrides
  if (this->turbo_bin_ == 0b00001100) {
    this->frame_[19] = 0;
    this->frame_[33] = this->turbo_bin_;
    this->frame_[35] = 0;
  }

  // Quiet overrides turbo & eco when active
  if (this->quiet_bin_ == 48) {
    this->frame_[33] = 0b00000100;
    this->frame_[35] = this->quiet_bin_;
  }

  // normalize one-shot bits
  if (this->eco_bin_ == 0b00010000) this->eco_bin_ = 0;
  if (this->quiet_bin_ == 16) this->quiet_bin_ = 0;
  this->turbo_bin_ = 0;

  put_crc_(this->frame_);
}

void ACHIComponent::put_crc_(std::vector<uint8_t> &buf) {
  short int csum = 0;
  int arrlen = buf.size();
  for (int i = 2; i < arrlen - 4; i++) csum += buf[i];
  uint8_t cr1 = (csum & 0xFF00) >> 8;
  uint8_t cr2 = (csum & 0x00FF);
  if (arrlen > 47) {
    buf[46] = cr1;
    buf[47] = cr2;
  }
}

}  // namespace ac_hi
}  // namespace esphome
