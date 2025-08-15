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
    this->first_publish_ = false;
  }
}

// --- неблокирующий loop(): читаем только доступное, 1 кадр за итерацию ---
void ACHIComponent::loop() {
  size_t avail = this->available();
  if (avail > 32) avail = 32;       // жёсткое ограничение времени цикла
  for (size_t i = 0; i < avail; i++) {
    uint8_t b;
    if (!this->read_byte(&b)) break; // non-blocking, т.к. проверили avail
    rb_push_(b);
  }

  // парсер: ищем начало
  size_t cnt = rb_count_();
  size_t start_idx = 0;
  for (; start_idx + 1 < cnt; start_idx++) {
    uint8_t b0, b1;
    rb_peek_(start_idx, b0);
    rb_peek_(start_idx + 1, b1);
    if (b0 == START0 && b1 == START1) break;
  }
  if (start_idx + 1 >= cnt) {
    // нет начала — выбрасываем мусор
    if (start_idx > 0) rb_pop_n_(start_idx);
    return;
  }
  // есть начало, выбрасываем мусор слева
  if (start_idx > 0) rb_pop_n_(start_idx);

  // пересчитываем количество после сдвига
  cnt = rb_count_();

  // ищем конец
  size_t end_idx = 2;
  for (; end_idx + 1 < cnt; end_idx++) {
    uint8_t b0, b1;
    rb_peek_(end_idx, b0);
    rb_peek_(end_idx + 1, b1);
    if (b0 == END0 && b1 == END1) break;
  }
  if (end_idx + 1 >= cnt) {
    // полного кадра ещё нет
    return;
  }

  // есть один полный кадр [0..end_idx+1]
  std::vector<uint8_t> frame;
  frame.reserve(end_idx + 2);
  for (size_t i = 0; i <= end_idx + 1; i++) {
    uint8_t b;
    rb_peek_(i, b);
    frame.push_back(b);
  }
  // удаляем кадр из буфера
  rb_pop_n_(end_idx + 2);

  // обработаем заголовок, но публикацию отложим до update()
  if (frame.size() > 20) {
    if (frame[CMD_IDX] == 102 && !this->lock_update_) {
      int crc = 0;
      for (size_t i = 2; i < frame.size() - 4; i++) crc += frame[i];
      if (crc != this->last_status_crc_) {
        this->last_status_crc_ = crc;
        this->pending_status_ = std::move(frame);
        this->has_pending_status_ = true;
      }
      this->state_ = State::IDLE;
      return;
    }
    if (frame[CMD_IDX] == 101) {
      this->unlock_on_101_(frame);
      this->state_ = State::IDLE;
      return;
    }
  }
}

void ACHIComponent::send_query_() {
  std::vector<uint8_t> req{0xF4,0xF5,0x00,0x40,0x0C,0x00,0x00,0x01,0x01,0xFE,0x01,0x00,0x00,0x66,0x00,0x00,0x00,0x01,0xB3,0xF4,0xFB};
  this->write_array(req.data(), req.size());
  this->state_ = State::WAIT_STATUS;
  ESP_LOGV(TAG, "Query status sent");
}

void ACHIComponent::handle_frame_(const std::vector<uint8_t> &bytes) {
  // (не используется для публикации — оставлено для совместимости)
  if (bytes.size() <= 20) return;
  if (bytes[CMD_IDX] == 101) {
    this->unlock_on_101_(bytes);
    this->state_ = State::IDLE;
  }
}

void ACHIComponent::publish_states_from_102_(const std::vector<uint8_t> &bytes) {
  // --- power ---
  bool new_power = (bytes[18] & 0x08) != 0;
  if (first_publish_ || new_power != last_power_) {
    last_power_ = new_power;
    if (this->power_text_) this->power_text_->publish_state(new_power ? "ON" : "OFF");
    if (this->power_sw_)   this->power_sw_->publish_state(new_power);
  }
  this->current_power_ = new_power;

  // --- wind ---
  uint8_t wind_raw = bytes[16];
  if (first_publish_ || wind_raw != last_wind_raw_) {
    last_wind_raw_ = wind_raw;
    const char* wind = (wind_raw < 19) ? this->decode_wind_[wind_raw] : "off";
    this->current_wind_ = wind;
    if (this->wind_s_)   this->wind_s_->publish_state(wind_raw);
    if (this->wind_sel_) this->wind_sel_->publish_state(this->current_wind_);
  }

  // --- sleep ---
  uint8_t sleep_raw = bytes[17];
  if (first_publish_ || sleep_raw != last_sleep_raw_) {
    last_sleep_raw_ = sleep_raw;
    const char* sleep = (sleep_raw < 5) ? this->decode_sleep_[sleep_raw] : "off";
    this->current_sleep_ = sleep;
    if (this->sleep_s_)   this->sleep_s_->publish_state(sleep_raw);
    if (this->sleep_sel_) this->sleep_sel_->publish_state(this->current_sleep_);
  }

  // --- mode ---
  uint8_t mode_raw = (bytes[18] >> 4) & 0x07;
  if (first_publish_ || mode_raw != last_mode_raw_) {
    last_mode_raw_ = mode_raw;
    const char* mode = this->decode_mode_[mode_raw];
    this->current_ac_mode_ = mode;
    if (this->mode_s_)   this->mode_s_->publish_state(mode_raw);
    if (this->mode_sel_) this->mode_sel_->publish_state(this->current_ac_mode_);
  }

  // --- temperatures ---
  uint8_t tset = bytes[19];
  if (first_publish_ || tset != last_t_set_) {
    last_t_set_ = tset;
    if (this->t_set_)    this->t_set_->publish_state(tset);
    if (this->temp_num_) this->temp_num_->publish_state(tset);
  }
  uint8_t tcur = bytes[20];
  if (first_publish_ || tcur != last_t_cur_) {
    last_t_cur_ = tcur;
    if (this->t_cur_)    this->t_cur_->publish_state(tcur);
  }
  uint8_t tpipe = bytes[21];
  if (first_publish_ || tpipe != last_t_pipe_) {
    last_t_pipe_ = tpipe;
    if (this->t_pipe_)   this->t_pipe_->publish_state(tpipe);
  }

  // --- flags ---
  bool q  = (bytes[36] & 0x04) != 0;
  bool e  = (bytes[35] & 0x04) != 0;
  bool t  = (bytes[35] & 0x02) != 0;
  bool l  = (bytes[37] & 0x80) != 0;
  bool ud = (bytes[35] & 0x80) != 0;
  bool lr = (bytes[35] & 0x40) != 0;

  if (first_publish_ || q  != last_quiet_) { last_quiet_  = q;  if (quiet_s_) quiet_s_->publish_state(q);  if (quiet_sw_) quiet_sw_->publish_state(q); }
  if (first_publish_ || e  != last_eco_)   { last_eco_    = e;  if (eco_s_)   eco_s_->publish_state(e);    if (eco_sw_)   eco_sw_->publish_state(e); }
  if (first_publish_ || t  != last_turbo_) { last_turbo_  = t;  if (turbo_s_) turbo_s_->publish_state(t);  if (turbo_sw_) turbo_sw_->publish_state(t); }
  if (first_publish_ || l  != last_led_)   { last_led_    = l;  if (led_s_)   led_s_->publish_state(l);    if (led_sw_)   led_sw_->publish_state(l); }
  if (first_publish_ || ud != last_ud_)    { last_ud_     = ud; if (ud_s_)    ud_s_->publish_state(ud);    if (ud_sw_)    ud_sw_->publish_state(ud); }
  if (first_publish_ || lr != last_lr_)    { last_lr_     = lr; if (lr_s_)    lr_s_->publish_state(lr);    if (lr_sw_)    lr_sw_->publish_state(lr); }
}

void ACHIComponent::unlock_on_101_(const std::vector<uint8_t> &/*bytes*/) {
  this->lock_update_ = false;
  ESP_LOGV(TAG, "Update lock released (0x101)");
}

void ACHIComponent::on_switch(ControlType t, bool state) {
  switch (t) {
    case CTRL_POWER:
      this->power_bin_ = state ? 0b00001100 : 0b00000100;
      if (!state) this->mode_bin_ = 0x00;  // clear mode when turning off
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
    uint8_t mode = idx << 4;
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
