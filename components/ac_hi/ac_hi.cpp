#include "ac_hi.h"

namespace esphome {
namespace ac_hi {

using esphome::uart::UARTDevice;
using esphome::uart::UARTComponent;

static const uint8_t START0 = 0xF4;
static const uint8_t START1 = 0xF5;
static const uint8_t END0   = 0xF4;
static const uint8_t END1   = 0xFB;

static const uint8_t CMD_IDX = 13;  // per YAML parsing

// ------------ ACHISwitch ------------
void ACHISwitch::write_state(bool state) {
  if (this->parent_ == nullptr) return;
  this->parent_->on_switch(this->type_, state);
  // actual publish happens from parent after device confirms / status arrives
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
  ESP_LOGI(TAG, "AC-Hi component init");
  // publish initial OFF as в вашем YAML
  if (this->power_text_ != nullptr) this->power_text_->publish_state("OFF");
}

void ACHIComponent::update() {
  // FSM tick every update_interval (default 1s)
  if (!this->lock_update_ && this->state_ == State::IDLE) {
    this->send_query_();
    this->state_ = State::QUERY_SENT;
  }

  // handle sliding write window
  if (this->write_changes_ && (millis() >= this->write_deadline_ms_)) {
    this->maybe_send_write_();
  }
}

void ACHIComponent::loop() {
  // collect bytes only when something is happening
  while (this->available()) {
    uint8_t b;
    if (!this->read_byte(&b)) break;
    rx_.push_back(b);

    // find end delimiter [F4 FB]
    size_t n = rx_.size();
    if (n >= 2 && rx_[n-2] == END0 && rx_[n-1] == END1) {
      // find start marker in buffer
      size_t start = 0;
      for (size_t i = 0; i + 1 < n; i++) {
        if (rx_[i] == START0 && rx_[i+1] == START1) { start = i; break; }
      }
      std::vector<uint8_t> frame(rx_.begin() + start, rx_.end());
      this->handle_frame_(frame);
      rx_.clear();
    }
  }
}

void ACHIComponent::send_query_() {
  // Short request from your YAML (0x66)
  std::vector<uint8_t> req{0xF4,0xF5,0x00,0x40,0x0C,0x00,0x00,0x01,0x01,0xFE,0x01,0x00,0x00,0x66,0x00,0x00,0x00,0x01,0xB3,0xF4,0xFB};
  this->write_array(req.data(), req.size());
  this->state_ = State::WAIT_STATUS;
  ESP_LOGV(TAG, "Query status sent");
}

void ACHIComponent::handle_frame_(const std::vector<uint8_t> &bytes) {
  if (bytes.size() <= 20) return;
  if (!(bytes[0]==START0 && bytes[1]==START1)) return;

  // 0x102 status payload (bytes[13] == 102) and not during locked write
  if (bytes[CMD_IDX] == 102 && !this->lock_update_) {
    // CRC check (sum[2..len-5])
    int crc = 0;
    for (size_t i = 2; i < bytes.size()-4; i++) crc += bytes[i];

    if (crc != this->last_status_crc_) {
      this->last_status_crc_ = crc;
      this->publish_states_from_102_(bytes);
    } else {
      ESP_LOGV(TAG, "Status unchanged (CRC)");
    }
    this->state_ = State::IDLE;
  }

  // 0x101 ack/unlock
  if (bytes[CMD_IDX] == 101) {
    this->unlock_on_101_(bytes);
    this->state_ = State::IDLE;
  }
}

void ACHIComponent::publish_states_from_102_(const std::vector<uint8_t> &bytes) {
  // -------- Power --------
  bool new_power = (bytes[18] & 0x08) != 0;
  if (new_power != this->current_power_) {
    this->current_power_ = new_power;
  }
  if (this->power_text_ != nullptr) this->power_text_->publish_state(new_power ? "ON" : "OFF");
  if (this->power_sw_ != nullptr) this->power_sw_->publish_state(new_power);

  // -------- Wind ----------
  const char* wind = (bytes[16] < 19) ? this->decode_wind_[bytes[16]] : "off";
  if (this->current_wind_ != wind) {
    this->current_wind_ = wind;
    if (this->wind_s_) this->wind_s_->publish_state(bytes[16]);
    if (this->wind_sel_) this->wind_sel_->publish_state(this->current_wind_);
  }

  // -------- Sleep ----------
  const char* sleep = (bytes[17] < 5) ? this->decode_sleep_[bytes[17]] : "off";
  if (this->current_sleep_ != sleep) {
    this->current_sleep_ = sleep;
    if (this->sleep_s_) this->sleep_s_->publish_state(bytes[17]);
    if (this->sleep_sel_) this->sleep_sel_->publish_state(this->current_sleep_);
  }

  // -------- Mode ----------
  const char* mode = this->decode_mode_[(bytes[18] >> 4) & 0x07];
  if (this->current_ac_mode_ != mode) {
    this->current_ac_mode_ = mode;
    if (this->mode_s_) this->mode_s_->publish_state(bytes[18] >> 4);
    if (this->mode_sel_) this->mode_sel_->publish_state(this->current_ac_mode_);
  }

  // -------- Temps ----------
  if (this->current_set_temp_ != bytes[19]) {
    this->current_set_temp_ = bytes[19];
    if (this->t_set_) this->t_set_->publish_state(bytes[19]);
    // Update number entity to reflect actual setpoint (decoded is already integer 18..28 in your YAML logic)
    if (this->temp_num_) this->temp_num_->publish_state(this->current_set_temp_);
  }
  if (this->t_cur_) this->t_cur_->publish_state(bytes[20]);
  if (this->t_pipe_) this->t_pipe_->publish_state(bytes[21]);

  // -------- Flags (only for cmd 102) ----------
  // quiet (byte 36 bit 2)
  if (this->quiet_s_) {
    bool q = (bytes[36] & 0x04) != 0;
    this->quiet_s_->publish_state(q);
    if (this->quiet_sw_) this->quiet_sw_->publish_state(q);
  }
  // eco (byte 35 bit 2)
  if (this->eco_s_) {
    bool e = (bytes[35] & 0x04) != 0;
    this->eco_s_->publish_state(e);
    if (this->eco_sw_) this->eco_sw_->publish_state(e);
  }
  // turbo (byte 35 bit 1)
  if (this->turbo_s_) {
    bool t = (bytes[35] & 0x02) != 0;
    this->turbo_s_->publish_state(t);
    if (this->turbo_sw_) this->turbo_sw_->publish_state(t);
  }
  // LED (byte 37 bit 7)
  if (this->led_s_) {
    bool l = (bytes[37] & 0x80) != 0;
    this->led_s_->publish_state(l);
    if (this->led_sw_) this->led_sw_->publish_state(l);
  }
  // Up-Down (byte 35 bit 7)
  if (this->ud_s_) {
    bool ud = (bytes[35] & 0x80) != 0;
    this->ud_s_->publish_state(ud);
    if (this->ud_sw_) this->ud_sw_->publish_state(ud);
  }
  // Left-Right (byte 35 bit 6)
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
  // mirror original YAML behavior & masks
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
      // direct byte write (byte 36)
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
  // encode: (c << 1) | 1
  uint8_t c = static_cast<uint8_t>(value);
  if (c < 17 || c > 30) return; // sanity, YAML used 18..28
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
    // table {fan_only, heat, cool, dry, auto} -> {0,1,2,3,4} then <<1 |1 then <<4
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
    // {off,auto,lowest,low,medium,high,highest} -> {0,1,10,12,14,16,18} then +1 when writing
    uint8_t listix = 0;
    if (value == "auto") listix = 1;
    else if (value == "lowest") listix = 2;
    else if (value == "low") listix = 3;
    else if (value == "medium") listix = 4;
    else if (value == "high") listix = 5;
    else if (value == "highest") listix = 6;
    static const uint8_t codes[7] = {0,1,10,12,14,16,18};
    uint8_t mode = codes[listix] + 1;
    this->frame_[16] = mode;
    this->lock_update_ = true;
    this->schedule_write_();
    ESP_LOGD(TAG, "Wind -> %s", value.c_str());
  } else if (t == CTRL_SLEEP) {
    // {off, s1, s2, s3, s4} -> {0,1,2,4,8} then <<1 |1
    uint8_t listix = 0;
    if (value == "sleep_1") listix = 1;
    else if (value == "sleep_2") listix = 2;
    else if (value == "sleep_3") listix = 3;
    else if (value == "sleep_4") listix = 4;
    static const uint8_t codes[5] = {0,1,2,4,8};
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

  // Merge fields
  this->build_write_frame_();

  // Send
  this->write_array(this->frame_.data(), this->frame_.size());
  this->state_ = State::WRITE_SENT;
  ESP_LOGD(TAG, "Changes written to AC");
}

void ACHIComponent::build_write_frame_() {
  // byte 18: power + mode
  this->frame_[18] = this->power_bin_ + this->mode_bin_;
  // byte 32: up-down + left-right
  this->frame_[32] = this->updown_bin_ + this->leftright_bin_;
  // byte 33: turbo + eco
  this->frame_[33] = this->turbo_bin_ + this->eco_bin_;
  // byte 35: quiet
  this->frame_[35] = this->quiet_bin_;

  // Turbo overrides eco & quiet
  if (this->turbo_bin_ == 0b00001100) {
    this->frame_[19] = 0;         // override temperature
    this->frame_[33] = this->turbo_bin_; // override eco
    this->frame_[35] = 0;         // override quiet
  }

  // Restore temp settings after turbo off
  if (this->turbo_bin_ == 0b00000100) {
    // keep frame_[19] as previously set
  }

  // Quiet overrides turbo & eco when turbo not switching on
  if (this->quiet_bin_ == 48) {
    this->frame_[33] = 0b00000100; // switching off turbo & eco
    this->frame_[35] = this->quiet_bin_;
  }

  // normalize one-shot bits (per YAML)
  if (this->eco_bin_ == 0b00010000) this->eco_bin_ = 0;
  if (this->quiet_bin_ == 16) this->quiet_bin_ = 0;
  this->turbo_bin_ = 0;

  // CRC
  put_crc_(this->frame_);
}

void ACHIComponent::put_crc_(std::vector<uint8_t> &buf) {
  short int csum = 0;
  int arrlen = buf.size();
  for (int i = 2; i < arrlen - 4; i++) csum += buf[i];
  uint8_t cr1 = (csum & 0xFF00) >> 8;
  uint8_t cr2 = (csum & 0x00FF);
  // positions as in YAML (46,47)
  if (arrlen > 47) {
    buf[46] = cr1;
    buf[47] = cr2;
  }
}

}  // namespace ac_hi
}  // namespace esphome
