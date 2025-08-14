#include "ac_hi.h"
#include "esphome/components/climate/climate.h"

namespace esphome {
namespace ac_hi {

ACHi::ACHi(uart::UARTComponent *parent) : PollingComponent(1000), uart::UARTDevice(parent) {
  // Initialize pointers to nullptr
  sensor_wind = nullptr;
  sensor_sleep = nullptr;
  sensor_mode = nullptr;
  temp_set = nullptr;
  temp_current = nullptr;
  compr_freq_set = nullptr;
  compr_freq = nullptr;
  temp_outdoor = nullptr;
  temp_outdoor_condenser = nullptr;
  sensor_quiet = nullptr;
  sensor_turbo = nullptr;
  sensor_led = nullptr;
  sensor_eco = nullptr;
  temp_pipe_current = nullptr;
  sensor_left_right = nullptr;
  sensor_up_down = nullptr;

  power_status = nullptr;

  my_temperature = nullptr;

  ac_mode_select = nullptr;
  ac_wind_select = nullptr;
  ac_sleep_select = nullptr;

  power_switch = nullptr;
  quiet_switch = nullptr;
  turbo_switch = nullptr;
  eco_switch = nullptr;
  led_switch = nullptr;
  swing_up_down_switch = nullptr;
  swing_left_right_switch = nullptr;
}

void ACHi::setup() {
  // Initialize variables
  this->current_power_ = false;
  this->current_set_temp_ = 25;  // Default to 25°C
  this->current_ac_mode_ = "auto";
  this->current_wind_ = "auto";
  this->current_sleep_ = "off";

  this->lock_update_ = false;
  this->write_changes_ = false;

  this->power_bin_ = 0;
  this->mode_bin_ = 0;
  this->updown_bin_ = 0;
  this->leftright_bin_ = 0;
  this->turbo_bin_ = 0;
  this->eco_bin_ = 0;
  this->quiet_bin_ = 0;
  this->led_bin_ = 0;

  // Initialize the bytearray with the appropriate length and default values
  this->bytearray_ = {0xF4, 0xF5, 0x00, 0x40, 0x29, 0x00, 0x00, 0x01, 0x01, 0xFE, 0x01, 0x00, 0x00, 0x65, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF4, 0xFB};

  this->last_read_time_ = 0;
  this->last_write_time_ = 0;
  this->status_crc_ = 0;

  this->pending_write_ = false;
}

void ACHi::update() {
  ESP_LOGD("ACHi", "Update (apply user intents)");
  // Power
  if (this->power_switch != nullptr) {
    bool desired = this->power_switch->state;
    if (desired != this->current_power_) {
      this->set_power(desired);
    }
  }
  // Temperature
  if (this->my_temperature != nullptr) {
    uint8_t desired = static_cast<uint8_t>(this->my_temperature->state);
    if (desired != this->current_set_temp_) {
      this->set_temperature(desired);
    }
  }
  // Mode
  if (this->ac_mode_select != nullptr) {
    std::string desired = this->ac_mode_select->state;
    if (!desired.empty() && desired != this->current_ac_mode_) {
      this->set_mode(desired);
    }
  }
  // Wind
  if (this->ac_wind_select != nullptr) {
    std::string desired = this->ac_wind_select->state;
    if (!desired.empty() && desired != this->current_wind_) {
      this->set_fan_speed(desired);
    }
  }
  // Sleep
  if (this->ac_sleep_select != nullptr) {
    std::string desired = this->ac_sleep_select->state;
    if (!desired.empty() && desired != this->current_sleep_) {
      this->set_sleep_mode(desired);
    }
  }
  // Quiet
  if (this->quiet_switch != nullptr) {
    bool desired = this->quiet_switch->state;
    if (desired != this->sensor_quiet->state) {
      this->set_quiet_mode(desired);
    }
  }
  // Turbo
  if (this->turbo_switch != nullptr) {
    bool desired = this->turbo_switch->state;
    if (desired != this->sensor_turbo->state) {
      this->set_turbo_mode(desired);
    }
  }
  // Eco
  if (this->eco_switch != nullptr) {
    bool desired = this->eco_switch->state;
    if (desired != this->sensor_eco->state) {
      this->set_eco_mode(desired);
    }
  }
  // LED
  if (this->led_switch != nullptr) {
    bool desired = this->led_switch->state;
    if (desired != this->sensor_led->state) {
      this->set_led(desired);
    }
  }
  // Swing UD
  if (this->swing_up_down_switch != nullptr) {
    bool desired = this->swing_up_down_switch->state;
    if (this->sensor_up_down != nullptr && desired != this->sensor_up_down->state) {
      this->set_swing_up_down(desired);
    }
  }
  // Swing LR
  if (this->swing_left_right_switch != nullptr) {
    bool desired = this->swing_left_right_switch->state;
    if (this->sensor_left_right != nullptr && desired != this->sensor_left_right->state) {
      this->set_swing_left_right(desired);
    }
  }
}


void ACHi::loop() {
  uint32_t now = millis();

  // Periodically send read command every 1000ms
  if (now - this->last_read_time_ >= 1000) {
    this->last_read_time_ = now;
    if (!this->lock_update_) {
      this->send_read_command();
    } else {
      ESP_LOGD("ACHi", "Reading lock enabled. Skipping AC state reading.");
    }
  }
  
  ESP_LOGD("ACHi", "Time since last write time: %s", to_string(now - this->last_write_time_).c_str());
  // Handle pending write after delay
  if (this->pending_write_ && now - this->last_write_time_ >= 1500) {
    ESP_LOGD("ACHi", "Write start");
    this->write_changes();
    this->pending_write_ = false;
    ESP_LOGD("ACHi", "Write end");
  }

  // Process incoming data
  while (available()) {
    std::vector<uint8_t> data;
    while (available()) {
      data.push_back(read());
    }
    this->process_incoming_data(data);
  }
}

// Send read command to the AC unit
void ACHi::send_read_command() {
  std::vector<uint8_t> command = {0xF4, 0xF5, 0x00, 0x40, 0x0C, 0x00, 0x00, 0x01,
                                  0x01, 0xFE, 0x01, 0x00, 0x00, 0x66, 0x00, 0x00,
                                  0x00, 0x01, 0xB3, 0xF4, 0xFB};
  this->write_array(command);
}

// Process incoming UART data from the AC unit
void ACHi::process_incoming_data(const std::vector<uint8_t> &bytes) {
  // Manual debug
  ESP_LOGD("ACHi", "Status CRC: %s", to_string(this->status_crc_).c_str());
  for (int val = 0; val < bytes.size(); val++) {
    if (bytes[val] != 0) {
      ESP_LOGD("ACHi", "BYTE %s: val %s", to_string(val).c_str(), to_string(bytes[val]).c_str());
    }
  }
  if (bytes.size() > 20 && bytes[0] == 0xF4 && bytes[1] == 0xF5) {
    // Parse status message from AC commands 102 and 101
    if ((bytes[13] == 102 && bytes[2] == 1 && !this->lock_update_)) {
      // Calculate CRC
      int crc = 0;
      int arrlen = bytes.size();
      for (int i = 2; i < arrlen - 4; i++) {
        crc += bytes[i];
      }
      ESP_LOGD("ACHi", "Status CRC New: %s", to_string(crc).c_str());
      if (crc != this->status_crc_) { //&& bytes[45] < 127
        this->status_crc_ = crc;
        ESP_LOGD("ACHi", "Ok");
        // Parse power status
        bool power_status = (bytes[18] & 0x08) != 0;
        //if (this->current_power_ != power_status) {
          this->current_power_ = power_status;
          if (this->power_status != nullptr)
            this->power_status->publish_state(power_status ? "ON" : "OFF");
          if (this->power_switch != nullptr)
            this->power_switch->publish_state(power_status);
        //}

        // Parse current wind
        std::string wind = this->decode_wind_codes_[bytes[16]];
        if (this->current_wind_ != wind) {
          this->current_wind_ = wind;
          if (this->sensor_wind != nullptr)
            this->sensor_wind->publish_state(bytes[16]);
          if (this->ac_wind_select != nullptr)
            this->ac_wind_select->publish_state(wind);
        }

        // Parse current sleep mode
        std::string sleep_mode = this->decode_sleep_codes_[bytes[17]];
        if (this->current_sleep_ != sleep_mode) {
          this->current_sleep_ = sleep_mode;
          if (this->sensor_sleep != nullptr)
            this->sensor_sleep->publish_state(bytes[17]);
          if (this->ac_sleep_select != nullptr)
            this->ac_sleep_select->publish_state(sleep_mode);
        }

        // Parse current AC mode
        std::string ac_mode = this->decode_acmode_codes_[bytes[18] >> 4];
        if (this->current_ac_mode_ != ac_mode) {
          this->current_ac_mode_ = ac_mode;
          //if (this->sensor_mode != nullptr)
          //this->sensor_mode->publish_state(bytes[18] >> 4);
          //if (this->ac_mode_select != nullptr)
          //this->ac_mode_select->publish_state(ac_mode);
        }

        // Parse current set temperature
        if (this->current_set_temp_ != bytes[19]) {
          this->current_set_temp_ = bytes[19];
          if (this->temp_set != nullptr)
            this->temp_set->publish_state(bytes[19]);
          if (this->my_temperature != nullptr)
            this->my_temperature->publish_state(bytes[19]);
        }

        // Update other sensors
        if (this->temp_current != nullptr)
          this->temp_current->publish_state(bytes[20]);
        if (this->temp_pipe_current != nullptr)
          this->temp_pipe_current->publish_state(bytes[21]);
        if (this->compr_freq_set != nullptr)
          this->compr_freq_set->publish_state(bytes[42]);
        if (this->compr_freq != nullptr)
          this->compr_freq->publish_state(bytes[43]);
        if (this->temp_outdoor != nullptr)
          this->temp_outdoor->publish_state(bytes[44]);
        if (this->temp_outdoor_condenser != nullptr)
          this->temp_outdoor_condenser->publish_state(bytes[45]);

        // Update switches and sensors for special modes if command from AC is 102
        if (bytes[13] == 102) {
          // Quiet mode
          bool quiet = (bytes[35] & 0x30) == 0x30;
          if (this->sensor_quiet != nullptr)
            this->sensor_quiet->publish_state(quiet);
          if (this->quiet_switch != nullptr)
            this->quiet_switch->publish_state(quiet);

          // Eco mode
          bool eco = (bytes[33] & 0x30) == 0x30;
          if (this->sensor_eco != nullptr)
            this->sensor_eco->publish_state(eco);
          if (this->eco_switch != nullptr)
            this->eco_switch->publish_state(eco);

          // Turbo mode
          bool turbo = (bytes[33] & 0x0C) == 0x0C;
          if (this->sensor_turbo != nullptr)
            this->sensor_turbo->publish_state(turbo);
          if (this->turbo_switch != nullptr)
            this->turbo_switch->publish_state(turbo);

          // LED
          bool led = (bytes[36] & 0xC0) == 0xC0;
          if (this->sensor_led != nullptr)
            this->sensor_led->publish_state(led);
          if (this->led_switch != nullptr)
            this->led_switch->publish_state(led);

          // Swing Up-Down
          bool swing_ud = (bytes[32] & 0xC0) == 0xC0;
          if (this->sensor_up_down != nullptr)
            this->sensor_up_down->publish_state(swing_ud);
          if (this->swing_up_down_switch != nullptr)
            this->swing_up_down_switch->publish_state(swing_ud);

          // Swing Left-Right
          bool swing_lr = (bytes[32] & 0x30) == 0x30;
          if (this->sensor_left_right != nullptr)
            this->sensor_left_right->publish_state(swing_lr);
          if (this->swing_left_right_switch != nullptr)
            this->swing_left_right_switch->publish_state(swing_lr);
        }

        ESP_LOGD("ACHi", "pending_write_ = true");
        this->pending_write_ = true;
      }
    }

    // Unlocking updates if command from AC is 101
    if (bytes[13] == 101 && bytes[2] == 1) {
      this->lock_update_ = false;
      ESP_LOGD("ACHi", "Update lock released");
    }
  }
}

// Schedule a write after a delay to collect multiple changes
void ACHi::schedule_write_changes() {
  this->write_changes_ = true;
  this->lock_update_ = true;
  //this->last_write_time_ = millis();
  this->pending_write_ = true;
}

// Implement the logic to write changes to the AC unit
void ACHi::write_changes() {
  ESP_LOGD("ACHi", "Writing changes to AC unit.");

  // Merge BIT 18 power & mode
  this->bytearray_[18] = this->power_bin_ + this->mode_bin_;
  ESP_LOGD("ACHi", "WRITE BYTE 18: Power - Mode, val %s", to_string(this->bytearray_[18]).c_str());

  // Merge BIT 32 up-down & left-right
  this->bytearray_[32] = this->updown_bin_ + this->leftright_bin_;
  ESP_LOGD("ACHi", "WRITE BYTE 32: Up-Down Left-Right, val %s", to_string(this->bytearray_[32]).c_str());

  // Merge BIT 33 turbo & eco
  this->bytearray_[33] = this->turbo_bin_ + this->eco_bin_;
  ESP_LOGD("ACHi", "WRITE BYTE 33: Eco - Turbo, val %s", to_string(this->bytearray_[33]).c_str());

  // Quiet mode
  this->bytearray_[35] = this->quiet_bin_;

  // LED mode
  this->bytearray_[36] = this->led_bin_;

  // Turbo overrides eco & quiet
  if (this->turbo_bin_ == 0x0C) {
    this->bytearray_[19] = 0;  // Override temperature
    this->bytearray_[33] = this->turbo_bin_;  // Override eco
    this->bytearray_[35] = 0;  // Override quiet
  }

  // Restore temp settings after turbo off
  if (this->turbo_bin_ == 0x04) {
    ESP_LOGD("ACHi", "WRITE BYTE 19: Turbo Off Tempdata, val %s", to_string(this->bytearray_[19]).c_str());
    // Update temperature
    if (this->my_temperature != nullptr)
      this->my_temperature->publish_state(this->my_temperature->state);
  }

  // Quiet overrides turbo & eco when turbo not switching on
  if (this->quiet_bin_ == 0x30) {
    this->bytearray_[33] = 0x04;  // Switching off turbo & eco
    this->bytearray_[35] = this->quiet_bin_;
  }

  // Reset bins after use
  if (this->eco_bin_ == 0x10)
    this->eco_bin_ = 0;
  if (this->quiet_bin_ == 0x10)
    this->quiet_bin_ = 0;

  // Turbo mode always to 0
  this->turbo_bin_ = 0;

  ESP_LOGD("ACHi", "WRITE BYTE 19: Temp setting, val %s", to_string(this->bytearray_[19]).c_str());

  // Calculate checksum
  short int csum = 0;
  int arrlen = this->bytearray_.size();
  for (int i = 2; i < arrlen - 4; i++) {
    csum += this->bytearray_[i];
  }

  uint8_t cr1 = (csum & 0x0000ff00) >> 8;
  uint8_t cr2 = (csum & 0x000000ff);

  this->bytearray_[46] = cr1;
  this->bytearray_[47] = cr2;

  // Send the command
  this->write_array(this->bytearray_);
  this->last_write_time_ = millis();

  // Reset write flags
  this->write_changes_ = false;
  this->lock_update_ = false;
}

// Control methods implementation

void ACHi::set_power(bool power) {
  if (this->current_power_ != power) {
    this->power_bin_ = power ? 0x0C : 0x04;
    this->schedule_write_changes();
    ESP_LOGD("ACHi", "Power set to %s", power ? "ON" : "OFF");
  }
}

void ACHi::set_temperature(float temperature) {
  uint8_t temp = static_cast<uint8_t>(temperature);
  if (temp >= 16 && temp <= 30 && this->current_set_temp_ != temp) {
    uint8_t tempX = (temp << 1) | 0x01;
    this->bytearray_[19] = tempX;
    this->schedule_write_changes();
    ESP_LOGD("ACHi", "Temperature set to %d", temp);
  }
}

void ACHi::set_mode(const std::string &mode) {
  auto it = std::find(std::begin(this->decode_acmode_codes_), std::end(this->decode_acmode_codes_), mode);
  if (it != std::end(this->decode_acmode_codes_)) {
    uint8_t index = std::distance(this->decode_acmode_codes_, it);
    uint8_t mode_bin = ((this->mode_codes_[index] << 1) | 0x01) << 4;
    this->mode_bin_ = mode_bin;
    this->schedule_write_changes();
    ESP_LOGD("ACHi", "AC mode set to %s", mode);
  }
}

void ACHi::set_fan_speed(const std::string &speed) {
  auto it = std::find(std::begin(this->decode_wind_codes_), std::end(this->decode_wind_codes_), speed);
  if (it != std::end(this->decode_wind_codes_)) {
    uint8_t index = std::distance(this->decode_wind_codes_, it);
    uint8_t mode = this->wind_codes_[index] + 1;
    this->bytearray_[16] = mode;
    this->schedule_write_changes();
    ESP_LOGD("ACHi", "Fan speed set to %s", speed.c_str());
  }
}

void ACHi::set_sleep_mode(const std::string &sleep_mode) {
  auto it = std::find(std::begin(this->decode_sleep_codes_), std::end(this->decode_sleep_codes_), sleep_mode);
  if (it != std::end(this->decode_sleep_codes_)) {
    uint8_t index = std::distance(this->decode_sleep_codes_, it);
    uint8_t mode = (this->sleep_codes_[index] << 1) | 0x01;
    this->bytearray_[17] = mode;
    this->schedule_write_changes();
    ESP_LOGD("ACHi", "Sleep mode set to %s", sleep_mode.c_str());
  }
}

void ACHi::set_quiet_mode(bool quiet) {
  if (quiet) {
    this->quiet_bin_ = 0x30;
  } else {
    this->quiet_bin_ = 0x10;
  }
  this->schedule_write_changes();
  ESP_LOGD("ACHi", "Quiet mode set to %s", quiet ? "ON" : "OFF");
}

void ACHi::set_turbo_mode(bool turbo) {
  if (turbo) {
    this->turbo_bin_ = 0x0C;
  } else {
    this->turbo_bin_ = 0x04;
  }
  this->schedule_write_changes();
  ESP_LOGD("ACHi", "Turbo mode set to %s", turbo ? "ON" : "OFF");
}

void ACHi::set_eco_mode(bool eco) {
  if (eco) {
    this->eco_bin_ = 0x30;
  } else {
    this->eco_bin_ = 0x10;
  }
  this->schedule_write_changes();
  ESP_LOGD("ACHi", "Eco mode set to %s", eco ? "ON" : "OFF");
}

void ACHi::set_led(bool led) {
  this->led_bin_ = led ? 0xC0 : 0x40;
  this->schedule_write_changes();
  ESP_LOGD("ACHi", "LED set to %s", led ? "ON" : "OFF");
}

void ACHi::set_swing_up_down(bool swing) {
  this->updown_bin_ = swing ? 0xC0 : 0x40;
  this->schedule_write_changes();
  ESP_LOGD("ACHi", "Swing Up/Down set to %s", swing ? "ON" : "OFF");
}

void ACHi::set_swing_left_right(bool swing) {
  this->leftright_bin_ = swing ? 0x30 : 0x10;
  this->schedule_write_changes();
  ESP_LOGD("ACHi", "Swing Left/Right set to %s", swing ? "ON" : "OFF");
}

}  // namespace ac_hi
}  // namespace esphome