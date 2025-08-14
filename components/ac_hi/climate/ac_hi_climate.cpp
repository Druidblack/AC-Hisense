// SPDX-License-Identifier: MIT
#include "ac_hi_climate.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ac_hi {

static const char *const TAG = "ac_hi.climate";

// ======================= Setup & Config =======================

void ACHiClimate::setup() {
  ESP_LOGI(TAG, "Init climate over RS-485");
  this->check_uart_settings(9600, 1, uart::UART_CONFIG_PARITY_NONE, 8);

  // Request first status soon (we build write-frames from it)
  this->set_timeout("init_status", 1000, [this]() { this->send_status_request_(); });
}

void ACHiClimate::dump_config() {
  ESP_LOGCONFIG(TAG, "AC-Hi Climate (UART RS-485)");
  ESP_LOGCONFIG(TAG, "  Poll interval: %u ms", this->update_interval_ms_);
}

climate::ClimateTraits ACHiClimate::traits() {
  climate::ClimateTraits t;
  t.set_supports_current_temperature(true);
  t.set_supported_modes({
      climate::CLIMATE_MODE_OFF,
      climate::CLIMATE_MODE_COOL,
      climate::CLIMATE_MODE_HEAT,
      climate::CLIMATE_MODE_DRY,
      climate::CLIMATE_MODE_AUTO,
      climate::CLIMATE_MODE_FAN_ONLY,
  });
  t.set_supported_fan_modes({
      climate::CLIMATE_FAN_AUTO,
      climate::CLIMATE_FAN_LOW,
      climate::CLIMATE_FAN_MEDIUM,
      climate::CLIMATE_FAN_HIGH,
  });
  t.set_supported_swing_modes({
      climate::CLIMATE_SWING_OFF,
      climate::CLIMATE_SWING_BOTH,
  });
  t.set_visual_min_temperature(16.0f);
  t.set_visual_max_temperature(30.0f);
  t.set_visual_temperature_step(1.0f);
  return t;
}

// ======================= Loop =======================

void ACHiClimate::loop() {
  // RX collect
  while (this->available()) {
    uint8_t b = this->read();
    this->rb_push_(b);
  }
  // Parse frames
  while (this->parse_next_frame_()) {}

  // Periodic status request
  const uint32_t now = millis();
  if (now - this->last_poll_ >= this->update_interval_ms_) {
    this->last_poll_ = now;
    this->send_status_request_();
  }
}

// ======================= Control =======================

void ACHiClimate::control(const climate::ClimateCall &call) {
  if (!have_status_template_) {
    ESP_LOGW(TAG, "Write ignored until first status template is received");
    return;
  }

  bool need_write = false;

  if (call.get_mode().has_value()) {
    auto m = *call.get_mode();
    if (m == climate::CLIMATE_MODE_OFF) {
      power_bin_ = 0b00000100; // keep bit2, clear bit3
      this->mode = climate::CLIMATE_MODE_OFF;
      this->power_ = false;
    } else {
      power_bin_ = 0b00001100; // bit2 + bit3
      this->mode = m;
      this->power_ = true;

      uint8_t idx = 4; // auto
      if (m == climate::CLIMATE_MODE_HEAT) idx = 1;
      else if (m == climate::CLIMATE_MODE_COOL) idx = 2;
      else if (m == climate::CLIMATE_MODE_DRY) idx = 3;
      else if (m == climate::CLIMATE_MODE_FAN_ONLY) idx = 0;
      mode_bin_ = uint8_t((((idx << 1) | 0x01) << 4));
    }
    need_write = true;
  }

  if (call.get_target_temperature().has_value()) {
    float t = *call.get_target_temperature();
    if (t < 16.0f) t = 16.0f;
    if (t > 30.0f) t = 30.0f;
    this->target_temperature = t;
    temp_byte_ = (uint8_t(t) << 1) | 0x01;
    need_write = true;
  }

  if (call.get_fan_mode().has_value()) {
    auto f = *call.get_fan_mode();
    this->fan_mode = f;
    // map to wind code used by protocol
    if (f == climate::CLIMATE_FAN_AUTO)        wind_code_ = 1;
    else if (f == climate::CLIMATE_FAN_LOW)    wind_code_ = 12;
    else if (f == climate::CLIMATE_FAN_MEDIUM) wind_code_ = 14;
    else if (f == climate::CLIMATE_FAN_HIGH)   wind_code_ = 16;
    need_write = true;
  }

  if (call.get_swing_mode().has_value()) {
    auto s = *call.get_swing_mode();
    swing_ud_ = (s == climate::CLIMATE_SWING_BOTH);
    swing_lr_ = (s == climate::CLIMATE_SWING_BOTH);
    need_write = true;
  }

  if (need_write) {
    this->send_write_frame_();
  }

  this->publish_state();
}

// ======================= Protocol I/O =======================

void ACHiClimate::send_status_request_() {
  // Short status request (cmd 0x66 / 102) — проверено в legacy
  const uint8_t req[] = {
    0xF4,0xF5,0x00,0x40,0x0C,0x00,0x00,0x01,0x01,0xFE,0x01,0x00,0x00,0x66,0x00,0x00,0x00,0x01,0xB3,0xF4,0xFB
  };
  this->write_array(req, sizeof(req));
  this->flush();
  ESP_LOGVV(TAG, "Sent status request (102)");
}

void ACHiClimate::rebuild_write_frame_from_last_status_() {
  // Start from last status frame as template
  out_ = last_status_;               // copy full frame
  if (out_.size() < 50) out_.resize(50, 0x00);

  // Header/trailer stay as-is from status.
  // Switch command to WRITE (0x65) at [13]
  out_[13] = 0x65;

  // Apply fields
  // [16] wind (protocol uses +1 in writes; status already raw)
  out_[16] = uint8_t(wind_code_ + 1);

  // [18] power + mode
  out_[18] = uint8_t(power_bin_ + mode_bin_);
  if ((power_bin_ & 0b00001000) == 0) {
    out_[18] = uint8_t(out_[18] & (~(1U<<3)));
  }

  // [32] swing (up-down + left-right)
  updown_bin_    = swing_ud_ ? 0b00110000 : 0b00010000;
  leftright_bin_ = swing_lr_ ? 0b00001100 : 0b00000100;
  out_[32] = uint8_t(updown_bin_ + leftright_bin_);

  // [33] turbo + eco (kept off by default)
  out_[33] = uint8_t(turbo_bin_ + eco_bin_);

  // [35] quiet
  out_[35] = quiet_bin_;

  // [19] target temp (0 — when turbo override)
  out_[19] = (turbo_bin_ == 0b00001100) ? 0x00 : temp_byte_;

  // Recompute CRC and ensure proper trailer
  compute_crc_(out_);
}

void ACHiClimate::send_write_frame_() {
  this->rebuild_write_frame_from_last_status_();
  this->write_array(out_.data(), out_.size());
  this->flush();
  ESP_LOGD(TAG, "Sent write frame (cmd 0x65)");
}

void ACHiClimate::compute_crc_(std::vector<uint8_t> &buf) {
  // CRC как в legacy: сумма по [2..len-5] -> [len-4],[len-3]; хвост [len-2]=F4,[len-1]=FB
  if (buf.size() < 8) return;
  const size_t n = buf.size();
  int csum = 0;
  for (size_t i = 2; i < n - 4; i++) csum += buf[i];
  uint8_t cr1 = (csum & 0xFF00) >> 8;
  uint8_t cr2 = (csum & 0x00FF);
  buf[n - 4] = cr1;
  buf[n - 3] = cr2;
  buf[n - 2] = 0xF4;
  buf[n - 1] = 0xFB;
}

// Frames start with F4 F5 and end with F4 FB
bool ACHiClimate::parse_next_frame_() {
  // sync to start
  size_t cnt = 0;
  uint8_t b;
  while (rb_pop_(b)) {
    if (b == 0xF4) {
      uint8_t b2;
      if (!rb_pop_(b2)) { rb_tail_ = (rb_tail_ + RB_SIZE - 1) % RB_SIZE; return false; }
      if (b2 == 0xF5) {
        std::vector<uint8_t> frame;
        frame.push_back(0xF4); frame.push_back(0xF5);
        int guard = 0;
        while (rb_pop_(b)) {
          frame.push_back(b);
          if (frame.size() >= 4 && frame[frame.size()-2] == 0xF4 && frame.back() == 0xFB) {
            if (frame.size() >= 20) handle_status_(frame);
            return true;
          }
          if (++guard > 512) break;
        }
        return false;
      } else {
        // put back b2
        rb_tail_ = (rb_tail_ + RB_SIZE - 1) % RB_SIZE;
      }
    }
    if (++cnt > RB_SIZE) break;
  }
  return false;
}

void ACHiClimate::handle_status_(const std::vector<uint8_t> &bytes) {
  if (bytes.size() < 20) return;

  // Save template if this looks like a full frame (keep original header & trailer)
  if (bytes.size() >= 50 && bytes[13] == 102 /*0x66*/) {
    last_status_ = bytes;
    have_status_template_ = true;
  }

  // We only care about cmd=102 (status)
  if (bytes[13] != 102) {
    // 101 — unlock after write; ignore for state, but accept as acks
    return;
  }

  // Power (bit3 in [18])
  bool new_power = (bytes[18] & 0b00001000) != 0;
  // Mode — high nibble of [18]
  uint8_t mode_raw = (bytes[18] >> 4) & 0x0F;
  climate::ClimateMode new_mode = climate::CLIMATE_MODE_AUTO;
  switch (mode_raw) {
    case 0: new_mode = climate::CLIMATE_MODE_FAN_ONLY; break;
    case 1: new_mode = climate::CLIMATE_MODE_HEAT;     break;
    case 2: new_mode = climate::CLIMATE_MODE_COOL;     break;
    case 3: new_mode = climate::CLIMATE_MODE_DRY;      break;
    default: new_mode = climate::CLIMATE_MODE_AUTO;    break;
  }

  // Fan — [16]
  uint8_t wind_raw = bytes[16];
  climate::ClimateFanMode new_fan = climate::CLIMATE_FAN_AUTO;
  if (wind_raw == 12) new_fan = climate::CLIMATE_FAN_LOW;
  else if (wind_raw == 14) new_fan = climate::CLIMATE_FAN_MEDIUM;
  else if (wind_raw == 16) new_fan = climate::CLIMATE_FAN_HIGH;
  else if (wind_raw == 1)  new_fan = climate::CLIMATE_FAN_AUTO;

  // Temperatures — setpoint and current
  uint8_t tset = bytes[19];
  uint8_t tcur = bytes[20];

  // Swing flags — legacy mapping: up/down bit7, left/right bit6 at [35]
  bool ud    = (bytes[35] & 0b10000000) != 0;
  bool lr    = (bytes[35] & 0b01000000) != 0;

  // Publish into Climate
  this->current_temperature = float(tcur);
  this->target_temperature  = float(tset);
  this->mode      = new_power ? new_mode : climate::CLIMATE_MODE_OFF;
  this->fan_mode  = new_fan;
  this->swing_mode = (ud || lr) ? climate::CLIMATE_SWING_BOTH : climate::CLIMATE_SWING_OFF;

  // Mirror to internal state
  this->power_ = new_power;
  this->mode_  = new_mode;
  this->fan_   = new_fan;
  this->swing_ud_ = ud;
  this->swing_lr_ = lr;
  this->room_temp_   = float(tcur);
  this->target_temp_ = float(tset);

  this->publish_state();
}

}  // namespace ac_hi
}  // namespace esphome
