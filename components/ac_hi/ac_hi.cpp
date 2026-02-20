#include "ac_hi.h"
#include <cmath>
#include <cstdio>
#include <Arduino.h>
#include "esphome/core/log.h"

namespace esphome {
namespace ac_hi {

static const char *const TAG = "ac_hi";

// ---- Local helpers for mode encoding/decoding ----
static inline climate::ClimateMode decode_mode_from_nibble(uint8_t nib) {
  switch (nib & 0x0F) {
    case 0x00: return climate::CLIMATE_MODE_FAN_ONLY;
    case 0x01: return climate::CLIMATE_MODE_HEAT;
    case 0x02: return climate::CLIMATE_MODE_COOL;
    case 0x03: return climate::CLIMATE_MODE_DRY;
    default:   return climate::CLIMATE_MODE_COOL;
  }
}

static inline uint8_t encode_nibble_from_mode(climate::ClimateMode m) {
  switch (m) {
    case climate::CLIMATE_MODE_FAN_ONLY: return 0x01;
    case climate::CLIMATE_MODE_HEAT:     return 0x03;
    case climate::CLIMATE_MODE_COOL:     return 0x05;
    case climate::CLIMATE_MODE_DRY:      return 0x07;
    default:                             return 0x05;
  }
}

// ---- ACHILEDTargetSwitch ----
void ACHILEDTargetSwitch::write_state(bool state) {
  if (parent_ != nullptr) {
    parent_->set_desired_led(state);
  }
  publish_state(state);
}

// ---- ACHIClimate implementation ----

void ACHIClimate::setup() {
  // Initial HA‑visible state
  mode = climate::CLIMATE_MODE_OFF;
  target_temperature = 24;
  fan_mode = climate::CLIMATE_FAN_AUTO;
  swing_mode = climate::CLIMATE_SWING_OFF;

  // Desired state mirrors initial
  d_power_on_     = false;
  d_mode_         = climate::CLIMATE_MODE_OFF;
  d_target_c_     = 24;
  d_fan_          = climate::CLIMATE_FAN_AUTO;
  d_swing_        = climate::CLIMATE_SWING_OFF;
  d_turbo_        = false;
  d_eco_          = false;
  d_quiet_        = false;
  d_led_          = true;
  d_sleep_stage_  = 0;

  recalc_desired_sig_();
  recalc_actual_sig_();

  publish_state();
  update_led_switch_state_();

  // Pre‑reserve buffers to avoid reallocations
  rx_.reserve(RX_BUFFER_RESERVE);
  last_status_frame_.reserve(MAX_FRAME_BYTES);
  last_tx_frame_.reserve(MAX_FRAME_BYTES);

  ESP_LOGI(TAG, "Setup complete");
}

void ACHIClimate::update() {
  if (!writing_lock_) {
    send_query_status_();
  } else {
    ESP_LOGV(TAG, "Polling skipped (write lock active)");
  }
}

void ACHIClimate::loop() {
  // 1. Accumulate incoming bytes
  uint8_t c;
  while (read_byte(&c)) {
    rx_.push_back(c);
  }

  // 2. Compact RX buffer if too much data has been consumed
  if (rx_start_ > RX_COMPACT_THRESHOLD) {
    ESP_LOGV(TAG, "Compacting RX buffer: removing %u bytes", (unsigned) rx_start_);
    rx_.erase(rx_.begin(), rx_.begin() + static_cast<std::ptrdiff_t>(rx_start_));
    rx_start_ = 0;
  }

  // 3. Prevent RX buffer from growing indefinitely
  if (rx_.size() - rx_start_ > 4096) {
    ESP_LOGW(TAG, "RX buffer overflow, clearing");
    rx_.clear();
    rx_start_ = 0;
  }

  // 4. Parse incoming frames (time‑bounded)
  try_parse_frames_from_buffer_(MAX_PARSE_TIME_MS);

  // 5. Handle write lock timeout
  if (writing_lock_ && (millis() - write_lock_time_ > WRITE_LOCK_TIMEOUT)) {
    ESP_LOGW(TAG, "Write lock timeout, forcing unlock");
    writing_lock_ = false;
  }

  // 6. Send pending control command if debounce period passed and no lock
  if (pending_control_ && !writing_lock_ &&
      (millis() - last_control_ms_ >= CONTROL_DEBOUNCE_MS)) {
    send_write_changes_();
    pending_control_ = false;
  }

  // 7. Optional memory diagnostics
  publish_memory_diagnostics_();
}

// ---- Climate traits ----
climate::ClimateTraits ACHIClimate::traits() {
  climate::ClimateTraits t{};
  t.set_supports_action(false);
  t.set_supported_modes({
    climate::CLIMATE_MODE_OFF,
    climate::CLIMATE_MODE_COOL,
    climate::CLIMATE_MODE_HEAT,
    climate::CLIMATE_MODE_DRY,
    climate::CLIMATE_MODE_FAN_ONLY
  });
  t.set_supported_fan_modes({climate::CLIMATE_FAN_AUTO, climate::CLIMATE_FAN_LOW,
                             climate::CLIMATE_FAN_MEDIUM, climate::CLIMATE_FAN_HIGH,
                             climate::CLIMATE_FAN_QUIET});
  t.set_supported_swing_modes({climate::CLIMATE_SWING_OFF, climate::CLIMATE_SWING_VERTICAL,
                               climate::CLIMATE_SWING_HORIZONTAL, climate::CLIMATE_SWING_BOTH});
  if (enable_presets_) {
    t.set_supported_presets({climate::CLIMATE_PRESET_NONE, climate::CLIMATE_PRESET_ECO,
                             climate::CLIMATE_PRESET_BOOST, climate::CLIMATE_PRESET_SLEEP});
  }
  t.set_visual_min_temperature(16);
  t.set_visual_max_temperature(30);
  t.set_visual_temperature_step(1.0f);
  t.set_supports_current_temperature(true);
  return t;
}

// ---- Control from HA ----
void ACHIClimate::control(const climate::ClimateCall &call) {
  bool changed = false;

  if (call.get_mode().has_value()) {
    auto m = *call.get_mode();
    if (m == climate::CLIMATE_MODE_OFF) {
      d_power_on_ = false;
      d_mode_ = climate::CLIMATE_MODE_COOL;   // fallback, power off overrides
    } else {
      d_power_on_ = true;
      d_mode_ = m;
    }
    changed = true;
  }

  if (call.get_target_temperature().has_value()) {
    float t = *call.get_target_temperature();
    if (!std::isnan(t)) {
      uint8_t c = static_cast<uint8_t>(std::round(t));
      c = std::max<uint8_t>(16, std::min<uint8_t>(30, c));
      d_target_c_ = c;
      changed = true;
    }
  }

  if (call.get_fan_mode().has_value()) {
    d_fan_ = *call.get_fan_mode();
    d_quiet_ = (d_fan_ == climate::CLIMATE_FAN_QUIET);
    changed = true;
  }

  if (call.get_swing_mode().has_value()) {
    d_swing_ = *call.get_swing_mode();
    changed = true;
  }

  if (call.get_preset().has_value()) {
    auto p = *call.get_preset();
    d_eco_ = (p == climate::CLIMATE_PRESET_ECO);
    d_turbo_ = (p == climate::CLIMATE_PRESET_BOOST);
    d_sleep_stage_ = (p == climate::CLIMATE_PRESET_SLEEP) ? 1 : 0;
    changed = true;
  }

  if (!changed) return;

  // HA takes priority over remote changes
  accept_remote_changes_ = false;
  ha_priority_active_ = true;

  recalc_desired_sig_();

  // Publish optimistically
  this->mode = d_power_on_ ? d_mode_ : climate::CLIMATE_MODE_OFF;
  this->target_temperature = d_target_c_;
  this->fan_mode = d_fan_;
  this->swing_mode = d_swing_;
  if (enable_presets_) {
    if (d_turbo_) this->preset = climate::CLIMATE_PRESET_BOOST;
    else if (d_eco_) this->preset = climate::CLIMATE_PRESET_ECO;
    else if (d_sleep_stage_ > 0) this->preset = climate::CLIMATE_PRESET_SLEEP;
    else this->preset = climate::CLIMATE_PRESET_NONE;
  }
  publish_state();
  update_led_switch_state_();

  // Mark that we have a pending command (debounced)
  pending_control_ = true;
  last_control_ms_ = millis();

  ESP_LOGD(TAG, "Control: new desired state registered, will send after %ums debounce", CONTROL_DEBOUNCE_MS);
}

// ---- Build TX frame from desired state ----
void ACHIClimate::build_tx_from_desired_() {
  // Power + mode (byte 18)
  uint8_t power_bin = d_power_on_ ? 0b00001100 : 0b00000100;
  uint8_t mode_hi = encode_mode_hi_nibble_(d_mode_); // returns nibble << 4
  tx_bytes_[IDX_POWER_MODE] = power_bin + mode_hi;

  // Target temperature (encoded)
  tx_bytes_[IDX_SET_TEMP] = encode_temp_(d_target_c_);

  // Fan speed (byte 16)
  tx_bytes_[IDX_WIND] = encode_fan_byte_(d_fan_);

  // Sleep mode (byte 17)
  tx_bytes_[IDX_SLEEP] = encode_sleep_byte_(d_sleep_stage_);

  // Swing (byte 32)
  bool v = (d_swing_ == climate::CLIMATE_SWING_VERTICAL) || (d_swing_ == climate::CLIMATE_SWING_BOTH);
  bool h = (d_swing_ == climate::CLIMATE_SWING_HORIZONTAL) || (d_swing_ == climate::CLIMATE_SWING_BOTH);
  uint8_t updown = v ? TxValues::UPDOWN_ON : TxValues::UPDOWN_OFF;
  uint8_t leftright = h ? TxValues::LEFTRIGHT_ON : TxValues::LEFTRIGHT_OFF;
  tx_bytes_[IDX_SWING] = updown + leftright;

  // Turbo, Eco, Quiet (bytes 33 and 35) – Turbo has priority
  if (d_turbo_) {
    tx_bytes_[IDX_TURBO_ECO] = TxValues::TURBO_ON;
    tx_bytes_[IDX_QUIET] = TxValues::QUIET_OFF;   // Turbo disables quiet
  } else {
    tx_bytes_[IDX_TURBO_ECO] = TxValues::TURBO_OFF + (d_eco_ ? TxValues::ECO_ON : TxValues::ECO_OFF);
    tx_bytes_[IDX_QUIET] = d_quiet_ ? TxValues::QUIET_ON : TxValues::QUIET_OFF;
  }

  // LED (byte 36)
  tx_bytes_[IDX_LED] = d_led_ ? TxValues::LED_ON : TxValues::LED_OFF;
}

// ---- Send status query ----
void ACHIClimate::send_query_status_() {
  ESP_LOGD(TAG, "Sending status query (0x66)");
  log_frame_("TX query", query_);
  for (auto b : query_) write_byte(b);
  flush();
}

// ---- Send write command ----
void ACHIClimate::send_write_changes_() {
  build_tx_from_desired_();                // ensure tx_bytes_ reflects latest d_*
  calc_and_patch_crc_(tx_bytes_);
  ESP_LOGD(TAG, "Sending write command (0x65)");
  log_frame_("TX write", tx_bytes_);
  for (auto b : tx_bytes_) write_byte(b);
  flush();

  last_tx_frame_.assign(tx_bytes_.begin(), tx_bytes_.end());

  writing_lock_ = true;
  write_lock_time_ = millis();
}

// ---- CRC calculation ----
void ACHIClimate::calc_and_patch_crc_(std::vector<uint8_t> &buf) {
  size_t n = buf.size();
  uint16_t csum = 0;
  for (size_t i = 2; i < n - 4; i++) csum = static_cast<uint16_t>(csum + buf[i]);
  buf[n - 4] = static_cast<uint8_t>((csum >> 8) & 0xFF);
  buf[n - 3] = static_cast<uint8_t>(csum & 0xFF);
}

bool ACHIClimate::validate_crc_(const std::vector<uint8_t> &buf, uint16_t *out_sum) const {
  if (buf.size() < 8) return false;
  size_t n = buf.size();
  uint16_t csum = 0;
  for (size_t i = 2; i < n - 4; i++) csum = static_cast<uint16_t>(csum + buf[i]);
  if (out_sum) *out_sum = csum;
  return (buf[n - 4] == ((csum >> 8) & 0xFF)) && (buf[n - 3] == (csum & 0xFF));
}

// ---- RX frame parsing ----
void ACHIClimate::try_parse_frames_from_buffer_(uint32_t budget_ms) {
  std::vector<uint8_t> frame;
  frame.reserve(MAX_FRAME_BYTES);

  uint8_t handled = 0;
  uint32_t start = millis();

  while (handled < MAX_FRAMES_PER_LOOP &&
         (millis() - start) < budget_ms &&
         extract_next_frame_(frame)) {

    log_frame_("RX", frame);

    uint16_t sum = 0;
    if (!validate_crc_(frame, &sum)) {
      ESP_LOGW(TAG, "CRC mismatch, ignoring frame");
      continue;                     // drop invalid frame
    }

    handle_frame_(frame);
    handled++;
    last_status_crc_ = sum;
  }
}

bool ACHIClimate::extract_next_frame_(std::vector<uint8_t> &frame) {
  frame.clear();
  if (rx_.size() <= rx_start_ + 5) return false;

  // Find header F4 F5
  size_t i = rx_start_;
  bool found = false;
  for (; i + 1 < rx_.size(); i++) {
    if (rx_[i] == HI_HDR0 && rx_[i + 1] == HI_HDR1) {
      found = true;
      break;
    }
  }
  if (!found) {
    // No header in remaining data – keep only possible trailing header byte
    if (!rx_.empty() && rx_.back() == HI_HDR0) {
      uint8_t keep = rx_.back();
      rx_.clear();
      rx_.push_back(keep);
      rx_start_ = 0;
    } else {
      rx_.clear();
      rx_start_ = 0;
    }
    return false;
  }
  rx_start_ = i;

  // Try to slice by declared length (byte 4)
  if (rx_.size() > rx_start_ + 5) {
    uint8_t decl = rx_[rx_start_ + 4];
    size_t expected = static_cast<size_t>(decl) + 9;
    if (rx_.size() >= rx_start_ + expected) {
      frame.assign(rx_.begin() + rx_start_, rx_.begin() + rx_start_ + expected);
      rx_start_ += expected;
      return true;
    }
  }

  // Otherwise search for tail F4 FB
  size_t j = rx_start_ + 2;
  for (; j + 1 < rx_.size(); j++) {
    if (rx_[j] == HI_TAIL0 && rx_[j + 1] == HI_TAIL1) {
      frame.assign(rx_.begin() + rx_start_, rx_.begin() + j + 2);
      rx_start_ = j + 2;
      return true;
    }
  }
  return false;
}

void ACHIClimate::handle_frame_(const std::vector<uint8_t> &b) {
  if (b.size() < 20) {
    ESP_LOGD(TAG, "Frame too short (%u), ignored", (unsigned) b.size());
    return;
  }
  uint8_t cmd = b[IDX_CMD];
  if (cmd == 102) {          // 0x66 – status response
    parse_status_102_(b);
  } else if (cmd == 101) {   // 0x65 – write acknowledge
    handle_ack_101_();
  } else {
    ESP_LOGV(TAG, "Unknown command 0x%02X, ignored", cmd);
  }
}

void ACHIClimate::parse_status_102_(const std::vector<uint8_t> &b) {
  last_status_frame_.assign(b.begin(), b.end());

  // Ensure we have all expected bytes
  if (b.size() < 46) {
    ESP_LOGE(TAG, "Status frame too short (%u), cannot parse fully", (unsigned) b.size());
    return;
  }

  // Power
  power_on_ = (b[IDX_POWER_MODE] & POWER_MASK) != 0;

  // Mode (upper nibble)
  uint8_t nib = (b[IDX_POWER_MODE] >> 4) & 0x0F;
  mode_ = decode_mode_from_nibble(nib);

  // Fan speed
  uint8_t raw_wind = b[IDX_WIND];
  if (power_on_) {
    if (raw_wind == 0 || raw_wind == 1 || raw_wind == 2) fan_ = climate::CLIMATE_FAN_AUTO;
    else if (raw_wind == 10) fan_ = climate::CLIMATE_FAN_QUIET;
    else if (raw_wind == 12) fan_ = climate::CLIMATE_FAN_LOW;
    else if (raw_wind == 14) fan_ = climate::CLIMATE_FAN_MEDIUM;
    else if (raw_wind == 16) fan_ = climate::CLIMATE_FAN_HIGH;
    else fan_ = climate::CLIMATE_FAN_AUTO;
  } else {
    fan_ = climate::CLIMATE_FAN_AUTO;   // when off, fan mode is irrelevant
  }

  // Sleep stage
  uint8_t raw_sleep = b[IDX_SLEEP];
  uint8_t code = raw_sleep >> 1;
  if (code == 0) sleep_stage_ = 0;
  else if (code == 1) sleep_stage_ = 1;
  else if (code == 2) sleep_stage_ = 2;
  else if (code == 4) sleep_stage_ = 3;
  else if (code == 8) sleep_stage_ = 4;
  else sleep_stage_ = 0;

  // Target temperature (direct value)
  target_c_ = b[IDX_SET_TEMP];
  if (target_c_ < 16 || target_c_ > 30) target_c_ = 24; // fallback

  // Current temperatures
  current_temperature = b[IDX_CURRENT_TEMP];

#ifdef USE_SENSOR
  if (pipe_sensor_ != nullptr) pipe_sensor_->publish_state(b[IDX_PIPE_TEMP]);
#endif

  // Turbo, Eco, Quiet, LED
  uint8_t b35 = b[IDX_TURBO_ECO];   // byte 35 (turbo/eco flags)
  turbo_ = (b35 & TURBO_MASK) != 0;
  eco_   = (b35 & ECO_MASK) != 0;
  quiet_ = (b[IDX_QUIET] & QUIET_MASK) != 0;   // byte 36
  led_   = (b[IDX_LED] & LED_MASK) != 0;       // byte 37

  // Swing
  bool updown = (b35 & UPDOWN_MASK) != 0;
  bool leftright = (b35 & LEFTRIGHT_MASK) != 0;
  if (updown && leftright) swing_ = climate::CLIMATE_SWING_BOTH;
  else if (updown) swing_ = climate::CLIMATE_SWING_VERTICAL;
  else if (leftright) swing_ = climate::CLIMATE_SWING_HORIZONTAL;
  else swing_ = climate::CLIMATE_SWING_OFF;

  // Recalculate actual signature
  recalc_actual_sig_();

  // Publish state with gating
  publish_gated_state_();
  update_led_switch_state_();

  // Publish optional sensors (with sign conversion for outdoor temperatures)
#ifdef USE_SENSOR
  if (set_temp_sensor_ != nullptr) set_temp_sensor_->publish_state(b[IDX_SET_TEMP]);
  if (room_temp_sensor_ != nullptr) room_temp_sensor_->publish_state(b[IDX_CURRENT_TEMP]);
  if (wind_code_sensor_ != nullptr) wind_code_sensor_->publish_state(b[IDX_WIND]);
  if (sleep_code_sensor_ != nullptr) sleep_code_sensor_->publish_state(b[IDX_SLEEP]);
  if (mode_code_sensor_ != nullptr) mode_code_sensor_->publish_state((b[IDX_POWER_MODE] >> 4) & 0x0F);
  if (quiet_code_sensor_ != nullptr) quiet_code_sensor_->publish_state(quiet_ ? 1.0f : 0.0f);
  if (turbo_code_sensor_ != nullptr) turbo_code_sensor_->publish_state(turbo_ ? 1.0f : 0.0f);
  if (eco_code_sensor_ != nullptr)   eco_code_sensor_->publish_state(eco_ ? 1.0f : 0.0f);
  if (swing_ud_sensor_ != nullptr)   swing_ud_sensor_->publish_state(updown ? 1.0f : 0.0f);
  if (swing_lr_sensor_ != nullptr)   swing_lr_sensor_->publish_state(leftright ? 1.0f : 0.0f);
  if (compressor_freq_set_sensor_ != nullptr) compressor_freq_set_sensor_->publish_state(b[IDX_COMP_FREQ_SET]);
  if (compressor_freq_sensor_ != nullptr)     compressor_freq_sensor_->publish_state(b[IDX_COMP_FREQ]);

  // Outdoor temperatures are signed!
  if (outdoor_temp_sensor_ != nullptr) {
    int8_t t = static_cast<int8_t>(b[IDX_OUTDOOR_TEMP]);
    outdoor_temp_sensor_->publish_state(static_cast<float>(t));
  }
  if (outdoor_cond_temp_sensor_ != nullptr) {
    int8_t t = static_cast<int8_t>(b[IDX_OUTDOOR_COND_TEMP]);
    outdoor_cond_temp_sensor_->publish_state(static_cast<float>(t));
  }
#endif

#ifdef USE_TEXT_SENSOR
  if (power_status_text_ != nullptr) {
    power_status_text_->publish_state(power_on_ ? "ON" : "OFF");
  }
#endif

  ESP_LOGD(TAG,
           "Parsed: power=%s, mode=%s, fan=%s, swing=%s, target=%u°C, current=%.1f°C, outdoor=%d°C",
           power_on_ ? "ON" : "OFF",
           climate::climate_mode_to_string(mode_),
           climate::climate_fan_mode_to_string(fan_),
           climate::climate_swing_mode_to_string(swing_),
           (unsigned) target_c_, current_temperature,
           static_cast<int8_t>(b[IDX_OUTDOOR_TEMP]));

  // If HA has priority, check convergence and possibly enforce
  maybe_force_to_target_();
}

void ACHIClimate::handle_ack_101_() {
  writing_lock_ = false;
  ESP_LOGD(TAG, "Write acknowledged (lock cleared)");

  // If there is a pending control command, it will be sent on next loop
  // (after debounce) because pending_control_ is still true.
}

// ---- Gating and convergence ----
void ACHIClimate::publish_gated_state_() {
  if (accept_remote_changes_) {
    // Publish actual state (from AC)
    this->mode = power_on_ ? mode_ : climate::CLIMATE_MODE_OFF;
    this->target_temperature = target_c_;
    this->fan_mode = fan_;
    this->swing_mode = swing_;
    if (enable_presets_) {
      if (turbo_) this->preset = climate::CLIMATE_PRESET_BOOST;
      else if (eco_) this->preset = climate::CLIMATE_PRESET_ECO;
      else if (sleep_stage_ > 0) this->preset = climate::CLIMATE_PRESET_SLEEP;
      else this->preset = climate::CLIMATE_PRESET_NONE;
    }
    // Sync desired with actual
    d_power_on_    = power_on_;
    d_mode_        = mode_;
    d_target_c_    = target_c_;
    d_fan_         = fan_;
    d_swing_       = swing_;
    d_eco_         = eco_;
    d_turbo_       = turbo_;
    d_quiet_       = quiet_;
    d_led_         = led_;
    d_sleep_stage_ = sleep_stage_;
    recalc_desired_sig_();
  } else {
    // Publish desired state (while enforcing)
    this->mode = d_power_on_ ? d_mode_ : climate::CLIMATE_MODE_OFF;
    this->target_temperature = d_target_c_;
    this->fan_mode = d_fan_;
    this->swing_mode = d_swing_;
    if (enable_presets_) {
      if (d_turbo_) this->preset = climate::CLIMATE_PRESET_BOOST;
      else if (d_eco_) this->preset = climate::CLIMATE_PRESET_ECO;
      else if (d_sleep_stage_ > 0) this->preset = climate::CLIMATE_PRESET_SLEEP;
      else this->preset = climate::CLIMATE_PRESET_NONE;
    }
  }
  publish_state();
}

void ACHIClimate::update_led_switch_state_() {
  if (led_switch_ == nullptr) return;
  led_switch_->publish_state(d_led_);
}

void ACHIClimate::maybe_force_to_target_() {
  if (!ha_priority_active_) return;

  if (actual_sig_ == desired_sig_) {
    ha_priority_active_ = false;
    accept_remote_changes_ = true;
    ESP_LOGI(TAG, "Converged to desired HA state; remote changes accepted again");
    return;
  }

  if (!writing_lock_ && pending_control_) {
    // There is already a pending command that will be sent soon, no need to duplicate
    return;
  }

  if (!writing_lock_) {
    ESP_LOGD(TAG, "Enforcing desired state (actual≠desired) – requesting write");
    pending_control_ = true;
    last_control_ms_ = millis();   // restart debounce
  }
}

// ---- Signature computation ----
uint32_t ACHIClimate::compute_control_signature_(bool power, climate::ClimateMode mode,
                                                 climate::ClimateFanMode fan, climate::ClimateSwingMode swing,
                                                 bool eco, bool turbo, bool quiet, bool led,
                                                 uint8_t sleep_stage, uint8_t target_c) const {
  uint32_t h = 2166136261u;
  auto mix = [&h](uint32_t x) {
    h ^= x;
    h *= 16777619u;
  };
  mix(power ? 1u : 0u);
  mix(static_cast<uint32_t>(mode));
  mix(static_cast<uint32_t>(fan));
  mix(static_cast<uint32_t>(swing));
  mix(eco ? 1u : 0u);
  mix(turbo ? 1u : 0u);
  mix(quiet ? 1u : 0u);
  mix(led ? 1u : 0u);
  mix(static_cast<uint32_t>(sleep_stage & 0x0Fu));
  mix(static_cast<uint32_t>(std::max<uint8_t>(16, std::min<uint8_t>(30, target_c))));
  return h;
}

void ACHIClimate::recalc_desired_sig_() {
  desired_sig_ = compute_control_signature_(d_power_on_, d_mode_, d_fan_, d_swing_,
                                            d_eco_, d_turbo_, d_quiet_, d_led_, d_sleep_stage_, d_target_c_);
}

void ACHIClimate::recalc_actual_sig_() {
  actual_sig_ = compute_control_signature_(power_on_, mode_, fan_, swing_,
                                           eco_, turbo_, quiet_, led_, sleep_stage_, target_c_);
}

void ACHIClimate::log_sig_diff_() const {
  // Optional verbose diff – can be enabled for debugging
}

// ---- External LED control ----
void ACHIClimate::set_desired_led(bool on) {
  d_led_ = on;
  accept_remote_changes_ = false;
  ha_priority_active_ = true;
  recalc_desired_sig_();

  pending_control_ = true;
  last_control_ms_ = millis();

  ESP_LOGD(TAG, "LED switch: desired_led=%s, pending write", on ? "ON" : "OFF");
  // update_led_switch_state_() will be called from loop after publish
}

// ---- Field encoders ----
uint8_t ACHIClimate::encode_mode_hi_nibble_(climate::ClimateMode m) {
  return static_cast<uint8_t>(encode_nibble_from_mode(m) << 4);
}

uint8_t ACHIClimate::encode_fan_byte_(climate::ClimateFanMode f) {
  uint8_t code = 1;
  switch (f) {
    case climate::CLIMATE_FAN_AUTO:   code = 1;  break;
    case climate::CLIMATE_FAN_LOW:    code = 12; break;
    case climate::CLIMATE_FAN_MEDIUM: code = 14; break;
    case climate::CLIMATE_FAN_HIGH:   code = 16; break;
    case climate::CLIMATE_FAN_QUIET:  code = 10; break;
    default:                          code = 1;  break;
  }
  return static_cast<uint8_t>(code + 1);   // writing requires +1
}

uint8_t ACHIClimate::encode_sleep_byte_(uint8_t stage) {
  uint8_t code = 0;
  switch (stage) {
    case 1: code = 1; break;
    case 2: code = 2; break;
    case 3: code = 4; break;
    case 4: code = 8; break;
    default: code = 0; break;
  }
  return static_cast<uint8_t>((code << 1) | 0x01);
}

// ---- Logging helper ----
void ACHIClimate::log_frame_(const char *prefix, const std::vector<uint8_t> &b) const {
  const size_t n = b.size();
  char header[64];
  snprintf(header, sizeof(header), "%s (%u bytes)", prefix, (unsigned) n);
  ESP_LOGV(TAG, "%s", header);
  for (size_t i = 0; i < n; i += 16) {
    char line[64];
    char *p = line;
    size_t remain = n - i;
    size_t chunk = remain < 16 ? remain : 16;
    for (size_t j = 0; j < chunk; j++) {
      p += snprintf(p, sizeof(line) - (p - line), "%02X ", b[i + j]);
    }
    ESP_LOGV(TAG, "  %s", line);
  }
}

// ---- Memory diagnostics ----
void ACHIClimate::publish_memory_diagnostics_() {
#ifdef USE_SENSOR
  static uint32_t last_ms = 0;
  uint32_t now = millis();
  if (now - last_ms < MEM_PUBLISH_INTERVAL_MS) return;
  last_ms = now;

  // Gather metrics (simplified, no heavy allocation)
  size_t heap_free = ESP.getFreeHeap();
  size_t heap_total = 0, heap_used = 0, heap_min_free = 0, heap_max_alloc = 0;
  int heap_frag_pct = -1;
  size_t psram_total = 0, psram_free = 0;

#if defined(ARDUINO_ARCH_ESP32)
  heap_total     = ESP.getHeapSize();
  heap_min_free  = ESP.getMinFreeHeap();
  heap_max_alloc = ESP.getMaxAllocHeap();
  psram_total    = ESP.getPsramSize();
  psram_free     = ESP.getFreePsram();
  if (heap_total > heap_free) heap_used = heap_total - heap_free;
  if (heap_free > 0 && heap_max_alloc > 0) {
    double ratio = 1.0 - static_cast<double>(heap_max_alloc) / static_cast<double>(heap_free);
    if (ratio < 0.0) ratio = 0.0;
    if (ratio > 1.0) ratio = 1.0;
    heap_frag_pct = static_cast<int>(std::lround(ratio * 100.0));
  }
#elif defined(ARDUINO_ARCH_ESP8266)
  heap_max_alloc = ESP.getMaxFreeBlockSize();
  heap_frag_pct  = ESP.getHeapFragmentation();
#endif

  if (heap_free_sensor_ != nullptr) heap_free_sensor_->publish_state(static_cast<float>(heap_free));
  if (heap_total_sensor_ != nullptr && heap_total > 0) heap_total_sensor_->publish_state(static_cast<float>(heap_total));
  if (heap_used_sensor_ != nullptr && heap_total > 0) heap_used_sensor_->publish_state(static_cast<float>(heap_used));
  if (heap_min_free_sensor_ != nullptr && heap_min_free > 0) heap_min_free_sensor_->publish_state(static_cast<float>(heap_min_free));
  if (heap_max_alloc_sensor_ != nullptr && heap_max_alloc > 0) heap_max_alloc_sensor_->publish_state(static_cast<float>(heap_max_alloc));
  if (heap_fragmentation_sensor_ != nullptr && heap_frag_pct >= 0) heap_fragmentation_sensor_->publish_state(static_cast<float>(heap_frag_pct));
  if (psram_total_sensor_ != nullptr && psram_total > 0) psram_total_sensor_->publish_state(static_cast<float>(psram_total));
  if (psram_free_sensor_ != nullptr && psram_free > 0) psram_free_sensor_->publish_state(static_cast<float>(psram_free));
#endif
}

}  // namespace ac_hi
}  // namespace esphome