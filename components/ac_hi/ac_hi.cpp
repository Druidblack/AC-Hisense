#include "ac_hi.h"
#include <cmath>
#include <algorithm>
#include <string>
#include <cstdio>   // snprintf
#include <Arduino.h>  // ESP heap APIs
#include "esphome/core/log.h"

namespace esphome {
namespace ac_hi {

static const char *const TAG = "ac_hi";

// ---- Local helpers for (de)encoding mode ----
// Target nibble layout (byte[18] >> 4) on RX:
// 0x00 = FAN_ONLY, 0x01 = HEAT, 0x02 = COOL, 0x03 = DRY.
// There's no AUTO here — it's not supported and not exposed to HA.
static inline climate::ClimateMode decode_mode_from_nibble(uint8_t nib) {
  switch (nib & 0x0F) {
    case 0x00: return climate::CLIMATE_MODE_FAN_ONLY;
    case 0x01: return climate::CLIMATE_MODE_HEAT;
    case 0x02: return climate::CLIMATE_MODE_COOL;
    case 0x03: return climate::CLIMATE_MODE_DRY;
    default:   return climate::CLIMATE_MODE_COOL;  // fallback
  }
}
static inline uint8_t encode_nibble_from_mode(climate::ClimateMode m) {
  // TX uses odd nibble codes (protocol quirk). Do not change mapping.
  switch (m) {
    case climate::CLIMATE_MODE_FAN_ONLY: return 0x01;
    case climate::CLIMATE_MODE_HEAT:     return 0x03;
    case climate::CLIMATE_MODE_COOL:     return 0x05;
    case climate::CLIMATE_MODE_DRY:      return 0x07;
    default:                             return 0x05; // no AUTO, use COOL
  }
}

// ---- ACHILEDTargetSwitch ----
void ACHILEDTargetSwitch::write_state(bool state) {
  // Forward the desired LED state to the climate parent and acknowledge
  if (this->parent_ != nullptr) {
    this->parent_->set_desired_led(state);
  }
  this->publish_state(state);
}

void ACHIClimate::setup() {
  // Initialize default HA visible state
  this->mode = climate::CLIMATE_MODE_OFF;
  this->target_temperature = 24;
  this->fan_mode = climate::CLIMATE_FAN_AUTO;
  this->swing_mode = climate::CLIMATE_SWING_OFF;

  // Initialize desired_* to match defaults
  d_power_on_     = false;
  d_mode_         = climate::CLIMATE_MODE_OFF;
  d_target_c_     = 24;
  d_fan_          = climate::CLIMATE_FAN_AUTO;
  d_swing_        = climate::CLIMATE_SWING_OFF;
  d_turbo_        = false;
  d_eco_          = false;
  d_quiet_        = false;
  d_led_          = true;   // default ON
  d_sleep_stage_  = 0;

  recalc_desired_sig_();
  recalc_actual_sig_();

  // Publish initial states
  this->publish_state();
  update_led_switch_state_();

  // Pre-reserve buffers to minimize heap churn and fragmentation
  rx_.reserve(RX_BUFFER_RESERVE);
  last_status_frame_.reserve(MAX_FRAME_BYTES);
  last_tx_frame_.reserve(MAX_FRAME_BYTES);

  ESP_LOGV(TAG, "Setup completed: defaults published (mode=OFF, target=24°C, fan=AUTO, swing=OFF, desired_led=ON)");
}

void ACHIClimate::update() {
  // Periodic poll: short status query
  if (!this->writing_lock_) {
    this->send_query_status_();
  } else {
    ESP_LOGV(TAG, "Skip polling while write lock is active");
  }
}

void ACHIClimate::loop() {
  // Non-blocking accumulation of incoming bytes
  uint8_t c;
  while (this->read_byte(&c)) {
    rx_.push_back(c);
  }

  // Sliding window / buffer compaction
  if (rx_start_ > RX_COMPACT_THRESHOLD) {
    ESP_LOGV(TAG, "RX buffer compaction: removing %u bytes", (unsigned) rx_start_);
    // Erase consumed prefix in-place; capacity remains (avoids reallocation)
    rx_.erase(rx_.begin(), rx_.begin() + static_cast<std::ptrdiff_t>(rx_start_));
    rx_start_ = 0;
  }
  if (rx_.size() - rx_start_ > 4096) {
    size_t remain = rx_.size() - rx_start_;
    ESP_LOGV(TAG, "RX buffer large (%u bytes remain). Applying safety trim.", (unsigned) remain);
    if (remain >= 1 && rx_.back() == HI_HDR0) {
      uint8_t keep = rx_.back();
      rx_.clear();    // keep capacity; size becomes 0
      rx_.push_back(keep);
      rx_start_ = 0;
      ESP_LOGV(TAG, "Trimmed buffer, kept trailing header byte 0x%02X", keep);
    } else {
      rx_.clear();    // keep capacity; size becomes 0
      rx_start_ = 0;
      ESP_LOGV(TAG, "Cleared RX buffer due to overflow");
    }
  }

  // Try to extract frames within time/quantity budget
  this->try_parse_frames_from_buffer_(MAX_PARSE_TIME_MS);

  // Publish memory diagnostics at a low frequency (does not affect protocol)
  this->publish_memory_diagnostics_();
}

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

void ACHIClimate::control(const climate::ClimateCall &call) {
  bool need_write = false;

  // --- Update desired_* only (HA has top priority) ---
  if (call.get_mode().has_value()) {
    auto m = *call.get_mode();
    if (m == climate::CLIMATE_MODE_OFF) {
      d_power_on_ = false;
      d_mode_ = climate::CLIMATE_MODE_COOL; // internal default; OFF handled by d_power_on_
    } else {
      d_power_on_ = true;
      d_mode_ = m;
    }
    need_write = true;
    ESP_LOGV(TAG, "Control: requested mode=%s (power_on=%s)",
             climate::climate_mode_to_string(m), d_power_on_ ? "true" : "false");
  }

  if (call.get_target_temperature().has_value()) {
    auto t = *call.get_target_temperature();
    if (!std::isnan(t)) {
      uint8_t c = static_cast<uint8_t>(std::round(t));
      c = std::max<uint8_t>(16, std::min<uint8_t>(30, c));
      d_target_c_ = c;
      need_write = true;
      ESP_LOGV(TAG, "Control: requested target temperature=%.1f°C -> clipped=%u°C", t, (unsigned) c);
    }
  }

  if (call.get_fan_mode().has_value()) {
    d_fan_ = *call.get_fan_mode();
    need_write = true;
    ESP_LOGV(TAG, "Control: requested fan=%s", climate::climate_fan_mode_to_string(d_fan_));
  }

  if (call.get_swing_mode().has_value()) {
    d_swing_ = *call.get_swing_mode();
    need_write = true;
    ESP_LOGV(TAG, "Control: requested swing=%s", climate::climate_swing_mode_to_string(d_swing_));
  }

  if (call.get_preset().has_value()) {
    auto p = *call.get_preset();
    d_eco_ = (p == climate::CLIMATE_PRESET_ECO);
    d_turbo_ = (p == climate::CLIMATE_PRESET_BOOST);
    d_sleep_stage_ = (p == climate::CLIMATE_PRESET_SLEEP) ? 1 : 0;
    need_write = true;
    ESP_LOGV(TAG, "Control: requested preset=%s -> flags{eco=%s,turbo=%s,sleep_stage=%u}",
             climate::climate_preset_to_string(p),
             d_eco_ ? "true" : "false",
             d_turbo_ ? "true" : "false",
             (unsigned) d_sleep_stage_);
  }

  // Quiet follows fan=QUIET (desired side)
  d_quiet_ = (d_fan_ == climate::CLIMATE_FAN_QUIET);

  if (need_write) {
    // As soon as HA issues a change, block remote changes and enforce target
    accept_remote_changes_ = false;
    ha_priority_active_ = true;

    // Build TX frame from desired_* (no mapping changes)
    build_tx_from_desired_();

    this->writing_lock_ = true;
    this->pending_write_ = true;

    recalc_desired_sig_();

    ESP_LOGV(TAG, "Control: issuing WRITE (lock=true, pending=true). DESIRED: mode=%s, target=%u°C, fan=%s, swing=%s, flags{eco=%s,turbo=%s,quiet=%s,led=%s}",
             climate::climate_mode_to_string(d_mode_),
             (unsigned) d_target_c_,
             climate::climate_fan_mode_to_string(d_fan_),
             climate::climate_swing_mode_to_string(d_swing_),
             d_eco_ ? "true" : "false",
             d_turbo_ ? "true" : "false",
             d_quiet_ ? "true" : "false",
             d_led_ ? "true" : "false");

    this->send_write_changes_();
  }

  // Publish HA-visible state optimistically to desired
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
  this->publish_state();
  update_led_switch_state_();
}

// ---- Field encoding (no mapping changes) ----

uint8_t ACHIClimate::encode_mode_hi_nibble_(climate::ClimateMode m) {
  // Build high nibble: (nibble {1,3,5,7}) << 4 (TX encoding)
  return static_cast<uint8_t>(encode_nibble_from_mode(m) << 4);
}

uint8_t ACHIClimate::encode_fan_byte_(climate::ClimateFanMode f) {
  // Mapping (base codes on device side): AUTO->1, QUIET->10, LOW->12, MED->14, HIGH->16; when writing +1
  uint8_t code = 1;
  switch (f) {
    case climate::CLIMATE_FAN_AUTO:   code = 1;  break;
    case climate::CLIMATE_FAN_LOW:    code = 12; break;
    case climate::CLIMATE_FAN_MEDIUM: code = 14; break;
    case climate::CLIMATE_FAN_HIGH:   code = 16; break;
    case climate::CLIMATE_FAN_QUIET:  code = 10; break;
    default:                          code = 1;  break;
  }
  return static_cast<uint8_t>(code + 1);
}

uint8_t ACHIClimate::encode_sleep_byte_(uint8_t stage) {
  // listix_to_sleep_codes {0,1,2,4,8} -> (code<<1)|1
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

uint8_t ACHIClimate::encode_swing_ud_(bool on) {
  return on ? 0b11000000 : 0b01000000;
}
uint8_t ACHIClimate::encode_swing_lr_(bool on) {
  return on ? 0b00110000 : 0b00010000;
}

// ---- Transport/CRC ----

void ACHIClimate::send_query_status_() {
  ESP_LOGV(TAG, "TX: sending STATUS QUERY (0x66)");
  this->log_frame_("TX", this->query_);
  for (auto b : this->query_) this->write_byte(b);
  this->flush();
}

void ACHIClimate::calc_and_patch_crc_(std::vector<uint8_t> &buf) {
  // Sum of bytes from 2 to (len-4)
  const int n = static_cast<int>(buf.size());
  uint16_t csum = 0;
  for (int i = 2; i < n - 4; i++) csum = static_cast<uint16_t>(csum + buf[i]);
  buf[n - 4] = static_cast<uint8_t>((csum & 0xFF00) >> 8);
  buf[n - 3] = static_cast<uint8_t>(csum & 0x00FF);
  ESP_LOGV(TAG, "CRC patched: sum=0x%04X (bytes[%d]=0x%02X, bytes[%d]=0x%02X)",
           (unsigned) csum, n - 4, buf[n - 4], n - 3, buf[n - 3]);
}

bool ACHIClimate::validate_crc_(const std::vector<uint8_t> &buf, uint16_t *out_sum) const {
  if (buf.size() < 8) return false;
  const size_t n = buf.size();
  uint16_t csum = 0;
  for (size_t i = 2; i < n - 4; i++) csum = static_cast<uint16_t>(csum + buf[i]);
  uint8_t cr1 = static_cast<uint8_t>((csum & 0xFF00) >> 8);
  uint8_t cr2 = static_cast<uint8_t>(csum & 0x00FF);
  if (out_sum) *out_sum = csum;
  bool ok = (buf[n - 4] == cr1) && (buf[n - 3] == cr2);
  ESP_LOGV(TAG, "CRC check: sum=0x%04X -> expect[%zu]=0x%02X,%zu=0x%02X ; got=0x%02X,0x%02X -> %s",
           (unsigned) csum, n - 4, cr1, n - 3, cr2, buf[n - 4], buf[n - 3], ok ? "OK" : "MISMATCH");
  return ok;
}

void ACHIClimate::send_write_changes_() {
  // Patch CRC directly in tx_bytes_ to avoid extra temporary allocations
  this->calc_and_patch_crc_(tx_bytes_);
  ESP_LOGV(TAG, "TX: sending WRITE frame (0x65)");
  this->log_frame_("TX", tx_bytes_);
  for (auto b : tx_bytes_) this->write_byte(b);
  this->flush();

  // Keep a copy of the last exact TX frame sent for analysis (capacity pre-reserved)
  last_tx_frame_.assign(tx_bytes_.begin(), tx_bytes_.end());
}

// ---- RX scanner/parser ----

void ACHIClimate::try_parse_frames_from_buffer_(uint32_t budget_ms) {
  // Reuse a pre-reserved frame buffer to reduce heap churn
  std::vector<uint8_t> frame;
  frame.reserve(MAX_FRAME_BYTES);

  uint8_t handled = 0;
  const uint32_t start = esphome::millis();

  while (handled < MAX_FRAMES_PER_LOOP &&
         (esphome::millis() - start) < budget_ms &&
         this->extract_next_frame_(frame)) {

    // For logging and repetition detection: compute sum (not a hard CRC drop)
    uint16_t sum = 0;
    for (size_t i = 2; i + 4 <= frame.size(); i++) sum = static_cast<uint16_t>(sum + frame[i]);

    // Log RX frame dump
    this->log_frame_("RX", frame);
    uint16_t chk = 0;
    (void) this->validate_crc_(frame, &chk);  // log outcome; do not alter flow

    // Dispatch
    this->handle_frame_(frame);
    handled++;

    // Store last sum to suppress repeats if needed (current logic uses it as info)
    last_status_crc_ = sum;
  }
}

bool ACHIClimate::extract_next_frame_(std::vector<uint8_t> &frame) {
  frame.clear();

  if (rx_.size() <= rx_start_ + 5) return false;

  // 1) Find header F4 F5 starting from rx_start_
  size_t i = rx_start_;
  bool found_header = false;
  for (; i + 1 < rx_.size(); i++) {
    if (rx_[i] == HI_HDR0 && rx_[i + 1] == HI_HDR1) {
      found_header = true;
      break;
    }
  }
  if (!found_header) {
    if (!rx_.empty() && rx_.back() == HI_HDR0) {
      uint8_t keep = rx_.back();
      rx_.clear();
      rx_.push_back(keep);
      rx_start_ = 0;
      ESP_LOGV(TAG, "RX: header not found; kept trailing header byte 0x%02X", keep);
    } else {
      rx_.clear();
      rx_start_ = 0;
      ESP_LOGV(TAG, "RX: header not found; buffer cleared");
    }
    return false;
  }

  rx_start_ = i;

  // 2) Try slice by declared length: total_size = bytes[4] + 9
  if (rx_.size() > rx_start_ + 5) {
    uint8_t decl = rx_[rx_start_ + 4];
    size_t expected_total = static_cast<size_t>(decl) + 9U;

    if (rx_.size() >= rx_start_ + expected_total) {
      // Assign into pre-reserved frame buffer (no reallocation)
      frame.assign(
        rx_.begin() + static_cast<std::ptrdiff_t>(rx_start_),
        rx_.begin() + static_cast<std::ptrdiff_t>(rx_start_ + expected_total)
      );
      rx_start_ += expected_total;
      ESP_LOGV(TAG, "RX: frame sliced by declared length (decl=%u, total=%u)", (unsigned) decl, (unsigned) expected_total);
      return true;
    }
  }

  // 3) Otherwise — until first tail F4 FB
  size_t j = rx_start_ + 2;
  bool found_tail = false;
  for (; j + 1 < rx_.size(); j++) {
    if (rx_[j] == HI_TAIL0 && rx_[j + 1] == HI_TAIL1) {
      found_tail = true;
      break;
    }
  }
  if (!found_tail) return false;

  frame.assign(
    rx_.begin() + static_cast<std::ptrdiff_t>(rx_start_),
    rx_.begin() + static_cast<std::ptrdiff_t>(j + 2)
  );
  rx_start_ = j + 2;
  ESP_LOGV(TAG, "RX: frame sliced by tail (pos=%u)", (unsigned) (j + 1));
  return true;
}

void ACHIClimate::handle_frame_(const std::vector<uint8_t> &b) {
  if (b.size() < 20) {
    ESP_LOGV(TAG, "RX: frame too short (%u), ignored", (unsigned) b.size());
    return;
  }

  const uint8_t cmd = b[13];
  ESP_LOGV(TAG, "RX: dispatch cmd=0x%02X", cmd);

  if (cmd == 102 /*0x66 status resp*/ ) {
    this->parse_status_102_(b);
  } else if (cmd == 101 /*0x65 ack*/) {
    this->handle_ack_101_();
  } else {
    // Unknown frames are ignored
    ESP_LOGV(TAG, "RX: unknown cmd=0x%02X, ignored", cmd);
  }
}

void ACHIClimate::parse_status_102_(const std::vector<uint8_t> &bytes) {
  // Store the raw frame for diagnostics (pre-reserved)
  last_status_frame_.assign(bytes.begin(), bytes.end());

  // ---- Parse actual device state (do not change mappings to indices) ----

  // Power (byte 18, bit 3)
  bool power = (bytes[18] & 0b00001000) != 0;
  this->power_on_ = power;

  // Mode — upper nibble per FAN/HEAT/COOL/DRY
  uint8_t nib = static_cast<uint8_t>((bytes[18] >> 4) & 0x0F);
  this->mode_ = decode_mode_from_nibble(nib);

  // Fan speed (byte 16)
  uint8_t raw_wind = bytes[16];
  ESP_LOGV(TAG, "STATUS: raw_wind=0x%02X (%u)", raw_wind, (unsigned) raw_wind);
  climate::ClimateFanMode new_fan = climate::CLIMATE_FAN_AUTO;
  // Device reports base codes, writes require +1. Recognize base codes here.
  if (raw_wind == 0 || raw_wind == 1 || raw_wind == 2) new_fan = climate::CLIMATE_FAN_AUTO; // include 0 as fallback
  else if (raw_wind == 10) new_fan = climate::CLIMATE_FAN_QUIET;
  else if (raw_wind == 12) new_fan = climate::CLIMATE_FAN_LOW;
  else if (raw_wind == 14) new_fan = climate::CLIMATE_FAN_MEDIUM;
  else if (raw_wind == 16) new_fan = climate::CLIMATE_FAN_HIGH;
  this->fan_ = new_fan;

  // Sleep (byte 17)
  uint8_t raw_sleep = bytes[17];
  uint8_t code = (raw_sleep >> 1);
  if (code == 0) this->sleep_stage_ = 0;
  else if (code == 1) this->sleep_stage_ = 1;
  else if (code == 2) this->sleep_stage_ = 2;
  else if (code == 4) this->sleep_stage_ = 3;
  else if (code == 8) this->sleep_stage_ = 4;
  else this->sleep_stage_ = 0;

  // Target temperature (byte 19) — value in °C directly on RX
  uint8_t raw_set = bytes[19];
  if (raw_set >= 16 && raw_set <= 30) this->target_c_ = raw_set;
  this->target_temperature = this->target_c_;

  // Current air temperature (byte 20)
  uint8_t tair = bytes[20];
  this->current_temperature = tair;

  // Pipe temperature (byte 21)
#ifdef USE_SENSOR
  if (this->pipe_sensor_ != nullptr) this->pipe_sensor_->publish_state(bytes[21]);
#endif

  // Turbo/Eco/Quiet/LED
  uint8_t b35 = bytes[35];
  this->turbo_ = (b35 & 0b00000010) != 0;       // turbo_mask = 0b00000010
  this->eco_   = (b35 & 0b00000100) != 0;       // eco_mask   = 0b00000100
  this->quiet_ = (bytes[36] & 0b00000100) != 0; // quiet      in 36th
  this->led_   = (bytes[37] & 0b10000000) != 0; // LED        in 37th

  // Swing (byte 35: updown bit7, leftright bit6)
  bool updown = (bytes[35] & 0b10000000) != 0;
  bool leftright = (bytes[35] & 0b01000000) != 0;
  if (updown && leftright) this->swing_ = climate::CLIMATE_SWING_BOTH;
  else if (updown) this->swing_ = climate::CLIMATE_SWING_VERTICAL;
  else if (leftright) this->swing_ = climate::CLIMATE_SWING_HORIZONTAL;
  else this->swing_ = climate::CLIMATE_SWING_OFF;

  // Recalculate actual control signature after parsing
  recalc_actual_sig_();

  // Publish HA-visible state with gating and keep LED switch in sync
  publish_gated_state_();
  update_led_switch_state_();

  // ---- Publish optional sensors (only if configured in YAML) ----
#ifdef USE_SENSOR
  if (set_temp_sensor_ != nullptr) set_temp_sensor_->publish_state(bytes[19]);
  if (room_temp_sensor_ != nullptr) room_temp_sensor_->publish_state(bytes[20]);
  if (wind_code_sensor_ != nullptr) wind_code_sensor_->publish_state(bytes[16]);
  if (sleep_code_sensor_ != nullptr) sleep_code_sensor_->publish_state(bytes[17]);
  if (mode_code_sensor_ != nullptr) mode_code_sensor_->publish_state((bytes[18] >> 4) & 0x0F);
  if (quiet_code_sensor_ != nullptr) quiet_code_sensor_->publish_state((bytes[36] & 0b00000100) ? 1 : 0);
  if (turbo_code_sensor_ != nullptr) turbo_code_sensor_->publish_state((bytes[35] & 0b00000010) ? 1 : 0);
  if (eco_code_sensor_ != nullptr)   eco_code_sensor_->publish_state((bytes[35] & 0b00000100) ? 1 : 0);
  if (swing_ud_sensor_ != nullptr)   swing_ud_sensor_->publish_state((bytes[35] & 0b10000000) ? 1 : 0);
  if (swing_lr_sensor_ != nullptr)   swing_lr_sensor_->publish_state((bytes[35] & 0b01000000) ? 1 : 0);
  if (compressor_freq_set_sensor_ != nullptr) compressor_freq_set_sensor_->publish_state(bytes[42]);
  if (compressor_freq_sensor_ != nullptr)     compressor_freq_sensor_->publish_state(bytes[43]);
  if (outdoor_temp_sensor_ != nullptr)        outdoor_temp_sensor_->publish_state(bytes[44]);
  if (outdoor_cond_temp_sensor_ != nullptr)   outdoor_cond_temp_sensor_->publish_state(bytes[45]);
#endif

#ifdef USE_TEXT_SENSOR
  if (power_status_text_ != nullptr) power_status_text_->publish_state(this->power_on_ ? "ON" : "OFF");
#endif

  ESP_LOGV(TAG,
           "Parsed STATUS: power=%s, mode=%s, fan=%s, swing=%s, target=%u°C, current=%u°C, sleep_stage=%u, flags{eco=%s,turbo=%s,quiet=%s,led=%s}",
           this->power_on_ ? "ON" : "OFF",
           climate::climate_mode_to_string(this->mode_),
           climate::climate_fan_mode_to_string(this->fan_),
           climate::climate_swing_mode_to_string(this->swing_),
           (unsigned) this->target_c_,
           (unsigned) this->current_temperature,
           (unsigned) this->sleep_stage_,
           this->eco_ ? "true" : "false",
           this->turbo_ ? "true" : "false",
           this->quiet_ ? "true" : "false",
           this->led_ ? "true" : "false");

  // If HA has priority, keep enforcing desired until signatures match
  maybe_force_to_target_();
}

void ACHIClimate::handle_ack_101_() {
  this->writing_lock_ = false;
  this->pending_write_ = false;
  ESP_LOGV(TAG, "ACK(0x65): write acknowledged (lock=false, pending=false)");
}

// ---- Priority/authority helpers ----

void ACHIClimate::build_tx_from_desired_() {
  // Build TX frame from desired_* fields; keep protocol mapping identical
  uint8_t power_bin = d_power_on_ ? 0b00001100 : 0b00000100;  // low nibble
  uint8_t mode_hi   = static_cast<uint8_t>(encode_nibble_from_mode(d_mode_) << 4);
  tx_bytes_[18] = static_cast<uint8_t>(power_bin + mode_hi);

  // setpoint (protocol requires encoded form on TX)
  tx_bytes_[19] = encode_temp_(d_target_c_);

  tx_bytes_[16] = encode_fan_byte_(d_fan_);             // fan
  tx_bytes_[17] = encode_sleep_byte_(d_sleep_stage_);   // sleep

  const bool v_swing = (d_swing_ == climate::CLIMATE_SWING_VERTICAL) || (d_swing_ == climate::CLIMATE_SWING_BOTH);
  const bool h_swing = (d_swing_ == climate::CLIMATE_SWING_HORIZONTAL) || (d_swing_ == climate::CLIMATE_SWING_BOTH);
  const uint8_t updown_bin = encode_swing_ud_(v_swing);
  const uint8_t leftright_bin = encode_swing_lr_(h_swing);
  tx_bytes_[32] = static_cast<uint8_t>(updown_bin + leftright_bin);

  const uint8_t turbo_bin = d_turbo_ ? 0b00001100 : 0b00000100;
  const uint8_t eco_bin = d_eco_ ? 0b00110000 : 0b00000000;
  tx_bytes_[33] = (d_turbo_ ? turbo_bin : (eco_bin ? eco_bin : 0));

  d_quiet_ = (d_fan_ == climate::CLIMATE_FAN_QUIET);
  tx_bytes_[35] = d_quiet_ ? 0b00110000 : 0b00000000;

  tx_bytes_[36] = d_led_ ? 0b11000000 : 0b01000000;

  // Turbo overrides certain fields per protocol
  if (d_turbo_) {
    tx_bytes_[19] = d_target_c_; // turbo mode: do not alter setpoint encoding here
    tx_bytes_[33] = turbo_bin;
    tx_bytes_[35] = 0;           // override quiet
  }
}

void ACHIClimate::publish_gated_state_() {
  // Always publish current temperatures from status
  // Gate only control-related fields depending on accept_remote_changes_
  if (accept_remote_changes_) {
    // Publish ACTUAL (remote side allowed)
    this->mode = this->power_on_ ? this->mode_ : climate::CLIMATE_MODE_OFF;
    this->target_temperature = this->target_c_;
    this->fan_mode = this->fan_;
    this->swing_mode = this->swing_;
    if (enable_presets_) {
      if (this->turbo_) this->preset = climate::CLIMATE_PRESET_BOOST;
      else if (this->eco_) this->preset = climate::CLIMATE_PRESET_ECO;
      else if (this->sleep_stage_ > 0) this->preset = climate::CLIMATE_PRESET_SLEEP;
      else this->preset = climate::CLIMATE_PRESET_NONE;
    }
    // When remote changes are accepted, mirror desired_* to actual_*
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
    // Publish DESIRED (while forcing target)
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
  this->publish_state();
}

void ACHIClimate::update_led_switch_state_() {
  if (led_switch_ == nullptr) return;
  // Show desired if enforcing, otherwise desired==actual (synced above)
  bool to_publish = d_led_;
  led_switch_->publish_state(to_publish);
}

uint32_t ACHIClimate::compute_control_signature_(bool power, climate::ClimateMode mode,
                                                 climate::ClimateFanMode fan, climate::ClimateSwingMode swing,
                                                 bool eco, bool turbo, bool quiet, bool led,
                                                 uint8_t sleep_stage, uint8_t target_c) const {
  // FNV-1a over normalized control fields (include LED; exclude sensor bytes)
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

void ACHIClimate::maybe_force_to_target_() {
  if (!ha_priority_active_) return;

  if (actual_sig_ == desired_sig_) {
    // Converged to the desired state, unlock remote acceptance
    ha_priority_active_ = false;
    accept_remote_changes_ = true;
    ESP_LOGI(TAG, "Converged to desired HA state; remote changes are accepted again.");
    return;
  }

  // Not yet converged — print diff and send another write if possible
  log_sig_diff_();

  if (!writing_lock_) {
    ESP_LOGD(TAG, "Enforcing desired HA state (sig_actual=0x%08X != sig_desired=0x%08X) -> WRITE",
             (unsigned) actual_sig_, (unsigned) desired_sig_);
    build_tx_from_desired_();
    writing_lock_ = true;
    pending_write_ = true;
    send_write_changes_();
  } else {
    ESP_LOGV(TAG, "Write lock active while enforcing target, will retry after ACK/status");
  }
}

void ACHIClimate::log_sig_diff_() const {
  // Debug helper: log which control fields differ (actual vs desired)
  auto b2s = [](bool v){ return v ? "true" : "false"; };
  if (power_on_ != d_power_on_) ESP_LOGV(TAG, "DIFF power_on: actual=%s desired=%s", b2s(power_on_), b2s(d_power_on_));
  if (mode_ != d_mode_) ESP_LOGV(TAG, "DIFF mode: actual=%s desired=%s", climate::climate_mode_to_string(mode_), climate::climate_mode_to_string(d_mode_));
  if (fan_ != d_fan_) ESP_LOGV(TAG, "DIFF fan: actual=%s desired=%s", climate::climate_fan_mode_to_string(fan_), climate::climate_fan_mode_to_string(d_fan_));
  if (swing_ != d_swing_) ESP_LOGV(TAG, "DIFF swing: actual=%s desired=%s", climate::climate_swing_mode_to_string(swing_), climate::climate_swing_mode_to_string(d_swing_));
  if (eco_ != d_eco_) ESP_LOGV(TAG, "DIFF eco: actual=%s desired=%s", b2s(eco_), b2s(d_eco_));
  if (turbo_ != d_turbo_) ESP_LOGV(TAG, "DIFF turbo: actual=%s desired=%s", b2s(turbo_), b2s(d_turbo_));
  if (quiet_ != d_quiet_) ESP_LOGV(TAG, "DIFF quiet: actual=%s desired=%s", b2s(quiet_), b2s(d_quiet_));
  if (led_ != d_led_) ESP_LOGV(TAG, "DIFF led: actual=%s desired=%s", b2s(led_), b2s(d_led_));
  if (sleep_stage_ != d_sleep_stage_) ESP_LOGV(TAG, "DIFF sleep_stage: actual=%u desired=%u", (unsigned) sleep_stage_, (unsigned) d_sleep_stage_);
  if (target_c_ != d_target_c_) ESP_LOGV(TAG, "DIFF target_c: actual=%u desired=%u", (unsigned) target_c_, (unsigned) d_target_c_);
}

// ---- External control from LED switch ----
void ACHIClimate::set_desired_led(bool on) {
  d_led_ = on;

  // HA takes priority now; enforce target LED along with other desired fields
  accept_remote_changes_ = false;
  ha_priority_active_ = true;

  build_tx_from_desired_();
  writing_lock_ = true;
  pending_write_ = true;

  recalc_desired_sig_();

  ESP_LOGV(TAG, "LED switch: desired_led=%s -> WRITE", on ? "ON" : "OFF");
  send_write_changes_();
}

// ---- Logging helpers ----

void ACHIClimate::log_frame_(const char *prefix, const std::vector<uint8_t> &b) const {
  // Hex dump with stack buffers to avoid heap fragmentation.
  // We print header line and then data lines with up to LOG_BYTES_PER_LINE bytes each.
  char header[64];
  int hn = snprintf(header, sizeof(header), "%s FRAME (%u bytes)", prefix, (unsigned) b.size());
  if (hn > 0) {
    ESP_LOGV(TAG, "%s", header);
  }

  const size_t n = b.size();
  size_t i = 0;
  while (i < n) {
    char line[8 + (3 * LOG_BYTES_PER_LINE) + 8]; // "0000: " + 3*bytes + safety
    int pos = snprintf(line, sizeof(line), "%04u: ", (unsigned) i);
    size_t chunk = std::min(LOG_BYTES_PER_LINE, n - i);
    for (size_t j = 0; j < chunk && pos < (int)sizeof(line) - 4; j++) {
      pos += snprintf(line + pos, sizeof(line) - pos, "%02X ", b[i + j]);
    }
    ESP_LOGV(TAG, "%s", line);
    i += chunk;
  }
}

// ---- Memory diagnostics (optional sensors) ----
void ACHIClimate::publish_memory_diagnostics_() {
#ifdef USE_SENSOR
  static uint32_t last_ms = 0;
  const uint32_t now = esphome::millis();
  if (now - last_ms < MEM_PUBLISH_INTERVAL_MS) return;
  last_ms = now;

  // Gather metrics with best-effort portability (no impact on AC protocol)
  size_t heap_free = 0, heap_total = 0, heap_used = 0, heap_min_free = 0, heap_max_alloc = 0;
  int heap_frag_pct = -1;
  size_t psram_total = 0, psram_free = 0;

#if defined(ARDUINO_ARCH_ESP32)
  heap_total     = ESP.getHeapSize();
  heap_free      = ESP.getFreeHeap();
  heap_min_free  = ESP.getMinFreeHeap();
  heap_max_alloc = ESP.getMaxAllocHeap();
  psram_total    = ESP.getPsramSize();
  psram_free     = ESP.getFreePsram();
  // Approximate fragmentation as (1 - largest_block / free) * 100
  if (heap_free > 0 && heap_max_alloc > 0) {
    double ratio = 1.0 - static_cast<double>(heap_max_alloc) / static_cast<double>(heap_free);
    if (ratio < 0.0) ratio = 0.0;
    if (ratio > 1.0) ratio = 1.0;
    heap_frag_pct = static_cast<int>(std::lround(ratio * 100.0));
  } else {
    heap_frag_pct = 0;
  }
#elif defined(ARDUINO_ARCH_ESP8266)
  heap_free      = ESP.getFreeHeap();
  heap_max_alloc = ESP.getMaxFreeBlockSize();
  heap_frag_pct  = ESP.getHeapFragmentation(); // 0..100
  // No reliable total heap API on ESP8266; leave heap_total/used unknown.
#endif

  if (heap_total > 0 && heap_total >= heap_free) heap_used = heap_total - heap_free;

  // Publish to sensors if configured; skip metrics that are not available on this arch.
  if (heap_free_sensor_ != nullptr) heap_free_sensor_->publish_state(static_cast<float>(heap_free));
  if (heap_total_sensor_ != nullptr && heap_total > 0) heap_total_sensor_->publish_state(static_cast<float>(heap_total));
  if (heap_used_sensor_  != nullptr && heap_total > 0) heap_used_sensor_->publish_state(static_cast<float>(heap_used));
  if (heap_min_free_sensor_ != nullptr && heap_min_free > 0) heap_min_free_sensor_->publish_state(static_cast<float>(heap_min_free));
  if (heap_max_alloc_sensor_ != nullptr && heap_max_alloc > 0) heap_max_alloc_sensor_->publish_state(static_cast<float>(heap_max_alloc));
  if (heap_fragmentation_sensor_ != nullptr && heap_frag_pct >= 0) heap_fragmentation_sensor_->publish_state(static_cast<float>(heap_frag_pct));
  if (psram_total_sensor_ != nullptr && psram_total > 0) psram_total_sensor_->publish_state(static_cast<float>(psram_total));
  if (psram_free_sensor_  != nullptr && psram_total > 0) psram_free_sensor_->publish_state(static_cast<float>(psram_free));

#else
  (void)0; // sensors not compiled-in; do nothing
#endif
}

} // namespace ac_hi
} // namespace esphome
