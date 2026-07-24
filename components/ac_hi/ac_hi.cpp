#include "ac_hi.h"
#include <cmath>
#include <cstdio>
#include <string>
#include <Arduino.h>
#include "esphome/core/log.h"
#ifdef USE_MQTT
#include "esphome/components/mqtt/mqtt_client.h"
#endif

namespace esphome {
namespace ac_hi {

static const char *const TAG = "ac_hi";

static void log_kelon168_data(const char *prefix, const Kelon168Data &data) {
  char buffer[KELON168_STATE_LENGTH * 3 + 1];
  size_t pos = 0;
  for (uint8_t i = 0; i < KELON168_STATE_LENGTH; i++) {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "%02X%s", data.state[i],
                    i + 1 == KELON168_STATE_LENGTH ? "" : " ");
  }
  ESP_LOGI(TAG, "%s Kelon168 IR: %s (command=0x%02X)", prefix, buffer, data.command());
}

static const char *const CUSTOM_PRESET_QUIET = "Тихий";
static const char *const CUSTOM_FAN_TURBO = "Турбо";

// Restore the last target temperature after Turbo is turned off from HA.
static uint8_t g_pre_turbo_target_c = 24;
static bool g_has_pre_turbo_target = false;

// Restore previous ECO state after ECO is turned off from HA.
static uint8_t g_pre_eco_target_c = 24;
static climate::ClimateFanMode g_pre_eco_fan = climate::CLIMATE_FAN_AUTO;
static bool g_has_pre_eco_state = false;

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

// ---- ACHICommandSoundSwitch ----
void ACHICommandSoundSwitch::write_state(bool state) {
  if (parent_ != nullptr) {
    // Parent may reject OFF while the display is OFF, so let it publish
    // the effective state instead of echoing the requested state here.
    parent_->set_command_sound_enabled(state);
  } else {
    publish_state(state);
  }
}

// ---- ACHIMemorySwitch ----
void ACHIMemorySwitch::write_state(bool state) {
  if (parent_ != nullptr) {
    parent_->set_memory_mode_enabled(state);
  } else {
    publish_state(state);
  }
}

// ---- ACHIClimate implementation ----

void ACHIClimate::setup() {
  // Register custom presets on the Climate entity, not on ClimateTraits.
  // ClimateTraits::set_supported_custom_presets() is deprecated and will be removed in ESPHome 2026.11.0.
  if (enable_presets_) {
    this->set_supported_custom_presets({CUSTOM_PRESET_QUIET});
  }
  // Fan Turbo is exposed as a custom fan mode, not as a preset.
  // It only sends raw Wind Mode Code 18 without changing target temperature.
  this->set_supported_custom_fan_modes({CUSTOM_FAN_TURBO});

  // Initial HA‑visible state
  mode = climate::CLIMATE_MODE_OFF;
  target_temperature = 24;
  fan_mode = climate::CLIMATE_FAN_AUTO;
  swing_mode = climate::CLIMATE_SWING_OFF;
  // Desired state mirrors initial
  d_power_on_     = false;
  d_mode_         = climate::CLIMATE_MODE_OFF;
  last_active_mode_ = climate::CLIMATE_MODE_COOL;
  d_target_c_     = 24;
  d_fan_          = climate::CLIMATE_FAN_AUTO;
  d_fan_turbo_    = false;
  d_swing_        = climate::CLIMATE_SWING_OFF;
  d_turbo_        = false;
  d_eco_          = false;
  d_quiet_        = false;
  d_led_          = true;
  d_sleep_stage_  = 0;
  
  g_pre_turbo_target_c = d_target_c_;
  g_has_pre_turbo_target = false;

  g_pre_eco_target_c = d_target_c_;
  g_pre_eco_fan = d_fan_;
  g_has_pre_eco_state = false;

  recalc_desired_sig_();
  recalc_actual_sig_();

  publish_state();
  update_led_switch_state_();
  update_sound_switch_state_();
  update_memory_switch_state_();

  // Remember boot time so the first status poll is delayed after a full power restore.
  // Indoor AC boards can be noisy on UART while they are still booting; polling too early
  // may flood the component and starve ESPHome API/Wi-Fi.
  boot_ms_ = millis();

  // Pre‑reserve buffers to avoid reallocations
  rx_.reserve(RX_BUFFER_RESERVE);
  last_status_frame_.reserve(MAX_FRAME_BYTES);
  last_tx_frame_.reserve(MAX_FRAME_BYTES);

  ESP_LOGI(TAG, "Setup complete; first AC status query will be delayed by %u ms",
           (unsigned) STARTUP_POLL_DELAY_MS);
}

void ACHIClimate::update() {
  // Give the indoor AC controller time to boot after a complete power loss.
  // Without this guard, the ESP can query the UART while the AC board is still
  // starting and producing incomplete/noisy frames.
  if (millis() - boot_ms_ < STARTUP_POLL_DELAY_MS) {
    ESP_LOGD(TAG, "Startup delay: skipping status query");
    return;
  }

  if (!writing_lock_ && !pending_control_) {
    send_query_status_();
  } else {
    ESP_LOGV(TAG, "Polling skipped (write lock active or pending control)");
  }
}

void ACHIClimate::loop() {
  // 1. Accumulate incoming bytes without provoking UART timeout logs when no data is available.
  // Limit the number of bytes read in one loop iteration. After a full AC power restore
  // the indoor controller may emit a burst of bytes/noise; reading it all at once can
  // starve ESPHome API/Wi-Fi and cause Home Assistant disconnects.
  uint8_t c;
  uint16_t read_count = 0;
  while (available() > 0 && read_count < MAX_UART_BYTES_PER_LOOP) {
    if (!read_byte(&c)) {
      break;
    }
    rx_.push_back(c);
    read_count++;
  }
  if (read_count >= MAX_UART_BYTES_PER_LOOP && available() > 0) {
    ESP_LOGV(TAG, "UART backlog remains after %u bytes, yielding to ESPHome",
             (unsigned) read_count);
    yield();
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
  t.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE);
  return t;
}

// ---- Control from HA ----
void ACHIClimate::control(const climate::ClimateCall &call) {
  bool changed = false;
  bool was_power_on = d_power_on_;

  if (call.get_mode().has_value()) {
    auto m = *call.get_mode();
    if (m == climate::CLIMATE_MODE_OFF) {
      if (memory_mode_enabled_) {
        // Remember the last real working mode before OFF. Do not let OFF erase it.
        if (d_power_on_ && d_mode_ != climate::CLIMATE_MODE_OFF) {
          last_active_mode_ = d_mode_;
        } else if (power_on_ && mode_ != climate::CLIMATE_MODE_OFF) {
          last_active_mode_ = mode_;
        }
      }

      d_power_on_ = false;
      if (memory_mode_enabled_) {
        // Keep the last real mode in the desired state while power is OFF.
        // The published HA mode remains OFF, but the next generic climate.turn_on
        // can restore the real previous mode instead of Home Assistant's fallback.
        d_mode_ = last_active_mode_;
      } else {
        // Original behavior: OFF falls back to COOL and the next turn_on accepts
        // whatever mode Home Assistant sends.
        d_mode_ = climate::CLIMATE_MODE_COOL;
      }
    } else {
      d_power_on_ = true;

      auto requested_mode = m;
      if (memory_mode_enabled_ && !was_power_on && last_active_mode_ != climate::CLIMATE_MODE_OFF &&
          requested_mode != last_active_mode_) {
        ESP_LOGD(TAG, "Power-on mode %s replaced by last active mode %s because Memory is ON",
                 climate::climate_mode_to_string(requested_mode),
                 climate::climate_mode_to_string(last_active_mode_));
        requested_mode = last_active_mode_;
      }

      d_mode_ = requested_mode;
      if (memory_mode_enabled_) {
        last_active_mode_ = d_mode_;
      }
      if (!was_power_on) {
        // Match remote behavior: powering on restores the front display LED,
        // but send the display command only if we are actually changing it.
        if (!d_led_) {
          d_led_ = true;
          led_command_pending_ = true;
        }
      }

      // Do not carry an implicit QUIET fan value from DRY/FAN_ONLY status into
      // a user mode change. On this indoor unit raw wind code 10 may be reported
      // by the unit while Quiet Mode Code is still 0; that is not an explicit
      // Quiet request from HA and should fall back to AUTO when the mode changes.
      if (!call.get_fan_mode().has_value() && d_fan_ == climate::CLIMATE_FAN_QUIET &&
          !d_quiet_ && !d_eco_ && !d_turbo_ && d_sleep_stage_ == 0) {
        d_fan_ = climate::CLIMATE_FAN_AUTO;
      }
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
    d_fan_turbo_ = false;
    d_quiet_ = (d_fan_ == climate::CLIMATE_FAN_QUIET);
    changed = true;
  }

  if (call.get_swing_mode().has_value()) {
    d_swing_ = *call.get_swing_mode();
    changed = true;
  }

  if (call.get_preset().has_value()) {
    auto p = *call.get_preset();
    bool was_turbo = d_turbo_;
    bool was_eco = d_eco_;

    d_eco_ = false;
    d_turbo_ = false;
    d_sleep_stage_ = 0;

    // Leaving Turbo should restore AUTO fan unless a new preset overrides it.
    if (was_turbo && p != climate::CLIMATE_PRESET_BOOST) {
      d_fan_ = climate::CLIMATE_FAN_AUTO;
    }

    if (p == climate::CLIMATE_PRESET_ECO) {
      if (!was_eco) {
        g_pre_eco_target_c = d_target_c_;
        g_pre_eco_fan = d_fan_;
        g_has_pre_eco_state = true;
      }

      d_eco_ = true;
      d_turbo_ = false;
      d_sleep_stage_ = 0;
      d_fan_turbo_ = false;

      // Match the behavior observed when ECO is enabled from the remote:
      // target temperature becomes 24°C and fan becomes QUIET.
      d_target_c_ = 24;
      d_fan_ = climate::CLIMATE_FAN_QUIET;
      d_quiet_ = false;
    } else if (p == climate::CLIMATE_PRESET_BOOST) {
      if (!was_turbo) {
        g_pre_turbo_target_c = d_target_c_;
        g_has_pre_turbo_target = true;
      }

      d_turbo_ = true;
      d_fan_turbo_ = false;
      d_quiet_ = false;
      if (d_mode_ == climate::CLIMATE_MODE_HEAT) {
        d_target_c_ = 30;
        d_fan_ = climate::CLIMATE_FAN_AUTO;
      } else {
        d_target_c_ = 16;
        d_fan_ = climate::CLIMATE_FAN_HIGH;
      }
    } else if (p == climate::CLIMATE_PRESET_SLEEP) {
      // Match the behavior observed from the remote:
      // SLEEP uses QUIET fan, target temperature increases by 1°C,
      // and the status frame reports sleep code 2 / 4 rather than 1.
      if (d_target_c_ < 30) {
        d_target_c_ += 1;
      }
      d_sleep_stage_ = 2;
      d_fan_turbo_ = false;
      d_quiet_ = false;
      d_fan_ = climate::CLIMATE_FAN_QUIET;
    } else if (p == climate::CLIMATE_PRESET_NONE) {
      if (was_turbo && g_has_pre_turbo_target) {
        d_target_c_ = g_pre_turbo_target_c;
        g_has_pre_turbo_target = false;
      }

      if (was_eco && g_has_pre_eco_state) {
        d_target_c_ = g_pre_eco_target_c;
        d_fan_ = g_pre_eco_fan;
        g_has_pre_eco_state = false;
      }

      d_quiet_ = false;
      if (d_fan_ == climate::CLIMATE_FAN_QUIET && !was_eco)
        d_fan_ = climate::CLIMATE_FAN_AUTO;
    }

    changed = true;
  }

  auto custom = call.get_custom_preset();
  if (!custom.empty()) {
    if (custom == CUSTOM_PRESET_QUIET) {
      d_quiet_ = true;
      d_turbo_ = false;
      d_eco_ = false;
      d_sleep_stage_ = 0;
      d_fan_turbo_ = false;
      d_fan_ = climate::CLIMATE_FAN_QUIET;
      changed = true;
    }
  }

  auto custom_fan = call.get_custom_fan_mode();
  if (!custom_fan.empty()) {
    if (custom_fan == CUSTOM_FAN_TURBO) {
      // Fan Turbo is independent from the BOOST preset: keep the current
      // temperature and mode, but send raw Wind Mode Code 18.
      d_fan_turbo_ = true;
      d_fan_ = climate::CLIMATE_FAN_HIGH;
      d_quiet_ = false;
      d_turbo_ = false;
      d_eco_ = false;
      d_sleep_stage_ = 0;
      changed = true;
    }
  }

  if (!changed) return;

  // HA takes priority over remote changes
  accept_remote_changes_ = false;
  ha_priority_active_ = true;

  recalc_desired_sig_();

  // Publish optimistically
  this->mode = d_power_on_ ? d_mode_ : climate::CLIMATE_MODE_OFF;
  this->target_temperature = d_target_c_;
  publish_fan_state_(d_fan_turbo_, d_fan_);
  this->swing_mode = d_swing_;
  if (enable_presets_) {
    if (d_turbo_) this->set_preset_(climate::CLIMATE_PRESET_BOOST);
    else if (d_eco_) this->set_preset_(climate::CLIMATE_PRESET_ECO);
    else if (d_sleep_stage_ > 0) this->set_preset_(climate::CLIMATE_PRESET_SLEEP);
    else if (d_quiet_) this->set_custom_preset_(CUSTOM_PRESET_QUIET);
    else this->set_preset_(climate::CLIMATE_PRESET_NONE);
  }
  publish_state();
  update_led_switch_state_();

  // Mark that we have a pending command (debounced)
  pending_control_ = true;
  last_control_ms_ = millis();
  user_command_next_write_ = true;
  beep_on_next_write_ = command_sound_enabled_;

  ESP_LOGD(TAG, "Control: new desired state registered, command_sound=%s, will send after %ums debounce",
           command_sound_enabled_ ? "ON" : "OFF", CONTROL_DEBOUNCE_MS);
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
  tx_bytes_[IDX_WIND] = encode_fan_byte_(d_fan_, d_fan_turbo_);

  // Sleep mode (byte 17)
  tx_bytes_[IDX_SLEEP] = encode_sleep_byte_(d_sleep_stage_);

  // Swing (byte 32)
  bool v = (d_swing_ == climate::CLIMATE_SWING_VERTICAL) || (d_swing_ == climate::CLIMATE_SWING_BOTH);
  bool h = (d_swing_ == climate::CLIMATE_SWING_HORIZONTAL) || (d_swing_ == climate::CLIMATE_SWING_BOTH);
  uint8_t updown = v ? TxValues::UPDOWN_ON : TxValues::UPDOWN_OFF;
  uint8_t leftright = h ? TxValues::LEFTRIGHT_ON : TxValues::LEFTRIGHT_OFF;
  tx_bytes_[IDX_TX_SWING] = updown + leftright;

  // Turbo, Eco, Quiet (bytes 33 and 35) – Turbo has priority
  if (d_turbo_) {
    tx_bytes_[IDX_TX_TURBO_ECO] = TxValues::TURBO_ON;
    tx_bytes_[IDX_TX_QUIET] = TxValues::QUIET_OFF;   // Turbo disables quiet
  } else {
    tx_bytes_[IDX_TX_TURBO_ECO] = TxValues::TURBO_OFF + (d_eco_ ? TxValues::ECO_ON : TxValues::ECO_OFF);
    tx_bytes_[IDX_TX_QUIET] = d_quiet_ ? TxValues::QUIET_ON : TxValues::QUIET_OFF;
  }

  // Display/LED command (byte 36).
  // 0xC0/0x40 are explicit display ON/OFF actions.
  // Keep automatic HA-priority re-sends neutral (0x00) so they stay silent.
  // But when the user sends a real climate command while the display is OFF,
  // include LED_OFF once in that command; otherwise this indoor unit turns the
  // front display back on after mode/preset changes. This must be independent
  // from audible confirmation, because sound can be disabled by sound_switch.
  if (led_command_pending_) {
    tx_bytes_[IDX_TX_LED] = d_led_ ? TxValues::LED_ON : TxValues::LED_OFF;
  } else if (user_command_next_write_ && !d_led_) {
    tx_bytes_[IDX_TX_LED] = TxValues::LED_OFF;
  } else {
    tx_bytes_[IDX_TX_LED] = 0x00;
  }
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
  tx_bytes_[IDX_TX_BEEP] = beep_on_next_write_ ? TxValues::BEEP_ON : TxValues::BEEP_OFF;
  calc_and_patch_crc_(tx_bytes_);
  ESP_LOGD(TAG, "Sending write command (0x65): beep_byte[23]=0x%02X led_byte[36]=0x%02X",
           tx_bytes_[IDX_TX_BEEP], tx_bytes_[IDX_TX_LED]);
  log_frame_("TX write", tx_bytes_);
  for (auto b : tx_bytes_) write_byte(b);
  flush();

  last_tx_frame_.assign(tx_bytes_.begin(), tx_bytes_.end());
  beep_on_next_write_ = false;
  user_command_next_write_ = false;
  led_command_pending_ = false;

  writing_lock_ = true;
  write_lock_time_ = millis();
}

// ---- IR iFeel / Follow Me over Kelon168 ----
uint8_t ACHIClimate::encode_kelon_mode_(climate::ClimateMode mode) const {
  switch (mode) {
    case climate::CLIMATE_MODE_HEAT:
      return KELON168_MODE_HEAT;
    case climate::CLIMATE_MODE_COOL:
      return KELON168_MODE_COOL;
    case climate::CLIMATE_MODE_DRY:
      return KELON168_MODE_DRY;
    case climate::CLIMATE_MODE_FAN_ONLY:
      return KELON168_MODE_FAN;
    default:
      return KELON168_MODE_AUTO;
  }
}

void ACHIClimate::set_kelon_fan_(Kelon168Data *data, climate::ClimateFanMode fan_mode, bool turbo_fan) const {
  // Fan mode in Kelon168 IR frames is split between state[2] bits 0..1
  // and state[17] bit 0x40. The mapping below is based on captures from
  // this Hisense indoor unit while sending vertical/horizontal swing commands,
  // i.e. frames where fan mode is preserved rather than cycled by the Fan key:
  //   AUTO   -> state[2] = 0, state[17]&0x40 = 0
  //   TURBO  -> state[2] = 1, state[17]&0x40 = 0
  //   HIGH   -> state[2] = 1, state[17]&0x40 = 1
  //   MEDIUM -> state[2] = 2, state[17]&0x40 = 0
  //   LOW    -> state[2] = 3, state[17]&0x40 = 1
  //   QUIET  -> state[2] = 3, state[17]&0x40 = 0
  data->state[2] &= ~0x03;
  data->state[17] &= ~0x40;

  if (turbo_fan) {
    data->state[2] |= 0x01;
    return;
  }

  switch (fan_mode) {
    case climate::CLIMATE_FAN_LOW:
      data->state[2] |= 0x03;
      data->state[17] |= 0x40;
      break;
    case climate::CLIMATE_FAN_MEDIUM:
      data->state[2] |= 0x02;
      break;
    case climate::CLIMATE_FAN_HIGH:
      data->state[2] |= 0x01;
      data->state[17] |= 0x40;
      break;
    case climate::CLIMATE_FAN_QUIET:
      data->state[2] |= 0x03;
      break;
    case climate::CLIMATE_FAN_AUTO:
    default:
      break;
  }
}

Kelon168Data ACHIClimate::build_kelon_state_from_current_(uint8_t command) const {
  auto data = Kelon168Protocol::make_default();

  // Prefer the real state parsed from UART. Before the first status frame arrives,
  // fall back to the HA desired state so iFeel still has a sane base frame.
  const bool have_actual = !this->last_status_frame_.empty();
  const bool base_power = have_actual ? this->power_on_ : this->d_power_on_;
  auto base_mode = have_actual ? this->mode_ : this->d_mode_;
  auto base_fan = have_actual ? this->fan_ : this->d_fan_;
  const bool base_fan_turbo = have_actual ? this->fan_turbo_ : this->d_fan_turbo_;
  auto base_swing = have_actual ? this->swing_ : this->d_swing_;
  const bool base_boost = have_actual ? this->turbo_ : this->d_turbo_;
  const uint8_t base_sleep_stage = have_actual ? this->sleep_stage_ : this->d_sleep_stage_;
  uint8_t base_target = have_actual ? this->target_c_ : this->d_target_c_;

  if (!base_power && base_mode == climate::CLIMATE_MODE_OFF) {
    // The unit ignores climate fields in iFeel frames, but OFF has no native Kelon168
    // mode. Keep a neutral COOL/24 base if someone calls the action too early.
    base_mode = climate::CLIMATE_MODE_COOL;
    base_fan = climate::CLIMATE_FAN_AUTO;
    base_swing = climate::CLIMATE_SWING_OFF;
    base_target = 24;
  }

  base_target = std::max<uint8_t>(16, std::min<uint8_t>(30, base_target));
  const uint8_t native_mode = this->encode_kelon_mode_(base_mode);

  data.state[2] = 0x00;
  if (base_sleep_stage > 0)
    data.state[2] |= 0x08;
  if (base_swing == climate::CLIMATE_SWING_VERTICAL || base_swing == climate::CLIMATE_SWING_BOTH)
    data.state[2] |= 0x80;

  this->set_kelon_fan_(&data, base_fan, base_fan_turbo);
  data.state[3] = static_cast<uint8_t>((native_mode & 0x07) | ((base_target - 16) << 4));

  if (base_boost)
    data.state[5] |= 0x90;
  if (base_swing == climate::CLIMATE_SWING_VERTICAL || base_swing == climate::CLIMATE_SWING_BOTH)
    data.state[8] |= 0x40;
  if (base_swing == climate::CLIMATE_SWING_HORIZONTAL || base_swing == climate::CLIMATE_SWING_BOTH)
    data.state[8] |= 0x80;

  data.state[11] = this->ifeel_enabled_ ? KELON168_FOLLOW_ME_ENABLED : 0x00;
  data.state[12] = this->ifeel_enabled_ ? this->ifeel_temperature_ : 0x00;
  data.state[15] = command;
  Kelon168Protocol::checksum(&data);
  return data;
}

Kelon168Data ACHIClimate::build_ifeel_state_(uint8_t temperature, bool enabled, bool update) const {
  auto data = this->build_kelon_state_from_current_(update ? KELON168_COMMAND_LIGHT : KELON168_COMMAND_IFEEL);

  // These bytes match the captured Kelon168 FollowMe frames from the donor IR project.
  // The rest of the frame is built from the current UART state so normal AC settings
  // are preserved as much as the protocol allows.
  data.state[6] = update ? 0x08 : 0x87;
  data.state[7] = update ? 0x03 : 0x3B;

  // Important: keep iFeel frames neutral for louvers. On this indoor unit the
  // swing bits (state[2] bit 0x80 and state[8] bits 0x40/0x80) are safe in
  // dedicated swing commands, but if they are also present in an iFeel/follow-me
  // frame the unit may treat the frame as a louver command and stop active swing.
  // So iFeel preserves fan/mode/temperature, but does not encode vertical or
  // horizontal swing bits at all. Existing louver motion is left untouched.
  data.state[2] &= ~0x80;
  data.state[8] &= ~0xC0;

  data.state[11] = enabled ? KELON168_FOLLOW_ME_ENABLED : 0x00;
  data.state[12] = enabled ? temperature : 0x00;
  data.state[15] = update ? KELON168_COMMAND_LIGHT : KELON168_COMMAND_IFEEL;
  Kelon168Protocol::checksum(&data);
  return data;
}

std::string ACHIClimate::kelon168_to_hex_(const Kelon168Data &data) const {
  char buffer[KELON168_STATE_LENGTH * 3 + 1];
  size_t pos = 0;
  for (uint8_t i = 0; i < KELON168_STATE_LENGTH; i++) {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "%02X%s", data.state[i],
                    i + 1 == KELON168_STATE_LENGTH ? "" : " ");
  }
  return std::string(buffer);
}

std::string ACHIClimate::kelon168_to_json_(const Kelon168Data &data, const char *kind, bool enabled,
                                            uint8_t temperature) const {
  std::string bytes = this->kelon168_to_hex_(data);
  char payload[256];
  snprintf(payload, sizeof(payload),
           "{\"protocol\":\"kelon168\",\"kind\":\"%s\",\"command\":\"0x%02X\",\"enabled\":%s,\"temperature\":%u,\"bytes\":\"%s\"}",
           kind, data.command(), enabled ? "true" : "false", static_cast<unsigned>(temperature), bytes.c_str());
  return std::string(payload);
}

bool ACHIClimate::transmit_kelon_ir_(const Kelon168Data &data) {
  if (this->ir_transmitter_ == nullptr)
    return false;

  auto transmit = this->ir_transmitter_->transmit();
  Kelon168Protocol().encode(transmit.get_data(), data);
  log_kelon168_data("Sending", data);
  transmit.perform();
  return true;
}

bool ACHIClimate::publish_kelon_mqtt_(const Kelon168Data &data, const char *kind, bool enabled, uint8_t temperature) {
  if (this->ifeel_mqtt_topic_.empty())
    return false;

#ifndef USE_MQTT
  ESP_LOGW(TAG, "iFeel MQTT topic is configured, but ESPHome mqtt: component is not enabled");
  return false;
#else
  if (mqtt::global_mqtt_client == nullptr || !mqtt::global_mqtt_client->is_connected()) {
    ESP_LOGW(TAG, "iFeel MQTT command not published because MQTT is not connected");
    return false;
  }

  const std::string payload =
      (this->ifeel_mqtt_payload_format_ == IFEEL_MQTT_PAYLOAD_JSON)
          ? this->kelon168_to_json_(data, kind, enabled, temperature)
          : this->kelon168_to_hex_(data);

  const bool ok = mqtt::global_mqtt_client->publish(this->ifeel_mqtt_topic_, payload,
                                                    this->ifeel_mqtt_qos_, this->ifeel_mqtt_retain_);
  if (ok) {
    ESP_LOGI(TAG, "Published iFeel Kelon168 %s to MQTT topic '%s': %s", kind,
             this->ifeel_mqtt_topic_.c_str(), payload.c_str());
  } else {
    ESP_LOGW(TAG, "Failed to publish iFeel Kelon168 %s to MQTT topic '%s'", kind,
             this->ifeel_mqtt_topic_.c_str());
  }
  return ok;
#endif
}

void ACHIClimate::emit_kelon_ifeel_(Kelon168Data data, const char *kind, bool enabled, uint8_t temperature) {
  Kelon168Protocol::checksum(&data);
  const bool sent_ir = this->transmit_kelon_ir_(data);
  const bool published_mqtt = this->publish_kelon_mqtt_(data, kind, enabled, temperature);

  if (!sent_ir && !published_mqtt) {
    ESP_LOGW(TAG, "iFeel Kelon168 command was formed but not sent: configure ir_transmitter_id and/or ifeel_mqtt_topic");
  }
}

void ACHIClimate::send_ifeel(float temperature, bool enabled) {
  const bool have_actual = !this->last_status_frame_.empty();
  const bool is_on = have_actual ? this->power_on_ : this->d_power_on_;

  if (!is_on) {
    this->ifeel_enabled_ = false;
    this->ifeel_temperature_ = 0;
    ESP_LOGD(TAG, "Skipping iFeel IR command because AC is off");
    return;
  }

  uint8_t temp_c = this->ifeel_temperature_;
  if (enabled) {
    if (!std::isfinite(temperature)) {
      ESP_LOGW(TAG, "Skipping iFeel IR command because temperature is unavailable");
      return;
    }
    int rounded = static_cast<int>(std::lround(temperature));
    rounded = std::max(0, std::min(50, rounded));
    temp_c = static_cast<uint8_t>(rounded);
  }

  const bool state_changed = this->ifeel_enabled_ != enabled;
  this->ifeel_enabled_ = enabled;
  this->ifeel_temperature_ = enabled ? temp_c : 0;

  ESP_LOGD(TAG, "iFeel IR: enabled=%s temperature=%u°C state_changed=%s",
           enabled ? "true" : "false", static_cast<unsigned>(this->ifeel_temperature_),
           state_changed ? "true" : "false");

  if (!enabled) {
    // Always send OFF when requested. After reboot we may not know whether the remote
    // had previously enabled iFeel, so a forced OFF frame is safer than relying on memory.
    auto off = this->build_ifeel_state_(0, false, false);
    this->emit_kelon_ifeel_(off, "off", false, 0);
    return;
  }

  if (state_changed) {
    // The dedicated iFeel ON/OFF command (0x0D) is acknowledged by this indoor
    // unit with a beep and may also wake the front display. The temperature
    // update frame (0x00) with FollowMe enabled is accepted by the unit as a
    // silent FollowMe/iFeel update. Therefore, when the user has disabled
    // command sound or wants the display to stay off, do not send the noisy
    // 0x0D ON frame; send only the silent update below.
    const bool allow_audible_ifeel_on = this->command_sound_enabled_ && this->d_led_;
    if (allow_audible_ifeel_on) {
      auto on = this->build_ifeel_state_(this->ifeel_temperature_, true, false);
      this->emit_kelon_ifeel_(on, "on", true, this->ifeel_temperature_);
    } else {
      ESP_LOGI(TAG,
               "Skipping audible/display iFeel ON frame; using silent update only "
               "(command_sound=%s, led=%s)",
               this->command_sound_enabled_ ? "ON" : "OFF", this->d_led_ ? "ON" : "OFF");
    }
  }

  // While enabled, send an update frame every time the action is called so the AC
  // receives the current external temperature from HA/ESPHome.
  auto update = this->build_ifeel_state_(this->ifeel_temperature_, true, true);
  this->emit_kelon_ifeel_(update, "update", true, this->ifeel_temperature_);
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
  if (b.size() < 49) {
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
  last_raw_wind_ = raw_wind;
  fan_turbo_ = false;
  if (power_on_) {
    if (raw_wind == 0 || raw_wind == 1 || raw_wind == 2) fan_ = climate::CLIMATE_FAN_AUTO;
    else if (raw_wind == 10) fan_ = climate::CLIMATE_FAN_QUIET;
    else if (raw_wind == 12) fan_ = climate::CLIMATE_FAN_LOW;
    else if (raw_wind == 14) fan_ = climate::CLIMATE_FAN_MEDIUM;
    else if (raw_wind == 16) fan_ = climate::CLIMATE_FAN_HIGH;
    else if (raw_wind == 18) {
      fan_ = climate::CLIMATE_FAN_HIGH;
      fan_turbo_ = true;
    } else fan_ = climate::CLIMATE_FAN_AUTO;
  } else {
    fan_ = climate::CLIMATE_FAN_AUTO;   // when off, fan mode is irrelevant
    fan_turbo_ = false;
  }

  // Sleep stage.
  // Some units report the live status as direct values 0..4,
  // while writes still use the encoded form (value<<1)|1.
  uint8_t raw_sleep = b[IDX_SLEEP];
  switch (raw_sleep) {
    case 0x00: sleep_stage_ = 0; break;
    case 0x01: sleep_stage_ = 1; break;
    case 0x02: sleep_stage_ = 2; break;
    case 0x03: sleep_stage_ = 1; break;
    case 0x04: sleep_stage_ = 3; break;
    case 0x05: sleep_stage_ = 2; break;
    case 0x09: sleep_stage_ = 3; break;
    case 0x11: sleep_stage_ = 4; break;
    default:   sleep_stage_ = 0; break;
  }

  // Target temperature (direct value)
  target_c_ = b[IDX_SET_TEMP];
  if (target_c_ < 16 || target_c_ > 30) target_c_ = 24; // fallback

  // Current temperatures
  current_temperature = b[IDX_CURRENT_TEMP];

#ifdef USE_SENSOR
  if (pipe_sensor_ != nullptr) pipe_sensor_->publish_state(b[IDX_PIPE_TEMP]);
#endif

  // Turbo, Eco, Quiet, LED
  uint8_t features = b[IDX_RX_SWING_TURBO_ECO];   // byte 35 in status frame
  turbo_ = (features & TURBO_MASK) != 0;
  eco_   = (features & ECO_MASK) != 0;
  quiet_ = (b[IDX_RX_QUIET] & QUIET_MASK) != 0;   // byte 36 in status frame
  led_   = (b[IDX_RX_LED] & LED_MASK) != 0;       // byte 37 in status frame

  // On this model Quiet is signaled by the quiet flag, while raw_wind may remain 2.
  if (quiet_) {
    fan_ = climate::CLIMATE_FAN_QUIET;
    fan_turbo_ = false;
  } else if ((mode_ == climate::CLIMATE_MODE_DRY || mode_ == climate::CLIMATE_MODE_FAN_ONLY) && raw_wind == 10) {
    // In DRY/FAN_ONLY the unit can report raw wind code 10 with Quiet Mode Code = 0.
    // Treat that as the unit's internal/automatic airflow, not as an explicit
    // HA Quiet fan request. This prevents QUIET from being learned and carried
    // into the next mode change.
    fan_ = climate::CLIMATE_FAN_AUTO;
    fan_turbo_ = false;
  } else if (turbo_ && raw_wind == 18) {
    // BOOST preset also uses raw Wind Mode Code 18, but it should stay a preset
    // in HA rather than being shown as custom fan mode Turbo.
    fan_turbo_ = false;
    fan_ = (mode_ == climate::CLIMATE_MODE_HEAT) ? climate::CLIMATE_FAN_AUTO
                                                 : climate::CLIMATE_FAN_HIGH;
  } else if (raw_wind == 18) {
    fan_turbo_ = true;
    fan_ = climate::CLIMATE_FAN_HIGH;
  }

  // Swing
  bool updown = (features & UPDOWN_MASK) != 0;
  bool leftright = (features & LEFTRIGHT_MASK) != 0;
  if (updown && leftright) swing_ = climate::CLIMATE_SWING_BOTH;
  else if (updown) swing_ = climate::CLIMATE_SWING_VERTICAL;
  else if (leftright) swing_ = climate::CLIMATE_SWING_HORIZONTAL;
  else swing_ = climate::CLIMATE_SWING_OFF;

  // Recalculate actual signature
  recalc_actual_sig_();

  // Publish state with gating
  publish_gated_state_();
  update_led_switch_state_();

  // Keep still-unmapped candidate fields available only in verbose logs.
  ESP_LOGV(TAG,
           "Extended status: compressor_set=%uHz compressor=%uHz exhaust=%u°C raw_b47=%u/%d raw_b48=%u/%d",
           b[IDX_COMP_FREQ_SET], b[IDX_COMP_FREQ], b[IDX_COMPRESSOR_EXHAUST_TEMP],
           b[47], static_cast<int8_t>(b[47]),
           b[48], static_cast<int8_t>(b[48]));

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
  if (compressor_exhaust_temp_sensor_ != nullptr) {
    compressor_exhaust_temp_sensor_->publish_state(static_cast<float>(b[IDX_COMPRESSOR_EXHAUST_TEMP]));
  }
#endif

#ifdef USE_TEXT_SENSOR
  if (power_status_text_ != nullptr) {
    power_status_text_->publish_state(power_on_ ? "ON" : "OFF");
  }
#endif

  ESP_LOGD(TAG,
           "Parsed: power=%s, mode=%s, fan=%s, swing=%s, target=%u°C, current=%.1f°C, outdoor=%d°C, compressor=%uHz, exhaust=%u°C",
           power_on_ ? "ON" : "OFF",
           climate::climate_mode_to_string(mode_),
           climate::climate_fan_mode_to_string(fan_),
           climate::climate_swing_mode_to_string(swing_),
           (unsigned) target_c_, current_temperature,
           static_cast<int8_t>(b[IDX_OUTDOOR_TEMP]),
           b[IDX_COMP_FREQ], b[IDX_COMPRESSOR_EXHAUST_TEMP]);

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
void ACHIClimate::publish_fan_state_(bool turbo_fan, climate::ClimateFanMode fan) {
  if (turbo_fan) {
    this->set_custom_fan_mode_(CUSTOM_FAN_TURBO);
  } else {
    this->set_fan_mode_(fan);
  }
}

void ACHIClimate::publish_gated_state_() {
  if (accept_remote_changes_) {
    // Publish actual state (from AC). Some Hisense units do not return explicit
    // Turbo/ECO/Sleep/Quiet bits when the front display is off. They acknowledge
    // those presets indirectly via target temperature and raw fan/wind code:
    //   BOOST -> target 16/30 and raw wind 18
    //   ECO   -> target 24 and raw wind 10
    //   SLEEP -> previous target+1 and raw wind 10
    // If the last HA-selected preset still matches that indirect state, keep it
    // visible in HA instead of immediately publishing Preset=None on the next
    // status frame.
    bool out_turbo = turbo_;
    bool out_eco = eco_;
    bool out_quiet = quiet_;
    uint8_t out_sleep_stage = sleep_stage_;
    auto out_fan = fan_;
    bool out_fan_turbo = fan_turbo_;

    if (power_on_ && d_power_on_ && mode_ == d_mode_ && target_c_ == d_target_c_) {
      if (d_turbo_ && last_raw_wind_ == 18) {
        out_turbo = true;
        out_eco = false;
        out_quiet = false;
        out_sleep_stage = 0;
        out_fan = d_fan_;
        out_fan_turbo = false;
      } else if (d_eco_ && last_raw_wind_ == 10) {
        out_turbo = false;
        out_eco = true;
        out_quiet = false;
        out_sleep_stage = 0;
        out_fan = d_fan_;
        out_fan_turbo = false;
      } else if (d_sleep_stage_ > 0 && last_raw_wind_ == 10) {
        out_turbo = false;
        out_eco = false;
        out_quiet = false;
        out_sleep_stage = d_sleep_stage_;
        out_fan = d_fan_;
        out_fan_turbo = false;
      } else if (d_quiet_ && (quiet_ || fan_ == climate::CLIMATE_FAN_QUIET)) {
        out_turbo = false;
        out_eco = false;
        out_quiet = true;
        out_sleep_stage = 0;
        out_fan = climate::CLIMATE_FAN_QUIET;
        out_fan_turbo = false;
      } else if (d_fan_turbo_ && last_raw_wind_ == 18) {
        out_turbo = false;
        out_eco = false;
        out_quiet = false;
        out_sleep_stage = 0;
        out_fan = d_fan_;
        out_fan_turbo = true;
      }
    }

    if (memory_mode_enabled_ && power_on_ && mode_ != climate::CLIMATE_MODE_OFF) {
      last_active_mode_ = mode_;
    }

    this->mode = power_on_ ? mode_ : climate::CLIMATE_MODE_OFF;
    this->target_temperature = target_c_;
    publish_fan_state_(out_fan_turbo, out_fan);
    this->swing_mode = swing_;
    if (enable_presets_) {
      if (out_turbo) this->set_preset_(climate::CLIMATE_PRESET_BOOST);
      else if (out_eco) this->set_preset_(climate::CLIMATE_PRESET_ECO);
      else if (out_sleep_stage > 0) this->set_preset_(climate::CLIMATE_PRESET_SLEEP);
      else if (out_quiet) this->set_custom_preset_(CUSTOM_PRESET_QUIET);
      else this->set_preset_(climate::CLIMATE_PRESET_NONE);
    }
    // Sync desired with the effective published state, not only with raw flags.
    // This keeps indirect display-off presets stable while still allowing real
    // remote/HA changes to clear them when the indirect state no longer matches.
    d_power_on_    = power_on_;
    d_mode_        = mode_;
    d_target_c_    = target_c_;
    d_fan_         = out_fan;
    d_fan_turbo_   = out_fan_turbo;
    d_swing_       = swing_;
    d_eco_         = out_eco;
    d_turbo_       = out_turbo;
    d_quiet_       = out_quiet;
    d_led_         = led_;
    d_sleep_stage_ = out_sleep_stage;
    recalc_desired_sig_();
  } else {
    // Publish desired state (while enforcing)
    this->mode = d_power_on_ ? d_mode_ : climate::CLIMATE_MODE_OFF;
    this->target_temperature = d_target_c_;
    publish_fan_state_(d_fan_turbo_, d_fan_);
    this->swing_mode = d_swing_;
    if (enable_presets_) {
      if (d_turbo_) this->set_preset_(climate::CLIMATE_PRESET_BOOST);
      else if (d_eco_) this->set_preset_(climate::CLIMATE_PRESET_ECO);
      else if (d_sleep_stage_ > 0) this->set_preset_(climate::CLIMATE_PRESET_SLEEP);
      else if (d_quiet_) this->set_custom_preset_(CUSTOM_PRESET_QUIET);
      else this->set_preset_(climate::CLIMATE_PRESET_NONE);
    }
  }
  publish_state();
}

void ACHIClimate::update_led_switch_state_() {
  // Dependency: when the display switch is OFF, command sound must stay ON.
  // With this indoor unit, keeping the display off during user climate commands
  // requires sending LED_OFF, and that action can itself make the unit beep.
  if (!d_led_ && !command_sound_enabled_) {
    command_sound_enabled_ = true;
    ESP_LOGD(TAG, "Command sound forced ON because display switch is OFF");
    update_sound_switch_state_();
  }

  if (led_switch_ == nullptr) return;
  led_switch_->publish_state(d_led_);
}

void ACHIClimate::update_sound_switch_state_() {
  if (sound_switch_ == nullptr) return;
  sound_switch_->publish_state(command_sound_enabled_);
}

void ACHIClimate::update_memory_switch_state_() {
  if (memory_switch_ == nullptr) return;
  memory_switch_->publish_state(memory_mode_enabled_);
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
                                                 climate::ClimateFanMode fan, bool fan_turbo,
                                                 climate::ClimateSwingMode swing,
                                                 bool eco, bool turbo, bool quiet, bool led,
                                                 uint8_t sleep_stage, uint8_t target_c) const {
  // In OFF state many indoor units keep reporting the last active mode while power is already off.
  // Normalize non-power fields so we do not get stuck in an endless enforce/write loop.
  if (!power) {
    // In OFF state this indoor unit may keep the last active mode/temperature/fan
    // in the status frame. Treat OFF as a single converged state so HA does not
    // keep re-sending power-off frames and block a later remote power-on.
    mode = climate::CLIMATE_MODE_OFF;
    fan = climate::CLIMATE_FAN_AUTO;
    fan_turbo = false;
    swing = climate::CLIMATE_SWING_OFF;
    eco = false;
    turbo = false;
    quiet = false;
    led = true;
    sleep_stage = 0;
    target_c = 24;
  }

  // The display/LED byte behaves like an action, not a stable climate field.
  // Ignore it for convergence even when the optional LED switch exists. This
  // prevents a climate command that temporarily wakes the display from causing
  // endless HA-priority re-sends or an automatic LED_ON command. The LED switch
  // itself still sends one explicit 0xC0/0x40 command when toggled.
  led = true;

  uint32_t h = 2166136261u;
  auto mix = [&h](uint32_t x) {
    h ^= x;
    h *= 16777619u;
  };
  mix(power ? 1u : 0u);
  mix(static_cast<uint32_t>(mode));
  mix(static_cast<uint32_t>(fan));
  mix(fan_turbo ? 1u : 0u);
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
  desired_sig_ = compute_control_signature_(d_power_on_, d_mode_, d_fan_, d_fan_turbo_, d_swing_,
                                            d_eco_, d_turbo_, d_quiet_, d_led_, d_sleep_stage_, d_target_c_);
}

void ACHIClimate::recalc_actual_sig_() {
  bool effective_eco = eco_;
  bool effective_turbo = turbo_;
  bool effective_quiet = quiet_;
  uint8_t effective_sleep_stage = sleep_stage_;
  auto effective_fan = fan_;
  bool effective_fan_turbo = fan_turbo_;

  // Some Hisense indoor units acknowledge special modes only indirectly in
  // status frames. For HA-priority convergence, accept those indirect states
  // without changing the normal published UI state. This stops repeated silent
  // writes after BOOST/ECO/SLEEP while preserving the original preset display
  // behavior from the working baseline.
  if (ha_priority_active_ && d_power_on_ && power_on_ && mode_ == d_mode_) {
    if (d_fan_turbo_ && target_c_ == d_target_c_ && last_raw_wind_ == 18) {
      effective_fan_turbo = true;
      effective_turbo = false;
      effective_eco = false;
      effective_quiet = false;
      effective_sleep_stage = 0;
      effective_fan = d_fan_;
    } else if (d_turbo_ && target_c_ == d_target_c_) {
      effective_turbo = true;
      effective_eco = false;
      effective_quiet = false;
      effective_sleep_stage = 0;
      effective_fan_turbo = false;
      effective_fan = d_fan_;
    } else if (d_eco_ && target_c_ == d_target_c_ && fan_ == climate::CLIMATE_FAN_QUIET) {
      effective_eco = true;
      effective_turbo = false;
      effective_quiet = false;
      effective_sleep_stage = 0;
      effective_fan_turbo = false;
      effective_fan = d_fan_;
    } else if (d_sleep_stage_ > 0 && target_c_ == d_target_c_ && fan_ == climate::CLIMATE_FAN_QUIET) {
      effective_sleep_stage = d_sleep_stage_;
      effective_eco = false;
      effective_turbo = false;
      effective_quiet = false;
      effective_fan_turbo = false;
      effective_fan = d_fan_;
    } else if (d_quiet_ && fan_ == climate::CLIMATE_FAN_QUIET) {
      effective_quiet = true;
      effective_eco = false;
      effective_turbo = false;
      effective_sleep_stage = 0;
      effective_fan_turbo = false;
      effective_fan = d_fan_;
    }
  }

  actual_sig_ = compute_control_signature_(power_on_, mode_, effective_fan, effective_fan_turbo, swing_,
                                           effective_eco, effective_turbo, effective_quiet,
                                           led_, effective_sleep_stage, target_c_);
}

void ACHIClimate::log_sig_diff_() const {
  // Optional verbose diff – can be enabled for debugging
}

// ---- External LED control ----
void ACHIClimate::set_desired_led(bool on) {
  d_led_ = on;

  // Dependency: turning the display OFF also turns command sound ON.
  // This keeps HA from showing an unsupported combination for this protocol.
  if (!on && !command_sound_enabled_) {
    command_sound_enabled_ = true;
    ESP_LOGD(TAG, "Command sound forced ON because display switch was turned OFF");
    update_sound_switch_state_();
  }

  led_command_pending_ = true;
  accept_remote_changes_ = false;
  ha_priority_active_ = true;
  recalc_desired_sig_();

  pending_control_ = true;
  last_control_ms_ = millis();
  user_command_next_write_ = true;
  beep_on_next_write_ = command_sound_enabled_;

  ESP_LOGD(TAG, "LED switch: desired_led=%s, command_sound=%s, pending write",
           on ? "ON" : "OFF", command_sound_enabled_ ? "ON" : "OFF");
  // update_led_switch_state_() will be called from loop after publish
}

// ---- External command sound control ----
void ACHIClimate::set_memory_mode_enabled(bool on) {
  memory_mode_enabled_ = on;

  if (memory_mode_enabled_) {
    // If Memory is enabled while the unit is already running, seed the memory
    // immediately from the real/published working mode.
    if (d_power_on_ && d_mode_ != climate::CLIMATE_MODE_OFF) {
      last_active_mode_ = d_mode_;
    } else if (power_on_ && mode_ != climate::CLIMATE_MODE_OFF) {
      last_active_mode_ = mode_;
    }
  } else if (!d_power_on_) {
    // With Memory OFF, keep the previous/original OFF fallback behavior.
    d_mode_ = climate::CLIMATE_MODE_COOL;
    recalc_desired_sig_();
  }

  update_memory_switch_state_();
  ESP_LOGD(TAG, "Memory switch: %s", memory_mode_enabled_ ? "ON" : "OFF");
}

void ACHIClimate::set_command_sound_enabled(bool on) {
  if (!on && !d_led_) {
    // The display is currently desired OFF. In this state user commands need
    // LED_OFF to keep the panel dark, and LED_OFF is audible on this unit.
    // Keep the sound switch ON so the UI reflects the real supported state.
    command_sound_enabled_ = true;
    update_sound_switch_state_();
    ESP_LOGD(TAG, "Command sound stays ON while display switch is OFF");
    return;
  }

  command_sound_enabled_ = on;
  update_sound_switch_state_();
  ESP_LOGD(TAG, "Command sound: %s", on ? "ON" : "OFF");
}

// ---- Field encoders ----
uint8_t ACHIClimate::encode_mode_hi_nibble_(climate::ClimateMode m) {
  return static_cast<uint8_t>(encode_nibble_from_mode(m) << 4);
}

uint8_t ACHIClimate::encode_fan_byte_(climate::ClimateFanMode f, bool turbo_fan) {
  uint8_t code = 1;
  if (turbo_fan) {
    code = 18;
    return static_cast<uint8_t>(code + 1);   // writing requires +1
  }
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