#include "ac_hi.h"
#include <cmath>
#include <cstdio>
#include <string>
#include <Arduino.h>
#include "esphome/core/log.h"
#ifdef USE_API
#include "esphome/components/api/api_server.h"
#endif
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

static const char *const CUSTOM_PRESET_QUIET = "Quiet";
static const char *const CUSTOM_PRESET_HEAT_8C = "+8°C";
static const char *const CUSTOM_FAN_TURBO = "Turbo";
static const char *const SLEEP_PROGRAM_1 = "Sleep 1";
static const char *const SLEEP_PROGRAM_2 = "Sleep 2";
static const char *const SLEEP_PROGRAM_3 = "Sleep 3";
static const char *const SLEEP_PROGRAM_4 = "Sleep 4";
static constexpr uint32_t SLEEP_LED_RESTORE_TIMEOUT_MS = 10000;

static const char *sleep_program_for_stage(uint8_t stage) {
  switch (stage) {
    case 1: return SLEEP_PROGRAM_1;
    case 2: return SLEEP_PROGRAM_2;
    case 3: return SLEEP_PROGRAM_3;
    case 4: return SLEEP_PROGRAM_4;
    default: return SLEEP_PROGRAM_2;
  }
}

static uint8_t sleep_stage_from_program(const std::string &program) {
  if (program == SLEEP_PROGRAM_1) return 1;
  if (program == SLEEP_PROGRAM_2) return 2;
  if (program == SLEEP_PROGRAM_3) return 3;
  if (program == SLEEP_PROGRAM_4) return 4;
  return 0;
}

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
    // SMART/AUTO exposes internally selected branches through status codes:
    // 4 = idle/fan, 5 = heat, 6 = cool, 7 = dehumidification. Home Assistant
    // must still see one stable HVAC mode: AUTO for every SMART branch.
    case 0x04:
    case 0x05:
    case 0x06:
    case 0x07:
      return climate::CLIMATE_MODE_AUTO;
    default:   return climate::CLIMATE_MODE_COOL;
  }
}

static inline uint8_t encode_nibble_from_mode(climate::ClimateMode m) {
  switch (m) {
    case climate::CLIMATE_MODE_FAN_ONLY: return 0x01;
    case climate::CLIMATE_MODE_HEAT:     return 0x03;
    case climate::CLIMATE_MODE_COOL:     return 0x05;
    case climate::CLIMATE_MODE_DRY:      return 0x07;
    // AUTO/SMART is command action 9, therefore byte 18 becomes 0x9C while
    // powered on (0x90 mode action + 0x0C power bits).
    case climate::CLIMATE_MODE_AUTO:     return 0x09;
    default:                             return 0x05;
  }
}

#ifdef USE_SENSOR
void ACHIClimate::publish_sensor_if_changed_(sensor::Sensor *sensor, float value) {
  if (sensor != nullptr && (!sensor->has_state() || sensor->get_raw_state() != value)) {
    sensor->publish_state(value);
  }
}
#endif

#ifdef USE_TEXT_SENSOR
void ACHIClimate::publish_text_sensor_if_changed_(text_sensor::TextSensor *sensor, const char *value) {
  if (sensor != nullptr && (!sensor->has_state() || sensor->get_raw_state() != value)) {
    sensor->publish_state(value);
  }
}
#endif

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

// ---- ACHISleepProgramSelect ----
void ACHISleepProgramSelect::control(const std::string &value) {
  if (parent_ != nullptr) {
    parent_->set_sleep_program(value);
  } else {
    publish_state(value);
  }
}

// ---- ACHIClimate implementation ----

void ACHIClimate::setup() {
  // Register custom presets on the Climate entity, not on ClimateTraits.
  // ClimateTraits::set_supported_custom_presets() is deprecated and will be removed in ESPHome 2026.11.0.
  if (enable_presets_) {
    // Until ProductType is known, expose the complete compatible list. The
    // 0x66/0x40 reply will remove unsupported custom presets and refresh HA.
    this->set_supported_custom_presets({CUSTOM_PRESET_QUIET, CUSTOM_PRESET_HEAT_8C});
  }
  // Fan Turbo is exposed as a custom fan mode, not as a preset.
  // It only sends raw Wind Mode Code 18 without changing target temperature.
  this->set_supported_custom_fan_modes({CUSTOM_FAN_TURBO});

  // Initial HA‑visible state
  mode = climate::CLIMATE_MODE_OFF;
  target_temperature = 24;
  fan_mode = climate::CLIMATE_FAN_AUTO;
  swing_mode = climate::CLIMATE_SWING_OFF;
  action = climate::CLIMATE_ACTION_OFF;
  // Desired state mirrors initial
  d_power_on_     = false;
  d_mode_         = climate::CLIMATE_MODE_OFF;
  last_active_mode_ = climate::CLIMATE_MODE_COOL;
  d_target_c_     = 24;
  last_cool_target_c_ = 24;
  last_heat_target_c_ = 24;
  d_fan_          = climate::CLIMATE_FAN_AUTO;
  d_fan_turbo_    = false;
  d_swing_        = climate::CLIMATE_SWING_OFF;
  d_turbo_        = false;
  d_eco_          = false;
  d_quiet_        = false;
  d_heat_8c_      = false;
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
  update_sleep_program_select_state_();
#ifdef USE_TEXT_SENSOR
  publish_text_sensor_if_changed_(device_capabilities_text_, "Waiting for 0x66/0x40");
  publish_text_sensor_if_changed_(ac_active_faults_text_, "Waiting for status");
  publish_text_sensor_if_changed_(power_on_timer_text_, "Waiting for timer event");
  publish_text_sensor_if_changed_(power_off_timer_text_, "Waiting for timer event");
#endif

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

  if (!writing_lock_ && !pending_control_ && !status_query_in_flight_) {
    const bool capability_due =
        !capabilities_.valid && !last_status_frame_.empty() &&
        capabilities_attempts_ < CAPABILITIES_MAX_ATTEMPTS &&
        millis() >= capabilities_next_attempt_ms_;
    if (capability_due) {
      send_query_capabilities_();
    } else {
      send_query_status_();
    }
  } else {
    ESP_LOGV(TAG, "Polling skipped (write lock, pending control, or query active)");
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

  // 5. Release a lost status request. Until this timeout expires, a pending
  // 0x65 command is deliberately held back so it cannot collide with the 0x66 response.
  if (status_query_in_flight_ && (millis() - status_query_time_ > STATUS_QUERY_TIMEOUT)) {
    const char *kind = query_kind_ == QUERY_CAPABILITIES ? "Capabilities 0x66/0x40" : "Status 0x66/0x00";
    ESP_LOGW(TAG, "%s query timeout after %lums; allowing pending control", kind,
             (unsigned long) STATUS_QUERY_TIMEOUT);
    if (query_kind_ == QUERY_CAPABILITIES && !capabilities_.valid) {
      capabilities_next_attempt_ms_ = millis() + CAPABILITIES_RETRY_MS;
#ifdef USE_TEXT_SENSOR
      if (capabilities_attempts_ >= CAPABILITIES_MAX_ATTEMPTS) {
        publish_text_sensor_if_changed_(device_capabilities_text_, "No 0x66/0x40 reply");
      }
#endif
    }
    status_query_in_flight_ = false;
    query_kind_ = QUERY_NONE;
  }

  // 6. Handle write lock timeout
  if (writing_lock_ && (millis() - write_lock_time_ > WRITE_LOCK_TIMEOUT)) {
    ESP_LOGW(TAG, "Write lock timeout, forcing unlock");
    writing_lock_ = false;
  }

  // 7. Send a debounced control command only when neither a write ACK nor a
  // status response is outstanding. This serializes 0x65 and 0x66 transactions.
  if (pending_control_ && !writing_lock_ && !status_query_in_flight_ &&
      (millis() - last_control_ms_ >= CONTROL_DEBOUNCE_MS)) {
    send_write_changes_();
    pending_control_ = pending_command_fields_ != CMD_FIELD_NONE;
  }

#ifdef USE_API
  // Climate traits are entity metadata. After ProductType changes the preset
  // list, reconnect native API clients once so Home Assistant requests fresh
  // entity information and removes unsupported presets from its selector.
  if (capability_api_refresh_pending_ && millis() >= capability_api_refresh_at_ms_) {
    capability_api_refresh_pending_ = false;
    if (api::global_api_server != nullptr && api::global_api_server->is_connected()) {
      ESP_LOGI(TAG, "Refreshing Home Assistant climate capabilities after 0x66/0x40");
      for (const auto &client : api::global_api_server->active_clients()) {
        if (client != nullptr) client->on_fatal_error();
      }
    }
  }
#endif

  // 8. Keep locally latched relative timers counting down. The indoor unit
  // announces timer changes only briefly and then returns zero in these bytes.
  update_timer_countdowns_();

  // 9. Optional memory diagnostics
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
    climate::CLIMATE_MODE_FAN_ONLY,
    // Keep AUTO last so generic climate.turn_on retains the previous COOL
    // fallback when the optional Memory switch is disabled.
    climate::CLIMATE_MODE_AUTO
  });
  t.set_supported_fan_modes({climate::CLIMATE_FAN_AUTO, climate::CLIMATE_FAN_LOW,
                             climate::CLIMATE_FAN_MEDIUM, climate::CLIMATE_FAN_HIGH,
                             climate::CLIMATE_FAN_QUIET});
  t.set_supported_swing_modes({climate::CLIMATE_SWING_OFF, climate::CLIMATE_SWING_VERTICAL,
                               climate::CLIMATE_SWING_HORIZONTAL, climate::CLIMATE_SWING_BOTH});
  if (enable_presets_) {
    climate::ClimatePresetMask presets;
    presets.insert(climate::CLIMATE_PRESET_NONE);
    // BOOST and SLEEP do not have reliable ProductType flags in the known
    // 0x66/0x40 map, so keep them available. ECO has a confirmed flag.
    if (!capabilities_.valid || capabilities_.power_save)
      presets.insert(climate::CLIMATE_PRESET_ECO);
    presets.insert(climate::CLIMATE_PRESET_BOOST);
    presets.insert(climate::CLIMATE_PRESET_SLEEP);
    t.set_supported_presets(presets);
  }
  t.set_visual_min_temperature(16);
  t.set_visual_max_temperature(30);
  t.set_visual_temperature_step(1.0f);
  t.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE);
  t.add_feature_flags(climate::CLIMATE_SUPPORTS_ACTION);
  return t;
}

// ---- Per-mode target temperature memory ----
void ACHIClimate::remember_target_for_mode_(climate::ClimateMode mode, uint8_t target_c) {
  target_c = std::max<uint8_t>(16, std::min<uint8_t>(30, target_c));
  if (mode == climate::CLIMATE_MODE_COOL) {
    last_cool_target_c_ = target_c;
  } else if (mode == climate::CLIMATE_MODE_HEAT) {
    last_heat_target_c_ = target_c;
  }
}

uint8_t ACHIClimate::target_for_mode_(climate::ClimateMode mode, uint8_t fallback) const {
  if (mode == climate::CLIMATE_MODE_COOL) return last_cool_target_c_;
  if (mode == climate::CLIMATE_MODE_HEAT) return last_heat_target_c_;
  return fallback;
}

// ---- Sleep program select ----
void ACHIClimate::set_sleep_program(const std::string &value) {
  const uint8_t stage = sleep_stage_from_program(value);
  if (stage == 0) {
    ESP_LOGW(TAG, "Unknown Sleep program: %s", value.c_str());
    update_sleep_program_select_state_();
    return;
  }

  const bool changed_selection = selected_sleep_stage_ != stage;
  selected_sleep_stage_ = stage;
  update_sleep_program_select_state_();

  // Merely choosing a program does not activate Sleep. If Sleep is already
  // confirmed (or an activation is currently pending), send a new one-shot
  // byte-17 action for the newly selected program.
  const bool sleep_active_or_pending = sleep_stage_ > 0 || d_sleep_stage_ > 0 || sleep_confirmation_pending_;
  if (!sleep_active_or_pending || !changed_selection) {
    ESP_LOGD(TAG, "Sleep program selected for next Sleep preset: %s", value.c_str());
    return;
  }

  d_sleep_stage_ = stage;
  // Preserve the fan snapshot captured before Sleep activation while switching
  // between Sleep 1..4. It is needed to restore the original fan on Sleep OFF.
  pending_command_fields_ |= CMD_FIELD_SLEEP;
  accept_remote_changes_ = false;
  ha_priority_active_ = true;
  recalc_desired_sig_();
  pending_control_ = true;
  last_control_ms_ = millis();
  user_command_next_write_ = true;
  beep_on_next_write_ = command_sound_enabled_;

  ESP_LOGD(TAG, "Changing active Sleep program to %s with a byte-17-only command",
           sleep_program_for_stage(stage));
}

// ---- Control from HA ----
void ACHIClimate::control(const climate::ClimateCall &call) {
  bool changed = false;
  bool was_power_on = d_power_on_;

  // Snapshot the desired state. After applying the ClimateCall we compare the
  // result with this snapshot and queue only the fields that really changed.
  const bool before_power_on = d_power_on_;
  const auto before_mode = d_mode_;
  const uint8_t before_target = d_target_c_;
  const auto before_fan = d_fan_;
  const bool before_fan_turbo = d_fan_turbo_;
  const auto before_swing = d_swing_;
  const bool before_turbo = d_turbo_;
  const bool before_eco = d_eco_;
  const bool before_quiet = d_quiet_;
  const bool before_heat_8c = d_heat_8c_;
  const bool before_led = d_led_;
  const uint8_t before_sleep_stage = d_sleep_stage_;

  if (call.get_mode().has_value()) {
    auto m = *call.get_mode();

    // Selecting another HVAC mode explicitly leaves frost protection. Merely
    // re-selecting HEAT keeps +8 °C active; Preset=None is its normal off path.
    if (d_heat_8c_ && m != climate::CLIMATE_MODE_HEAT) {
      d_heat_8c_ = false;
      heat_8c_restore_valid_ = false;
    }

    if (m == climate::CLIMATE_MODE_OFF) {
      // Preserve the normal setpoint of the mode being turned off. Temporary
      // Boost/Eco/Sleep targets are intentionally excluded.
      if (d_power_on_ && !d_turbo_ && !d_eco_ && !d_heat_8c_ && d_sleep_stage_ == 0) {
        remember_target_for_mode_(d_mode_, d_target_c_);
      }

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
                 LOG_STR_ARG(climate::climate_mode_to_string(requested_mode)),
                 LOG_STR_ARG(climate::climate_mode_to_string(last_active_mode_)));
        requested_mode = last_active_mode_;
      }

      // Store the normal target of the mode we are leaving and restore the
      // separately remembered target of COOL or HEAT. An explicitly supplied
      // target temperature in the same ClimateCall is processed below and wins.
      if ((!was_power_on || requested_mode != d_mode_) &&
          !d_turbo_ && !d_eco_ && !d_heat_8c_ && d_sleep_stage_ == 0) {
        if (was_power_on && requested_mode != d_mode_) {
          remember_target_for_mode_(d_mode_, d_target_c_);
        }
        const uint8_t restored_target = target_for_mode_(requested_mode, d_target_c_);
        if ((requested_mode == climate::CLIMATE_MODE_COOL || requested_mode == climate::CLIMATE_MODE_HEAT) &&
            restored_target != d_target_c_) {
          ESP_LOGD(TAG, "Mode %s: restoring remembered target %u°C",
                   LOG_STR_ARG(climate::climate_mode_to_string(requested_mode)),
                   (unsigned) restored_target);
        }
        d_target_c_ = restored_target;
      }

      d_mode_ = requested_mode;

      if (d_mode_ == climate::CLIMATE_MODE_AUTO) {
        // SMART owns its target temperature and indoor fan. Enter it with a
        // mode-only command and clear mutually exclusive manual features from
        // the desired state so they are not re-applied after the AC confirms
        // AUTO with status code 4, 5, 6 or 7.
        d_turbo_ = false;
        d_eco_ = false;
        d_quiet_ = false;
        d_heat_8c_ = false;
        heat_8c_restore_valid_ = false;
        d_fan_turbo_ = false;
        d_fan_ = climate::CLIMATE_FAN_AUTO;
        g_has_pre_turbo_target = false;
        g_has_pre_eco_state = false;
      }

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
          !d_quiet_ && !d_eco_ && !d_turbo_ && !d_heat_8c_ && d_sleep_stage_ == 0) {
        d_fan_ = climate::CLIMATE_FAN_AUTO;
      }
    }
    changed = true;
  }

  if (call.get_target_temperature().has_value()) {
    float t = *call.get_target_temperature();
    if (!std::isnan(t)) {
      // Hisense inverter models allow the SMART comfort temperature to be
      // adjusted while AUTO remains active. The UART command still carries the
      // ordinary absolute temperature byte; the indoor unit translates it into
      // the SMART comfort correction and continues selecting branches 4/5/6/7.
      // Send this as a one-shot field only. Convergence intentionally ignores
      // AUTO target changes so the component never fights later SMART decisions.
      d_heat_8c_ = false;
      heat_8c_restore_valid_ = false;
      uint8_t c = static_cast<uint8_t>(std::round(t));
      c = std::max<uint8_t>(16, std::min<uint8_t>(30, c));
      if (c != d_target_c_) {
        d_target_c_ = c;
        if (d_mode_ == climate::CLIMATE_MODE_AUTO) {
          ESP_LOGD(TAG,
                   "SMART/AUTO target adjustment requested: %u°C (one-shot, mode remains AUTO)",
                   (unsigned) c);
        } else {
          remember_target_for_mode_(d_mode_, c);
        }
        changed = true;
      }
    }
  }

  if (call.get_fan_mode().has_value()) {
    if (d_mode_ == climate::CLIMATE_MODE_AUTO) {
      ESP_LOGD(TAG, "Ignoring fan-mode command while SMART/AUTO is active");
    } else {
    // While Sleep is active, QUIET is controlled by the indoor unit itself.
    // Remember that this fan change is explicit so the next status parser does
    // not immediately replace the requested fan with Sleep's QUIET value.
    sleep_fan_override_pending_ = (sleep_stage_ > 0 || d_sleep_stage_ > 0);

    // An explicit fan command supersedes any pending Sleep rollback snapshot.
    sleep_restore_fan_valid_ = false;
    d_heat_8c_ = false;
    heat_8c_restore_valid_ = false;
    d_fan_ = *call.get_fan_mode();
    d_fan_turbo_ = false;
    d_quiet_ = (d_fan_ == climate::CLIMATE_FAN_QUIET);
    changed = true;
    }
  }

  if (call.get_swing_mode().has_value()) {
    d_swing_ = *call.get_swing_mode();
    changed = true;
  }

  if (call.get_preset().has_value()) {
    auto p = *call.get_preset();
    const bool preset_allowed_in_mode =
        d_mode_ != climate::CLIMATE_MODE_AUTO || p == climate::CLIMATE_PRESET_NONE;
    const bool preset_supported =
        !(p == climate::CLIMATE_PRESET_ECO && capabilities_.valid && !capabilities_.power_save);
    if (!preset_allowed_in_mode) {
      ESP_LOGD(TAG, "Ignoring preset command while SMART/AUTO is active");
    } else if (!preset_supported) {
      ESP_LOGW(TAG, "Ignoring unsupported ECO preset (ProductType power_save=0)");
    } else {
    bool was_turbo = d_turbo_;
    bool was_eco = d_eco_;
    bool was_heat_8c = d_heat_8c_;
    const bool was_sleep = before_sleep_stage > 0 || sleep_stage_ > 0;
    const auto fan_before_preset = d_fan_;
    const bool fan_turbo_before_preset = d_fan_turbo_;
    const bool quiet_before_preset = d_quiet_;

    // Preserve the pre-Sleep fan snapshot only while leaving an active Sleep
    // preset through Preset=None. Any other preset/fan choice supersedes it.
    const bool leaving_sleep_to_none =
        p == climate::CLIMATE_PRESET_NONE && was_sleep;
    if (p != climate::CLIMATE_PRESET_SLEEP && !leaving_sleep_to_none) {
      sleep_restore_fan_valid_ = false;
    }

    d_eco_ = false;
    d_turbo_ = false;
    d_heat_8c_ = false;
    d_sleep_stage_ = 0;
    if (p != climate::CLIMATE_PRESET_NONE && was_heat_8c)
      heat_8c_restore_valid_ = false;

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
      // Request the first Sleep program without changing the set temperature.
      // The UI does not treat this desired value as active Sleep: only a
      // non-zero Sleep Mode Code in a later status frame can confirm it.
      // Save the current fan state so it can be restored both when a Sleep
      // request is rejected and when a successfully activated Sleep is turned off.
      if (sleep_stage_ == 0) {
        sleep_restore_fan_ = fan_before_preset;
        sleep_restore_fan_turbo_ = fan_turbo_before_preset;
        sleep_restore_quiet_ = quiet_before_preset;
        sleep_restore_fan_valid_ = true;
        sleep_restore_led_ = d_led_;
        sleep_restore_led_valid_ = true;
        sleep_led_restore_pending_ = false;
        ESP_LOGD(TAG,
                 "Sleep request: saved previous fan=%s turbo=%s quiet=%s and LED=%s for restore",
                 LOG_STR_ARG(climate::climate_fan_mode_to_string(sleep_restore_fan_)),
                 sleep_restore_fan_turbo_ ? "YES" : "NO",
                 sleep_restore_quiet_ ? "YES" : "NO",
                 sleep_restore_led_ ? "ON" : "OFF");
      } else {
        // Sleep is already confirmed. Keep the original pre-Sleep fan snapshot
        // so changing Sleep 1..4 does not lose the value needed on Sleep OFF.
      }

      // Send only the selected Sleep action. Do not pre-emptively change
      // Wind/Quiet: a successful indoor-unit Sleep program will report its own
      // QUIET fan state in the following status frame.
      d_sleep_stage_ = selected_sleep_stage_;
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

      if (was_heat_8c && heat_8c_restore_valid_) {
        d_power_on_ = heat_8c_restore_power_;
        d_mode_ = heat_8c_restore_mode_;
        d_target_c_ = heat_8c_restore_target_c_;
        d_fan_ = heat_8c_restore_fan_;
        d_fan_turbo_ = heat_8c_restore_fan_turbo_;
        heat_8c_restore_valid_ = false;
        if (memory_mode_enabled_ && d_power_on_ && d_mode_ != climate::CLIMATE_MODE_OFF)
          last_active_mode_ = d_mode_;
        ESP_LOGD(TAG, "+8°C disabled: restored power=%s mode=%s target=%u°C fan=%s",
                 d_power_on_ ? "ON" : "OFF",
                 LOG_STR_ARG(climate::climate_mode_to_string(d_mode_)),
                 (unsigned) d_target_c_,
                 LOG_STR_ARG(climate::climate_fan_mode_to_string(d_fan_)));
      }

      d_quiet_ = false;
      // A Sleep OFF action must remain byte-17-only. Let the indoor unit
      // report the fan it restores after leaving Sleep, then synchronize to it.
      if (d_fan_ == climate::CLIMATE_FAN_QUIET && !was_eco && !was_sleep)
        d_fan_ = climate::CLIMATE_FAN_AUTO;
    }

    changed = true;
    }  // preset_supported
  }

  auto custom = call.get_custom_preset();
  if (!custom.empty()) {
    if (custom == CUSTOM_PRESET_HEAT_8C) {
      const bool heat8_supported = !capabilities_.valid || capabilities_.heat_8c || capabilities_.enable_8heat;
      if (!heat8_supported) {
        ESP_LOGW(TAG, "Ignoring unsupported +8°C preset (ProductType 8heat=0 enable_8heat=0)");
      } else {
      if (!d_heat_8c_) {
        heat_8c_restore_valid_ = true;
        heat_8c_restore_power_ = d_power_on_;
        heat_8c_restore_mode_ = d_mode_;
        heat_8c_restore_target_c_ = d_target_c_;
        heat_8c_restore_fan_ = d_fan_;
        heat_8c_restore_fan_turbo_ = d_fan_turbo_;
      }

      const bool was_off = !d_power_on_;
      d_power_on_ = true;
      d_mode_ = climate::CLIMATE_MODE_HEAT;
      d_heat_8c_ = true;
      d_turbo_ = false;
      d_eco_ = false;
      d_quiet_ = false;
      d_sleep_stage_ = 0;
      d_fan_turbo_ = false;
      d_fan_ = climate::CLIMATE_FAN_AUTO;
      sleep_restore_fan_valid_ = false;

      if (was_off && !d_led_) {
        d_led_ = true;
        led_command_pending_ = true;
      }
      if (memory_mode_enabled_) last_active_mode_ = climate::CLIMATE_MODE_HEAT;

      ESP_LOGD(TAG,
               "+8°C requested: HEAT frost protection, command byte[37]=0x03; normal target %u°C preserved",
               (unsigned) d_target_c_);
      changed = true;
      }  // heat8_supported
    } else if (custom == CUSTOM_PRESET_QUIET) {
      if (d_mode_ == climate::CLIMATE_MODE_AUTO) {
        ESP_LOGD(TAG, "Ignoring Quiet preset while SMART/AUTO is active");
      } else if (capabilities_.valid && !capabilities_.fan_mute) {
        ESP_LOGW(TAG, "Ignoring unsupported Quiet preset (ProductType fan_mute=0)");
      } else {
      sleep_restore_fan_valid_ = false;
      d_heat_8c_ = false;
      heat_8c_restore_valid_ = false;
      d_quiet_ = true;
      d_turbo_ = false;
      d_eco_ = false;
      d_sleep_stage_ = 0;
      d_fan_turbo_ = false;
      d_fan_ = climate::CLIMATE_FAN_QUIET;
      changed = true;
      }  // Quiet supported
    }
  }

  auto custom_fan = call.get_custom_fan_mode();
  if (!custom_fan.empty()) {
    if (custom_fan == CUSTOM_FAN_TURBO && d_mode_ == climate::CLIMATE_MODE_AUTO) {
      ESP_LOGD(TAG, "Ignoring custom Turbo fan while SMART/AUTO is active");
    } else if (custom_fan == CUSTOM_FAN_TURBO) {
      sleep_fan_override_pending_ = (sleep_stage_ > 0 || d_sleep_stage_ > 0);
      sleep_restore_fan_valid_ = false;
      d_heat_8c_ = false;
      heat_8c_restore_valid_ = false;
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

  uint16_t changed_fields = CMD_FIELD_NONE;
  if (d_power_on_ != before_power_on || d_mode_ != before_mode)
    changed_fields |= CMD_FIELD_POWER_MODE;
  if (d_target_c_ != before_target)
    changed_fields |= CMD_FIELD_TEMP;
  if ((d_fan_ != before_fan || d_fan_turbo_ != before_fan_turbo) &&
      d_mode_ != climate::CLIMATE_MODE_AUTO)
    changed_fields |= CMD_FIELD_WIND;
  if (d_swing_ != before_swing)
    changed_fields |= CMD_FIELD_SWING;
  if (d_turbo_ != before_turbo || d_eco_ != before_eco)
    changed_fields |= CMD_FIELD_TURBO_ECO;
  if (d_quiet_ != before_quiet)
    changed_fields |= CMD_FIELD_QUIET;
  if (d_heat_8c_ != before_heat_8c)
    changed_fields |= CMD_FIELD_HEAT_8C;
  if (d_led_ != before_led || led_command_pending_)
    changed_fields |= CMD_FIELD_LED;
  if (d_sleep_stage_ != before_sleep_stage)
    changed_fields |= CMD_FIELD_SLEEP;

  // A no-op ClimateCall does not need an empty write frame.
  if (changed_fields == CMD_FIELD_NONE) {
    ESP_LOGD(TAG, "Control: desired state unchanged; no UART write queued");
    return;
  }
  pending_command_fields_ |= changed_fields;

  // HA takes priority over remote changes
  accept_remote_changes_ = false;
  ha_priority_active_ = true;

  recalc_desired_sig_();

  // Publish optimistically
  this->mode = d_power_on_ ? d_mode_ : climate::CLIMATE_MODE_OFF;
  this->target_temperature = d_heat_8c_ ? 8 : d_target_c_;
  publish_fan_state_(d_fan_turbo_, d_fan_);
  this->swing_mode = d_swing_;
  if (enable_presets_) {
    if (d_heat_8c_ && (!capabilities_.valid || capabilities_.heat_8c || capabilities_.enable_8heat)) this->set_custom_preset_(CUSTOM_PRESET_HEAT_8C);
    else if (d_turbo_) this->set_preset_(climate::CLIMATE_PRESET_BOOST);
    else if (d_eco_ && (!capabilities_.valid || capabilities_.power_save)) this->set_preset_(climate::CLIMATE_PRESET_ECO);
    else if (sleep_stage_ > 0) this->set_preset_(climate::CLIMATE_PRESET_SLEEP);
    else if (d_quiet_ && (!capabilities_.valid || capabilities_.fan_mute)) this->set_custom_preset_(CUSTOM_PRESET_QUIET);
    else this->set_preset_(climate::CLIMATE_PRESET_NONE);
  }
  publish_state();
  update_led_switch_state_();

  // Mark that we have a pending command (debounced)
  pending_control_ = true;
  last_control_ms_ = millis();
  user_command_next_write_ = true;
  beep_on_next_write_ = command_sound_enabled_;

  ESP_LOGD(TAG, "Control: new desired state registered, command_sound=%s, will send after %lums debounce",
           command_sound_enabled_ ? "ON" : "OFF", (unsigned long) CONTROL_DEBOUNCE_MS);
}

// ---- Build a one-shot neutral TX frame ----
void ACHIClimate::build_tx_from_pending_fields_(uint16_t fields) {
  // Bytes 16..45 are command/action payload. Zero means "do not change".
  // Never copy the complete desired state into an ordinary write frame.
  std::fill(tx_bytes_.begin() + IDX_WIND, tx_bytes_.begin() + 46, 0x00);

  if (fields & CMD_FIELD_POWER_MODE) {
    const uint8_t power_bin = d_power_on_ ? 0b00001100 : 0b00000100;
    const uint8_t mode_hi = encode_mode_hi_nibble_(d_mode_);
    tx_bytes_[IDX_POWER_MODE] = power_bin + mode_hi;
  }

  if (fields & CMD_FIELD_TEMP)
    tx_bytes_[IDX_SET_TEMP] = encode_temp_(d_target_c_);

  if (fields & CMD_FIELD_WIND)
    tx_bytes_[IDX_WIND] = encode_fan_byte_(d_fan_, d_fan_turbo_);

  if (fields & CMD_FIELD_SLEEP)
    tx_bytes_[IDX_SLEEP] = encode_sleep_byte_(d_sleep_stage_);

  if (fields & CMD_FIELD_SWING) {
    const bool v = d_swing_ == climate::CLIMATE_SWING_VERTICAL ||
                   d_swing_ == climate::CLIMATE_SWING_BOTH;
    const bool h = d_swing_ == climate::CLIMATE_SWING_HORIZONTAL ||
                   d_swing_ == climate::CLIMATE_SWING_BOTH;
    tx_bytes_[IDX_TX_SWING] = (v ? TxValues::UPDOWN_ON : TxValues::UPDOWN_OFF) +
                              (h ? TxValues::LEFTRIGHT_ON : TxValues::LEFTRIGHT_OFF);
  }

  if (fields & CMD_FIELD_TURBO_ECO) {
    // Turbo and Eco are independent action commands in byte 33.
    // Do not combine TURBO_OFF (0x04) with Eco, because the confirmed
    // neutral one-shot Eco actions are exactly 0x30 (ON) and 0x10 (OFF).
    tx_bytes_[IDX_TX_TURBO_ECO] = d_turbo_
        ? TxValues::TURBO_ON
        : (d_eco_ ? TxValues::ECO_ON : TxValues::ECO_OFF);
  }

  if (fields & CMD_FIELD_QUIET)
    tx_bytes_[IDX_TX_QUIET] = d_quiet_ ? TxValues::QUIET_ON : TxValues::QUIET_OFF;

  if (fields & CMD_FIELD_HEAT_8C) {
    // t_8heat occupies bits 0..1 of command byte 37. Use OR so future
    // companion controls in the upper bits of this packed byte are preserved.
    tx_bytes_[IDX_TX_HEAT_8C] |= d_heat_8c_ ? TxValues::HEAT_8C_ON
                                            : TxValues::HEAT_8C_OFF;
  }

  // Display is also action-style. Send it when explicitly changed, or append
  // LED_OFF once to an ordinary user command while the desired display is off.
  // A confirmed Sleep program temporarily reports the panel as OFF even when it
  // was ON before Sleep. Never turn that temporary status into an LED_OFF action:
  // otherwise Sleep OFF first wakes the panel and the delayed LED_OFF turns it
  // dark again a few seconds later.
  const bool sleep_session =
      sleep_stage_ > 0 || d_sleep_stage_ > 0 || sleep_confirmation_pending_ ||
      (fields & CMD_FIELD_SLEEP);
  const bool sleep_temporarily_owns_led =
      sleep_session && (!sleep_restore_led_valid_ || sleep_restore_led_);
  const bool append_led_off =
      user_command_next_write_ && !d_led_ && !sleep_temporarily_owns_led;
  if ((fields & CMD_FIELD_LED) || append_led_off)
    tx_bytes_[IDX_TX_LED] = d_led_ ? TxValues::LED_ON : TxValues::LED_OFF;
}

// Queue only the fields that still differ after a status response. Sleep is
// deliberately excluded: Sleep Mode Code is authoritative and failed Sleep
// activation is handled by the confirmation/rollback logic without retries.
void ACHIClimate::queue_retry_fields_from_state_() {
  if (d_power_on_ != power_on_ || (d_power_on_ && d_mode_ != mode_))
    pending_command_fields_ |= CMD_FIELD_POWER_MODE;

  if (d_power_on_ && power_on_) {
    // A user temperature adjustment in SMART/AUTO is deliberately one-shot.
    // Do not retry it: after accepting the command, the indoor unit may change
    // the reported target again while switching between SMART branches 4/5/6/7.
    if (d_mode_ != climate::CLIMATE_MODE_AUTO && d_target_c_ != target_c_)
      pending_command_fields_ |= CMD_FIELD_TEMP;

    // A confirmed Sleep program owns the fan and normally forces QUIET. Do not
    // fight that automatic fan value unless the user explicitly selected a fan
    // mode in Home Assistant while Sleep was active.
    const bool sleep_owns_fan = sleep_stage_ > 0 && !sleep_fan_override_pending_;
    if (d_mode_ != climate::CLIMATE_MODE_AUTO && !sleep_owns_fan &&
        (d_fan_ != fan_ || d_fan_turbo_ != fan_turbo_))
      pending_command_fields_ |= CMD_FIELD_WIND;
    if (d_swing_ != swing_)
      pending_command_fields_ |= CMD_FIELD_SWING;
    if (d_turbo_ != turbo_ || d_eco_ != eco_)
      pending_command_fields_ |= CMD_FIELD_TURBO_ECO;
    if (d_quiet_ != quiet_)
      pending_command_fields_ |= CMD_FIELD_QUIET;
    if (d_heat_8c_ != heat_8c_)
      pending_command_fields_ |= CMD_FIELD_HEAT_8C;
  }
}

// ---- Send status query ----
void ACHIClimate::send_query_status_() {
  ESP_LOGD(TAG, "Sending status query (0x66/subtype 0x00)");
  send_logical_frame_(query_, "TX status query 0x66/0x00");
  status_query_in_flight_ = true;
  status_query_time_ = millis();
  query_kind_ = QUERY_STATUS;
}

void ACHIClimate::send_query_capabilities_() {
  std::vector<uint8_t> query = query_;
  if (query.size() <= 18) {
    ESP_LOGE(TAG, "Cannot build capabilities query: status template is too short");
    return;
  }

  // ProductType uses the normal 0x66 frame with subtype byte[14] changed to
  // 0x40. Recompute the running-sum checksum for this exact local template.
  query[14] = 0x40;
  calc_and_patch_crc_(query);

  capabilities_attempts_++;
  ESP_LOGI(TAG, "Sending device capabilities query 0x66/0x40 (attempt %u/%u)",
           (unsigned) capabilities_attempts_, (unsigned) CAPABILITIES_MAX_ATTEMPTS);
  send_logical_frame_(query, "TX capabilities query 0x66/0x40");
  status_query_in_flight_ = true;
  status_query_time_ = millis();
  query_kind_ = QUERY_CAPABILITIES;
#ifdef USE_TEXT_SENSOR
  publish_text_sensor_if_changed_(device_capabilities_text_, "Querying 0x66/0x40");
#endif
}

// ---- Send write command ----
void ACHIClimate::send_write_changes_() {
  const uint16_t fields = pending_command_fields_;
  if (fields == CMD_FIELD_NONE) {
    ESP_LOGV(TAG, "Pending write contained no command fields; skipped");
    return;
  }

  build_tx_from_pending_fields_(fields);

  // Every explicit Sleep action, including OFF, is confirmed only by the
  // next real Sleep Mode Code. Ordinary commands keep byte 17 equal to 0x00.
  sleep_confirmation_pending_ = (fields & CMD_FIELD_SLEEP) != 0;
  if (sleep_confirmation_pending_)
    sleep_confirmation_target_stage_ = d_sleep_stage_;
  tx_bytes_[IDX_TX_BEEP] = beep_on_next_write_ ? TxValues::BEEP_ON : TxValues::BEEP_OFF;
  calc_and_patch_crc_(tx_bytes_);
  ESP_LOGD(TAG,
           "Sending neutral one-shot write (0x65): fields=0x%03X wind[16]=0x%02X sleep[17]=0x%02X power_mode[18]=0x%02X temp[19]=0x%02X swing[32]=0x%02X features[33]=0x%02X quiet[35]=0x%02X led[36]=0x%02X heat8[37]=0x%02X beep[23]=0x%02X",
           (unsigned) fields, tx_bytes_[IDX_WIND], tx_bytes_[IDX_SLEEP],
           tx_bytes_[IDX_POWER_MODE], tx_bytes_[IDX_SET_TEMP],
           tx_bytes_[IDX_TX_SWING], tx_bytes_[IDX_TX_TURBO_ECO],
           tx_bytes_[IDX_TX_QUIET], tx_bytes_[IDX_TX_LED],
           tx_bytes_[IDX_TX_HEAT_8C], tx_bytes_[IDX_TX_BEEP]);
  send_logical_frame_(tx_bytes_, "TX one-shot write");

  last_tx_frame_.assign(tx_bytes_.begin(), tx_bytes_.end());
  pending_command_fields_ &= static_cast<uint16_t>(~fields);
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
    case climate::CLIMATE_MODE_AUTO:
      return KELON168_MODE_AUTO;
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
  // SMART branch 7 was observed during dehumidification. Preserve SMART in
  // Follow-Me/iFeel frames for every raw SMART branch (4..7); otherwise an
  // iFeel update built from a misclassified branch can switch the unit to a
  // normal COOL command and later provoke an unnecessary AUTO re-command.
  if (have_actual && this->raw_mode_code_ >= 0x04 && this->raw_mode_code_ <= 0x07)
    base_mode = climate::CLIMATE_MODE_AUTO;
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

// Hisense/Kelon UART byte stuffing: every logical 0xF4 between the header and
// footer is duplicated on the wire. The header F4 F5 and footer F4 FB remain
// unescaped. The declared length and CRC are calculated from the logical frame.
std::vector<uint8_t> ACHIClimate::encode_wire_frame_(const std::vector<uint8_t> &logical) const {
  if (logical.size() < 4 || logical[0] != HI_HDR0 || logical[1] != HI_HDR1 ||
      logical[logical.size() - 2] != HI_TAIL0 || logical.back() != HI_TAIL1) {
    ESP_LOGE(TAG, "Cannot byte-stuff malformed logical frame (%u bytes)",
             (unsigned) logical.size());
    return logical;
  }

  std::vector<uint8_t> wire;
  wire.reserve(std::min(MAX_WIRE_FRAME_BYTES, logical.size() * 2));
  wire.push_back(logical[0]);
  wire.push_back(logical[1]);

  for (size_t i = 2; i + 2 < logical.size(); i++) {
    wire.push_back(logical[i]);
    if (logical[i] == HI_HDR0) {
      wire.push_back(HI_HDR0);
    }
  }

  wire.push_back(HI_TAIL0);
  wire.push_back(HI_TAIL1);
  return wire;
}

void ACHIClimate::send_logical_frame_(const std::vector<uint8_t> &logical, const char *log_prefix) {
  const auto wire = encode_wire_frame_(logical);
  log_frame_(log_prefix, logical);
  if (wire.size() != logical.size()) {
    ESP_LOGD(TAG, "%s: byte-stuffed %u logical bytes to %u wire bytes", log_prefix,
             (unsigned) logical.size(), (unsigned) wire.size());
    log_frame_("TX wire", wire);
  }
  for (auto b : wire) write_byte(b);
  flush();
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
      if (frame.size() > IDX_CMD && frame[IDX_CMD] == 0x66) {
        // The requested response arrived but was corrupt. Do not hold a user
        // command for the full query timeout; the next poll will refresh state.
        if (query_kind_ == QUERY_CAPABILITIES && !capabilities_.valid) {
          capabilities_next_attempt_ms_ = millis() + CAPABILITIES_RETRY_MS;
        }
        status_query_in_flight_ = false;
        query_kind_ = QUERY_NONE;
      }
      continue;                     // drop invalid frame
    }

    handle_frame_(frame);
    handled++;
    last_status_crc_ = sum;
  }
}

bool ACHIClimate::extract_next_frame_(std::vector<uint8_t> &frame) {
  frame.clear();

  while (true) {
    if (rx_.size() <= rx_start_ + 1) return false;

    // Find an unescaped frame header F4 F5. Payload F4 values are doubled on
    // the wire. In the sequence F4 F4 F5, the second F4 belongs to an escaped
    // payload byte and must not be treated as a new header.
    size_t frame_start = rx_start_;
    bool found = false;
    for (; frame_start + 1 < rx_.size(); frame_start++) {
      const bool preceded_by_f4 = frame_start > 0 && rx_[frame_start - 1] == HI_HDR0;
      if (!preceded_by_f4 && rx_[frame_start] == HI_HDR0 && rx_[frame_start + 1] == HI_HDR1) {
        found = true;
        break;
      }
    }

    if (!found) {
      // Keep a possible first byte of a split header and discard older noise.
      if (!rx_.empty() && rx_.back() == HI_HDR0) {
        const uint8_t keep = rx_.back();
        rx_.clear();
        rx_.push_back(keep);
      } else {
        rx_.clear();
      }
      rx_start_ = 0;
      return false;
    }

    frame.clear();
    frame.reserve(MAX_FRAME_BYTES);
    frame.push_back(HI_HDR0);
    frame.push_back(HI_HDR1);

    size_t wire_pos = frame_start + 2;
    size_t expected_logical_size = 0;
    bool invalid = false;

    while (wire_pos < rx_.size()) {
      const uint8_t value = rx_[wire_pos];

      if (value == HI_HDR0) {
        if (wire_pos + 1 >= rx_.size()) {
          // A stuffed F4 pair or footer may be split across loop iterations.
          return false;
        }

        const uint8_t next = rx_[wire_pos + 1];
        if (next == HI_HDR0) {
          // F4 F4 on the wire represents one logical payload/CRC byte F4.
          frame.push_back(HI_HDR0);
          wire_pos += 2;
        } else if (next == HI_TAIL1) {
          // Unescaped F4 FB uniquely identifies the logical footer.
          frame.push_back(HI_TAIL0);
          frame.push_back(HI_TAIL1);
          wire_pos += 2;

          if (expected_logical_size != 0 && frame.size() == expected_logical_size) {
            rx_start_ = wire_pos;
            return true;
          }

          ESP_LOGW(TAG,
                   "Frame length mismatch after unstuffing: declared=%u logical=%u wire=%u", 
                   (unsigned) expected_logical_size, (unsigned) frame.size(),
                   (unsigned) (wire_pos - frame_start));
          invalid = true;
          break;
        } else {
          ESP_LOGW(TAG, "Invalid unescaped F4 0x%02X inside frame; resynchronizing", next);
          invalid = true;
          break;
        }
      } else {
        frame.push_back(value);
        wire_pos++;
      }

      if (frame.size() == 5) {
        expected_logical_size = static_cast<size_t>(frame[4]) + 9;
        if (expected_logical_size < 9 || expected_logical_size > MAX_FRAME_BYTES) {
          ESP_LOGW(TAG, "Invalid declared logical frame size %u; resynchronizing",
                   (unsigned) expected_logical_size);
          invalid = true;
          break;
        }
        ESP_LOGV(TAG, "RX frame declares %u logical bytes", (unsigned) expected_logical_size);
      }

      if (expected_logical_size != 0 && frame.size() >= expected_logical_size) {
        // A valid frame reaches its declared logical size only when the footer
        // branch above appends F4 FB. Reaching it here means the frame is corrupt.
        ESP_LOGW(TAG, "Declared frame length reached without footer; resynchronizing");
        invalid = true;
        break;
      }
    }

    if (!invalid) {
      // Header and a partial body are present; wait for more UART bytes.
      return false;
    }

    // Drop only the first byte of the bad candidate, then search for the next
    // header. This preserves any valid frame that follows immediately.
    rx_start_ = frame_start + 1;
    frame.clear();
  }
}

void ACHIClimate::handle_frame_(const std::vector<uint8_t> &b) {
  if (b.size() < 20) {
    ESP_LOGD(TAG, "Frame too short (%u), ignored", (unsigned) b.size());
    return;
  }
  uint8_t cmd = b[IDX_CMD];
  if (cmd == 102) {          // 0x66 – status or ProductType response
    status_query_in_flight_ = false;
    query_kind_ = QUERY_NONE;
    const uint8_t subtype = b.size() > 14 ? b[14] : 0x00;
    if (subtype == 0x40) {
      parse_capabilities_102_64_(b);
    } else {
      parse_status_102_(b);
    }
  } else if (cmd == 101) {   // 0x65 – write acknowledge
    handle_ack_101_();
  } else {
    ESP_LOGV(TAG, "Unknown command 0x%02X, ignored", cmd);
  }
}

void ACHIClimate::parse_capabilities_102_64_(const std::vector<uint8_t> &b) {
  if (b.size() <= 35 || b[IDX_CMD] != 0x66 || b[14] != 0x40) {
    ESP_LOGW(TAG, "Invalid/short capabilities response 0x66/0x40 (%u bytes)",
             (unsigned) b.size());
    capabilities_next_attempt_ms_ = millis() + CAPABILITIES_RETRY_MS;
    return;
  }

  ACHIDeviceCapabilities caps{};
  caps.cool_heat = (b[18] & 0x80) != 0;
  caps.power_save = (b[23] & 0x40) != 0;
  caps.purify = (b[23] & 0x08) != 0;
  caps.fan_mute = (b[24] & 0x40) != 0;
  caps.infinite_fan = (b[25] & 0x08) != 0;
  caps.heat_8c = (b[26] & 0x80) != 0;
  caps.swing_follow = (b[26] & 0x02) != 0;
  caps.power_display = static_cast<uint8_t>((b[27] >> 6) & 0x03);
  caps.ai = (b[28] & 0x40) != 0;
  caps.swing_dir_8 = (b[28] & 0x10) != 0;
  caps.humidity = (b[32] & 0x01) != 0;
  caps.demand_response = static_cast<uint8_t>(b[35] & 0x03);
  caps.ext_valid = b.size() > 39;
  if (caps.ext_valid) {
    caps.trans_102_64 = (b[38] & 0x08) != 0;
    caps.q_display = (b[39] & 0x40) != 0;
    caps.enable_8heat = (b[39] & 0x04) != 0;
  }
  caps.reply_len = static_cast<uint8_t>(std::min<size_t>(b.size(), 255));
  caps.valid = true;
  capabilities_ = caps;
  apply_capability_availability_();

  ESP_LOGI(TAG,
           "Capabilities 0x66/0x40 parsed (%uB): heat=%u eco=%u quiet=%u 8heat=%u "
           "humidity=%u purify=%u ai=%u infinite_fan=%u",
           (unsigned) caps.reply_len, caps.cool_heat, caps.power_save, caps.fan_mute,
           caps.heat_8c, caps.humidity, caps.purify, caps.ai, caps.infinite_fan);
  ESP_LOGI(TAG,
           "Capabilities: swing8=%u follow=%u display=%u demand=%u ext=%u "
           "q_display=%u enable_8heat=%u trans_102_64=%u",
           caps.swing_dir_8, caps.swing_follow, (unsigned) caps.power_display,
           (unsigned) caps.demand_response, caps.ext_valid, caps.q_display,
           caps.enable_8heat, caps.trans_102_64);

#ifdef USE_TEXT_SENSOR
  char summary[256];
  if (caps.ext_valid) {
    snprintf(summary, sizeof(summary),
             "heat=%u eco=%u quiet=%u 8C=%u humidity=%u purify=%u ai=%u "
             "fan_inf=%u swing8=%u follow=%u display=%u q_display=%u "
             "demand=%u enable8=%u profile199=%u len=%u",
             caps.cool_heat, caps.power_save, caps.fan_mute, caps.heat_8c,
             caps.humidity, caps.purify, caps.ai, caps.infinite_fan,
             caps.swing_dir_8, caps.swing_follow, (unsigned) caps.power_display,
             caps.q_display, (unsigned) caps.demand_response, caps.enable_8heat,
             caps.trans_102_64, (unsigned) caps.reply_len);
  } else {
    snprintf(summary, sizeof(summary),
             "heat=%u eco=%u quiet=%u 8C=%u humidity=%u purify=%u ai=%u "
             "fan_inf=%u swing8=%u follow=%u display=%u demand=%u ext=unknown len=%u",
             caps.cool_heat, caps.power_save, caps.fan_mute, caps.heat_8c,
             caps.humidity, caps.purify, caps.ai, caps.infinite_fan,
             caps.swing_dir_8, caps.swing_follow, (unsigned) caps.power_display,
             (unsigned) caps.demand_response, (unsigned) caps.reply_len);
  }
  publish_text_sensor_if_changed_(device_capabilities_text_, summary);
#endif
}

void ACHIClimate::apply_capability_availability_() {
  if (!capabilities_.valid) return;

  const bool quiet_supported = capabilities_.fan_mute;
  const bool heat8_supported = capabilities_.heat_8c || capabilities_.enable_8heat;

  if (enable_presets_) {
    // Replacing the custom-preset vector can invalidate the active pointer, so
    // clear it first and then restore only a supported actual preset.
    this->clear_custom_preset_();
    std::vector<const char *> custom_presets;
    if (quiet_supported) custom_presets.push_back(CUSTOM_PRESET_QUIET);
    if (heat8_supported) custom_presets.push_back(CUSTOM_PRESET_HEAT_8C);
    this->set_supported_custom_presets(custom_presets);

    if (heat_8c_ && heat8_supported) {
      this->set_custom_preset_(CUSTOM_PRESET_HEAT_8C);
    } else if (quiet_ && quiet_supported && sleep_stage_ == 0) {
      this->set_custom_preset_(CUSTOM_PRESET_QUIET);
    }
  }

#ifdef USE_SENSOR
  if (!capabilities_.humidity) {
    // Keep both entities registered, but make them unavailable in Home
    // Assistant instead of reporting the protocol marker 0x80 as 128%.
    if (indoor_humidity_setting_sensor_ != nullptr)
      indoor_humidity_setting_sensor_->publish_state(NAN);
    if (indoor_humidity_sensor_ != nullptr)
      indoor_humidity_sensor_->publish_state(NAN);
  }
#endif

  const bool preset_list_changed =
      !capabilities_.power_save || !quiet_supported || !heat8_supported;
#ifdef USE_API
  if (preset_list_changed) {
    capability_api_refresh_pending_ = true;
    capability_api_refresh_at_ms_ = millis() + 750;
  }
#endif

  publish_state();
}


void ACHIClimate::process_timer_event_(ACHITimerState &state, const char *name,
                                            uint8_t raw_hour, uint8_t raw_minute_status) {
  // The timer fields are bit-packed MSB first. This explains the verified
  // 1-hour sample 0x0B/0x01: 0x0B >> 3 = 1 hour, 0x01 >> 2 = 0 minutes,
  // and bit 0 = enabled.
  const uint8_t hours = static_cast<uint8_t>((raw_hour >> 3) & 0x1F);
  const uint8_t minutes = static_cast<uint8_t>((raw_minute_status >> 2) & 0x3F);
  const bool enabled = (raw_minute_status & 0x01) != 0;
  const uint16_t signature = static_cast<uint16_t>(raw_hour) << 8 | raw_minute_status;
  const bool frame_contains_event = raw_hour != 0 || raw_minute_status != 0;

  // Zeroes are ordinary status silence after the event was announced. Do not
  // clear a latched timer merely because subsequent polls contain 00/00.
  if (!frame_contains_event) return;

  const uint32_t now = millis();
  const uint16_t total_minutes = static_cast<uint16_t>(hours) * 60u + minutes;

  // An enabled marker with zero duration is treated as an explicit clear or an
  // already expired timer. It is safer than leaving a stale countdown active.
  if (!enabled || total_minutes == 0 || minutes > 59) {
    const bool changed = !state.known || state.active;
    state.known = true;
    state.active = false;
    state.initial_minutes = 0;
    state.last_published_remaining = 0xFFFF;
    state.last_event_signature = signature;
    state.last_event_ms = now;
    if (changed) ESP_LOGI(TAG, "%s timer disabled (raw=%02X/%02X)", name, raw_hour, raw_minute_status);
    return;
  }

  // The same event is normally repeated for about three status frames. Avoid
  // restarting the countdown for those repeats, while still allowing the user
  // to set the same duration again later.
  const bool repeated_announcement = state.known && state.active &&
      state.last_event_signature == signature &&
      now - state.last_event_ms <= TIMER_REPEAT_EVENT_WINDOW_MS;
  state.last_event_ms = now;
  if (repeated_announcement) return;

  state.known = true;
  state.active = true;
  state.initial_minutes = total_minutes;
  state.started_ms = now;
  state.last_published_remaining = 0xFFFF;
  state.last_event_signature = signature;
  ESP_LOGI(TAG, "%s timer captured: %02u:%02u remaining (raw=%02X/%02X)",
           name, hours, minutes, raw_hour, raw_minute_status);
}

void ACHIClimate::clear_timer_after_silence_(ACHITimerState &state, const char *name,
                                                   uint8_t raw_hour, uint8_t raw_minute_status) {
  // 00/00 is used both as ordinary status silence and after a physical-remote
  // cancellation, so a single zero frame cannot clear a timer. The indoor
  // unit refreshes an active timer periodically (observed about every 32 s).
  // If valid status frames continue but no non-zero refresh arrives for more
  // than 75 s, consider the timer cancelled/expired in the indoor unit.
  if (!state.known || !state.active) return;
  if (raw_hour != 0 || raw_minute_status != 0) return;
  if (state.last_event_ms == 0) return;

  const uint32_t silent_ms = millis() - state.last_event_ms;
  if (silent_ms <= TIMER_STATUS_REFRESH_TIMEOUT_MS) return;

  state.active = false;
  state.initial_minutes = 0;
  state.last_published_remaining = 0xFFFF;
  state.last_event_signature = 0;
  ESP_LOGI(TAG, "%s timer disabled after %.1fs without a status refresh (raw=00/00)",
           name, silent_ms / 1000.0f);
}

void ACHIClimate::publish_timer_state_(ACHITimerState &state, bool power_on_timer) {
  if (!state.known) return;

  uint16_t remaining = 0;
  if (state.active) {
    const uint32_t elapsed_minutes = (millis() - state.started_ms) / 60000u;
    remaining = elapsed_minutes >= state.initial_minutes
                    ? 0
                    : static_cast<uint16_t>(state.initial_minutes - elapsed_minutes);
    if (remaining == 0) {
      state.active = false;
      ESP_LOGI(TAG, "%s timer countdown expired locally",
               power_on_timer ? "Power-on" : "Power-off");
    }
  }

  if (state.last_published_remaining == remaining &&
      !(remaining == 0 && state.last_published_remaining == 0xFFFF)) {
    return;
  }
  state.last_published_remaining = remaining;

#ifdef USE_SENSOR
  sensor::Sensor *remaining_sensor = power_on_timer
      ? power_on_timer_remaining_sensor_
      : power_off_timer_remaining_sensor_;
  publish_sensor_if_changed_(remaining_sensor, static_cast<float>(remaining));
#endif
#ifdef USE_BINARY_SENSOR
  binary_sensor::BinarySensor *active_sensor = power_on_timer
      ? power_on_timer_active_binary_
      : power_off_timer_active_binary_;
  if (active_sensor != nullptr) active_sensor->publish_state(state.active);
#endif
#ifdef USE_TEXT_SENSOR
  text_sensor::TextSensor *text = power_on_timer ? power_on_timer_text_ : power_off_timer_text_;
  if (state.active) {
    char value[40];
    snprintf(value, sizeof(value), "%02u:%02u remaining",
             static_cast<unsigned>(remaining / 60), static_cast<unsigned>(remaining % 60));
    publish_text_sensor_if_changed_(text, value);
  } else {
    publish_text_sensor_if_changed_(text, "Disabled");
  }
#endif
}

void ACHIClimate::parse_timer_status_(const std::vector<uint8_t> &b) {
  if (b.size() <= IDX_OFF_TIMER_MINUTE_STATUS) return;

  const uint8_t on_hour = b[IDX_ON_TIMER_HOUR];
  const uint8_t on_minute_status = b[IDX_ON_TIMER_MINUTE_STATUS];
  const uint8_t off_hour = b[IDX_OFF_TIMER_HOUR];
  const uint8_t off_minute_status = b[IDX_OFF_TIMER_MINUTE_STATUS];

  process_timer_event_(power_on_timer_state_, "Power-on", on_hour, on_minute_status);
  process_timer_event_(power_off_timer_state_, "Power-off", off_hour, off_minute_status);

  clear_timer_after_silence_(power_on_timer_state_, "Power-on", on_hour, on_minute_status);
  clear_timer_after_silence_(power_off_timer_state_, "Power-off", off_hour, off_minute_status);

  publish_timer_state_(power_on_timer_state_, true);
  publish_timer_state_(power_off_timer_state_, false);
}

void ACHIClimate::update_timer_countdowns_() {
  publish_timer_state_(power_on_timer_state_, true);
  publish_timer_state_(power_off_timer_state_, false);
}

void ACHIClimate::publish_fault_state_(const std::vector<uint8_t> &b) {
  if (b.size() <= IDX_FAULT_PROTECT) {
    ESP_LOGV(TAG, "Status frame too short for complete fault map (%u bytes)",
             (unsigned) b.size());
    return;
  }

  const uint8_t raw_indoor = b[IDX_FAULT_INDOOR];
  const uint8_t raw_module = b[IDX_FAULT_MODULE];
  const uint8_t raw_outdoor = b[IDX_FAULT_OUTDOOR];

  // Byte 66 bit 7 is the active +8 C frost-protection mode flag, not a fault.
  const uint8_t raw_protect = static_cast<uint8_t>(b[IDX_FAULT_PROTECT] & 0x7F);
  const bool any = (raw_indoor | raw_module | raw_outdoor | raw_protect) != 0;
  const uint32_t signature = static_cast<uint32_t>(raw_indoor) |
                             (static_cast<uint32_t>(raw_module) << 8) |
                             (static_cast<uint32_t>(raw_outdoor) << 16) |
                             (static_cast<uint32_t>(raw_protect) << 24);

  std::string faults;
  bool truncated = false;
  auto append_fault = [&](const char *label) {
    if (truncated) return;
    const size_t extra = faults.empty() ? 0 : 2;
    const size_t label_len = std::char_traits<char>::length(label);
    if (faults.size() + extra + label_len > 240) {
      truncated = true;
      return;
    }
    if (!faults.empty()) faults += "; ";
    faults += label;
  };

  if (raw_indoor & 0x80) append_fault("Indoor temp sensor");
  if (raw_indoor & 0x40) append_fault("Indoor coil sensor");
  if (raw_indoor & 0x20) append_fault("Indoor humidity sensor");
  if (raw_indoor & 0x10) append_fault("Condensate tray full");
  if (raw_indoor & 0x08) append_fault("Indoor fan motor");
  if (raw_indoor & 0x04) append_fault("Grille/drive");
  if (raw_indoor & 0x02) append_fault("Zero-cross detector");
  if (raw_indoor & 0x01) append_fault("Indoor/outdoor communication");

  if (raw_module & 0x80) append_fault("Display");
  if (raw_module & 0x40) append_fault("Keypad");
  if (raw_module & 0x20) append_fault("Wi-Fi module");
  if (raw_module & 0x10) append_fault("Indoor electrical module");
  if (raw_module & 0x08) append_fault("Indoor EEPROM");

  if (raw_outdoor & 0x40) append_fault("Outdoor EEPROM");
  if (raw_outdoor & 0x20) append_fault("Outdoor coil sensor");
  if (raw_outdoor & 0x10) append_fault("Compressor discharge sensor");
  if (raw_outdoor & 0x08) append_fault("Outdoor temp sensor");

  if (raw_protect & 0x10) append_fault("Overheat/overcool protection");

  // Keep unnamed bits visible instead of incorrectly declaring the unit healthy.
  const uint8_t unknown_module = static_cast<uint8_t>(raw_module & 0x07);
  const uint8_t unknown_outdoor = static_cast<uint8_t>(raw_outdoor & 0x87);
  const uint8_t unknown_protect = static_cast<uint8_t>(raw_protect & 0x6F);
  char unknown[40];
  if (unknown_module != 0) {
    snprintf(unknown, sizeof(unknown), "Unknown module bits 0x%02X", unknown_module);
    append_fault(unknown);
  }
  if (unknown_outdoor != 0) {
    snprintf(unknown, sizeof(unknown), "Unknown outdoor bits 0x%02X", unknown_outdoor);
    append_fault(unknown);
  }
  if (unknown_protect != 0) {
    snprintf(unknown, sizeof(unknown), "Unknown protection bits 0x%02X", unknown_protect);
    append_fault(unknown);
  }
  if (truncated && faults.size() <= 235) faults += "; ...";
  if (!any) faults = "OK";
  else if (faults.empty()) {
    char raw[80];
    snprintf(raw, sizeof(raw), "Unknown fault: 39=%02X 40=%02X 64=%02X 66=%02X",
             raw_indoor, raw_module, raw_outdoor, raw_protect);
    faults = raw;
  }

#ifdef USE_BINARY_SENSOR
  if (ac_fault_binary_ != nullptr && (!fault_state_valid_ || last_fault_any_ != any))
    ac_fault_binary_->publish_state(any);
#endif
#ifdef USE_TEXT_SENSOR
  publish_text_sensor_if_changed_(ac_active_faults_text_, faults.c_str());
#endif

  if (!fault_state_valid_ || signature != last_fault_signature_) {
    if (any) {
      ESP_LOGW(TAG, "AC faults: %s (raw39=%02X raw40=%02X raw64=%02X raw66=%02X)",
               faults.c_str(), raw_indoor, raw_module, raw_outdoor, raw_protect);
    } else if (fault_state_valid_ && last_fault_any_) {
      ESP_LOGI(TAG, "AC faults cleared");
    } else {
      ESP_LOGD(TAG, "AC fault status: OK");
    }
  }

  fault_state_valid_ = true;
  last_fault_any_ = any;
  last_fault_signature_ = signature;
}

void ACHIClimate::parse_status_102_(const std::vector<uint8_t> &b) {
  last_status_frame_.assign(b.begin(), b.end());

  // Ensure we have all expected bytes
  if (b.size() < 49) {
    ESP_LOGE(TAG, "Status frame too short (%u), cannot parse fully", (unsigned) b.size());
    return;
  }

  parse_timer_status_(b);
  publish_fault_state_(b);

  // Power
  power_on_ = (b[IDX_POWER_MODE] & POWER_MASK) != 0;

  // Mode (upper nibble)
  uint8_t nib = (b[IDX_POWER_MODE] >> 4) & 0x0F;
  raw_mode_code_ = nib;
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

  // Sleep Mode Code is the only authoritative indication that Sleep is active.
  // This indoor unit reports the remote programs as 2, 4, 6 and 8.
  // Odd values are retained only as compatibility fallbacks for other revisions.
  uint8_t raw_sleep = b[IDX_SLEEP];
  switch (raw_sleep) {
    case 0x00: sleep_stage_ = 0; break;
    case 0x02: sleep_stage_ = 1; break;
    case 0x04: sleep_stage_ = 2; break;
    case 0x06: sleep_stage_ = 3; break;
    case 0x08: sleep_stage_ = 4; break;
    case 0x01: sleep_stage_ = 0; break;  // Sleep OFF action compatibility
    case 0x03: sleep_stage_ = 1; break;
    case 0x05: sleep_stage_ = 2; break;
    case 0x07: sleep_stage_ = 3; break;
    case 0x09: sleep_stage_ = 4; break;
    default:   sleep_stage_ = 0; break;
  }

  if (sleep_stage_ > 0 && selected_sleep_stage_ != sleep_stage_) {
    selected_sleep_stage_ = sleep_stage_;
    update_sleep_program_select_state_();
    ESP_LOGD(TAG, "Sleep Program synchronized from AC: %s (Sleep Mode Code=%u)",
             sleep_program_for_stage(selected_sleep_stage_), raw_sleep);
  }

  // Once Sleep was confirmed, a later status code 0 is authoritative too.
  // Clear the desired preset instead of trying to reconstruct it with ordinary
  // climate writes. A newly transmitted Sleep request is handled separately
  // below by sleep_confirmation_pending_.
  if (!sleep_confirmation_pending_ && d_sleep_stage_ > 0 && sleep_stage_ == 0) {
    d_sleep_stage_ = 0;
    pending_command_fields_ &= static_cast<uint16_t>(~CMD_FIELD_SLEEP);
    recalc_desired_sig_();
    ESP_LOGD(TAG, "Sleep Mode Code returned to 0; clearing desired HA Sleep preset");
  }

  // Target temperature (direct value). This must be read before +8 °C
  // detection because this indoor-unit firmware does not set either of the
  // documented frost-protection status bits. Instead, a remote-control +8 °C
  // command is reported as HEAT with the otherwise invalid normal setpoint 8.
  const uint8_t raw_target_c = b[IDX_SET_TEMP];

  // 8 °C frost-protection mode. Keep the documented status-bit checks for
  // compatible models, and additionally accept HEAT + raw target 8 for this
  // model. A normal Hisense setpoint is limited to 16–30 °C, so raw value 8 is
  // an unambiguous action-style +8 °C indication rather than an ordinary
  // target temperature.
  const bool heat_8c_primary = b.size() > IDX_RX_HEAT_8C &&
                               (b[IDX_RX_HEAT_8C] & 0x01) != 0;
  const bool heat_8c_companion = b.size() > IDX_RX_HEAT_8C_COMPANION &&
                                 (b[IDX_RX_HEAT_8C_COMPANION] & 0x80) != 0;
  const bool heat_8c_target_marker = power_on_ &&
                                     mode_ == climate::CLIMATE_MODE_HEAT &&
                                     raw_target_c == 8;
  heat_8c_ = heat_8c_primary || heat_8c_companion || heat_8c_target_marker;
  if (heat_8c_) mode_ = climate::CLIMATE_MODE_HEAT;
  if (heat_8c_) {
    // Units may report an out-of-normal-range internal frost setpoint (5 or
    // 8 °C). Keep the remembered normal HEAT target internally and publish the
    // logical +8 °C target through the preset instead of poisoning temperature
    // memory with that special controller value.
    target_c_ = target_for_mode_(climate::CLIMATE_MODE_HEAT, d_target_c_);
  } else {
    target_c_ = raw_target_c;
    if (target_c_ < 16 || target_c_ > 30) target_c_ = 24; // fallback
  }

  // Current temperatures
  current_temperature = b[IDX_CURRENT_TEMP];

#ifdef USE_SENSOR
  publish_sensor_if_changed_(pipe_sensor_, b[IDX_PIPE_TEMP]);
#endif

  // Turbo, Eco, Quiet, LED
  uint8_t features = b[IDX_RX_SWING_TURBO_ECO];   // byte 35 in status frame
  turbo_ = (features & TURBO_MASK) != 0;
  eco_   = (features & ECO_MASK) != 0;
  quiet_ = (b[IDX_RX_QUIET] & QUIET_MASK) != 0;   // byte 36 in status frame
  led_   = (b[IDX_RX_LED] & LED_MASK) != 0;       // byte 37 in status frame

  if (sleep_led_restore_pending_) {
    if (led_) {
      sleep_led_restore_pending_ = false;
      d_led_ = true;
      ESP_LOGI(TAG, "Pre-Sleep LED=ON restored and confirmed by status");
    } else if (millis() - sleep_led_restore_started_ms_ >= SLEEP_LED_RESTORE_TIMEOUT_MS) {
      // Normally the indoor unit restores the panel by itself after Sleep OFF.
      // If it does not, send one silent explicit LED_ON action as a fallback.
      sleep_led_restore_pending_ = false;
      d_led_ = true;
      led_command_pending_ = true;
      pending_command_fields_ |= CMD_FIELD_LED;
      pending_control_ = true;
      last_control_ms_ = millis();
      user_command_next_write_ = false;
      beep_on_next_write_ = false;
      accept_remote_changes_ = false;
      ha_priority_active_ = true;
      recalc_desired_sig_();
      ESP_LOGW(TAG, "Pre-Sleep LED=ON was not restored in time; queued silent LED_ON fallback");
    }
  }

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

  // During a confirmed Sleep program the indoor unit controls the fan and
  // normally reports QUIET. Keep the desired/UI fan synchronized with that
  // real value so unrelated HA commands (temperature, swing, display, etc.) do
  // not trigger repeated AUTO writes that eventually cancel Sleep.
  //
  // An explicit fan command made while Sleep is active is the only exception:
  // preserve the user's requested fan until the unit applies it (usually with
  // Sleep Mode Code returning to 0) or reports that exact fan value.
  if (sleep_stage_ > 0) {
    if (sleep_fan_override_pending_) {
      const bool requested_fan_applied =
          fan_ == d_fan_ && fan_turbo_ == d_fan_turbo_;
      if (requested_fan_applied) {
        sleep_fan_override_pending_ = false;
        ESP_LOGD(TAG, "Explicit fan command applied while Sleep remains active");
      }
    } else {
      const bool desired_fan_changed =
          d_fan_ != fan_ || d_fan_turbo_ != fan_turbo_ || d_quiet_;
      d_fan_ = fan_;
      d_fan_turbo_ = fan_turbo_;
      // Sleep is a preset, not the standalone Quiet custom preset.
      d_quiet_ = false;
      pending_command_fields_ &= static_cast<uint16_t>(~(CMD_FIELD_WIND | CMD_FIELD_QUIET));
      if (desired_fan_changed) {
        recalc_desired_sig_();
        ESP_LOGD(TAG, "Sleep controls fan: synchronized HA fan to actual %s",
                 LOG_STR_ARG(climate::climate_fan_mode_to_string(d_fan_)));
      }
    }
  } else if (sleep_fan_override_pending_) {
    // The usual result of a deliberate fan change is Sleep Mode Code = 0.
    // From this point normal desired/actual fan convergence applies.
    sleep_fan_override_pending_ = false;
    ESP_LOGD(TAG, "Sleep ended after explicit fan command; normal fan convergence resumed");
  }

  // Swing
  bool updown = (features & UPDOWN_MASK) != 0;
  bool leftright = (features & LEFTRIGHT_MASK) != 0;
  if (updown && leftright) swing_ = climate::CLIMATE_SWING_BOTH;
  else if (updown) swing_ = climate::CLIMATE_SWING_VERTICAL;
  else if (leftright) swing_ = climate::CLIMATE_SWING_HORIZONTAL;
  else swing_ = climate::CLIMATE_SWING_OFF;

  // The first status after a byte-17 Sleep action is authoritative. A command
  // is successful only when the reported program exactly matches the requested
  // program. Failed commands are not retried automatically.
  if (sleep_confirmation_pending_) {
    sleep_confirmation_pending_ = false;
    const uint8_t requested_stage = sleep_confirmation_target_stage_;
    sleep_confirmation_target_stage_ = 0;

    if (sleep_stage_ == requested_stage) {
      d_sleep_stage_ = requested_stage;
      if (requested_stage > 0) {
        selected_sleep_stage_ = requested_stage;
        update_sleep_program_select_state_();
        ESP_LOGI(TAG, "Sleep confirmed by status: requested=%s Sleep Mode Code=%u",
                 sleep_program_for_stage(requested_stage), raw_sleep);
      } else {
        // Restore the display preference that existed before Sleep. Sleep's
        // temporary LED=OFF status is not a user preference and must not survive
        // the transition back to Preset=None.
        const bool have_saved_led = sleep_restore_led_valid_;
        const bool saved_led = sleep_restore_led_;
        sleep_restore_led_valid_ = false;
        if (have_saved_led) {
          d_led_ = saved_led;
          pending_command_fields_ &= static_cast<uint16_t>(~CMD_FIELD_LED);
          led_command_pending_ = false;
          if (saved_led) {
            sleep_led_restore_pending_ = !led_;
            sleep_led_restore_started_ms_ = millis();
            ESP_LOGI(TAG,
                     "Sleep OFF confirmed: restoring pre-Sleep LED=ON; ignoring temporary LED_OFF status");
          } else {
            sleep_led_restore_pending_ = false;
            ESP_LOGI(TAG, "Sleep OFF confirmed: preserving pre-Sleep LED=OFF");
          }
        }

        // This indoor unit leaves the fan in QUIET after Sleep OFF. Restore the
        // fan that was active immediately before Sleep was enabled from HA.
        const bool have_saved_fan = sleep_restore_fan_valid_;
        const auto saved_fan = sleep_restore_fan_;
        const bool saved_fan_turbo = sleep_restore_fan_turbo_;
        const bool saved_quiet = sleep_restore_quiet_;
        sleep_restore_fan_valid_ = false;

        if (have_saved_fan &&
            (fan_ != saved_fan || fan_turbo_ != saved_fan_turbo || quiet_ != saved_quiet)) {
          d_fan_ = saved_fan;
          d_fan_turbo_ = saved_fan_turbo;
          d_quiet_ = saved_quiet;
          d_turbo_ = false;
          d_eco_ = false;

          pending_command_fields_ &= static_cast<uint16_t>(~CMD_FIELD_SLEEP);
          pending_command_fields_ |= CMD_FIELD_WIND | CMD_FIELD_QUIET;
          accept_remote_changes_ = false;
          ha_priority_active_ = true;
          pending_control_ = true;
          last_control_ms_ = millis();
          user_command_next_write_ = false;
          beep_on_next_write_ = false;
          recalc_desired_sig_();

          ESP_LOGI(TAG,
                   "Sleep OFF confirmed: restoring pre-Sleep fan=%s%s with a neutral one-shot command",
                   LOG_STR_ARG(climate::climate_fan_mode_to_string(d_fan_)),
                   d_fan_turbo_ ? " (Turbo)" : "");
        } else {
          d_fan_ = fan_;
          d_fan_turbo_ = fan_turbo_;
          d_quiet_ = quiet_;
          pending_command_fields_ &= static_cast<uint16_t>(~(CMD_FIELD_WIND | CMD_FIELD_QUIET));
          recalc_desired_sig_();
          ESP_LOGI(TAG,
                   "Sleep OFF confirmed by status: Sleep Mode Code=0; no pre-Sleep fan restore needed");
        }
      }
    } else if (requested_stage > 0 && sleep_stage_ == 0) {
      // Preserve the proven rollback behavior. With byte-17-only activation the
      // fan normally never changes on failure, so no rollback write is needed.
      d_sleep_stage_ = 0;

      if (sleep_restore_led_valid_) {
        d_led_ = sleep_restore_led_;
        sleep_restore_led_valid_ = false;
      }
      sleep_led_restore_pending_ = false;

      const bool have_fan_rollback = sleep_restore_fan_valid_;
      const auto rollback_fan = sleep_restore_fan_;
      const bool rollback_fan_turbo = sleep_restore_fan_turbo_;
      const bool rollback_quiet = sleep_restore_quiet_;
      sleep_restore_fan_valid_ = false;

      if (have_fan_rollback &&
          (fan_ != rollback_fan || fan_turbo_ != rollback_fan_turbo)) {
        d_fan_ = rollback_fan;
        d_fan_turbo_ = rollback_fan_turbo;
        d_quiet_ = rollback_quiet;
        d_turbo_ = false;
        d_eco_ = false;
        accept_remote_changes_ = false;
        ha_priority_active_ = true;
        recalc_desired_sig_();
        pending_command_fields_ |= CMD_FIELD_WIND | CMD_FIELD_QUIET | CMD_FIELD_TURBO_ECO;
        pending_control_ = true;
        last_control_ms_ = millis();
        user_command_next_write_ = false;
        beep_on_next_write_ = false;
        ESP_LOGW(TAG,
                 "Sleep %s not confirmed: Sleep Mode Code=0; restoring previous fan=%s%s",
                 sleep_program_for_stage(requested_stage),
                 LOG_STR_ARG(climate::climate_fan_mode_to_string(d_fan_)),
                 d_fan_turbo_ ? " (Turbo)" : "");
      } else {
        pending_control_ = false;
        pending_command_fields_ &= static_cast<uint16_t>(~CMD_FIELD_SLEEP);
        ha_priority_active_ = false;
        accept_remote_changes_ = true;
        recalc_desired_sig_();
        ESP_LOGW(TAG,
                 "Sleep %s not confirmed: Sleep Mode Code=0; HA Sleep preset cleared",
                 sleep_program_for_stage(requested_stage));
      }
    } else {
      // The AC stayed in another Sleep program (or rejected an OFF command).
      // Reflect the actual program and stop without fighting it.
      d_sleep_stage_ = sleep_stage_;
      // If Sleep is still active, retain the original pre-Sleep fan snapshot.
      // It remains valid for a later successful Sleep OFF action.
      pending_control_ = false;
      pending_command_fields_ &= static_cast<uint16_t>(~CMD_FIELD_SLEEP);
      ha_priority_active_ = false;
      accept_remote_changes_ = true;
      if (sleep_stage_ > 0) {
        selected_sleep_stage_ = sleep_stage_;
        update_sleep_program_select_state_();
      }
      recalc_desired_sig_();
      ESP_LOGW(TAG,
               "Sleep command not confirmed: requested=%s, actual=%s, Sleep Mode Code=%u",
               requested_stage > 0 ? sleep_program_for_stage(requested_stage) : "Off",
               sleep_stage_ > 0 ? sleep_program_for_stage(sleep_stage_) : "Off",
               raw_sleep);
    }
  }

  // Report the real HVAC action from the confirmed compressor feedback (byte 41).
  // The configured HVAC mode alone is not sufficient: an inverter unit can stay
  // powered in COOL/HEAT/DRY while the compressor is stopped.
  const bool compressor_running = b[IDX_COMP_FREQ_ACTUAL] > 0;
  if (!power_on_) {
    this->action = climate::CLIMATE_ACTION_OFF;
  } else if (mode_ == climate::CLIMATE_MODE_FAN_ONLY) {
    this->action = climate::CLIMATE_ACTION_FAN;
  } else if (!compressor_running) {
    this->action = climate::CLIMATE_ACTION_IDLE;
  } else if (mode_ == climate::CLIMATE_MODE_COOL) {
    this->action = climate::CLIMATE_ACTION_COOLING;
  } else if (mode_ == climate::CLIMATE_MODE_HEAT) {
    this->action = climate::CLIMATE_ACTION_HEATING;
  } else if (mode_ == climate::CLIMATE_MODE_DRY) {
    this->action = climate::CLIMATE_ACTION_DRYING;
  } else if (mode_ == climate::CLIMATE_MODE_AUTO) {
    // SMART status code describes the branch selected by the indoor unit.
    // Code 4 is the idle/fan branch (some revisions keep it while completing a
    // drying cycle), 5 is heat, 6 is cool and 7 is explicit dehumidification.
    // Compressor feedback remains authoritative for whether the branch is active.
    if (!compressor_running && raw_mode_code_ == 0x04)
      this->action = climate::CLIMATE_ACTION_FAN;
    else if (!compressor_running)
      this->action = climate::CLIMATE_ACTION_IDLE;
    else if (raw_mode_code_ == 0x05)
      this->action = climate::CLIMATE_ACTION_HEATING;
    else if (raw_mode_code_ == 0x06)
      this->action = climate::CLIMATE_ACTION_COOLING;
    else if (raw_mode_code_ == 0x07 || raw_mode_code_ == 0x04)
      this->action = climate::CLIMATE_ACTION_DRYING;
    else
      this->action = climate::CLIMATE_ACTION_IDLE;
  } else {
    this->action = climate::CLIMATE_ACTION_IDLE;
  }

  // Recalculate actual signature
  recalc_actual_sig_();

  // Some firmware revisions confirm a write with a status frame rather than a
  // dedicated 0x65 ACK. A matching status is therefore sufficient to release
  // the write lock without waiting five seconds for the fallback timeout.
  if (writing_lock_ && actual_sig_ == desired_sig_) {
    writing_lock_ = false;
    ESP_LOGD(TAG, "Write confirmed by matching status (lock cleared)");
  }

  // Publish state with gating
  publish_gated_state_();
  update_led_switch_state_();

  ESP_LOGV(TAG,
           "Extended status: compressor_actual=%uHz compressor_set=%uHz compressor_command=%uHz "
           "exhaust=%u°C raw_b22=0x%02X raw_b23=0x%02X raw_b47=0x%02X raw_b48=%u/%d",
           b[IDX_COMP_FREQ_ACTUAL], b[IDX_COMP_FREQ_SET], b[IDX_COMP_FREQ_COMMAND],
           b[IDX_COMPRESSOR_EXHAUST_TEMP], b[22], b[23], b[47],
           b[48], static_cast<int8_t>(b[48]));

  // Publish optional sensors (with sign conversion for outdoor temperatures)
#ifdef USE_SENSOR
  // The OFF status of this indoor unit reports 26°C as a service/default value.
  // Keep the user-facing setpoint sensor aligned with the remembered COOL/HEAT
  // target instead of exposing that transient value.
  const uint8_t published_setpoint = heat_8c_
      ? 8
      : (power_on_ ? b[IDX_SET_TEMP] : target_for_mode_(mode_, d_target_c_));
  publish_sensor_if_changed_(set_temp_sensor_, published_setpoint);
  publish_sensor_if_changed_(room_temp_sensor_, b[IDX_CURRENT_TEMP]);
  publish_sensor_if_changed_(wind_code_sensor_, b[IDX_WIND]);
  publish_sensor_if_changed_(sleep_code_sensor_, b[IDX_SLEEP]);
  publish_sensor_if_changed_(mode_code_sensor_, (b[IDX_POWER_MODE] >> 4) & 0x0F);
  publish_sensor_if_changed_(quiet_code_sensor_, quiet_ ? 1.0f : 0.0f);
  publish_sensor_if_changed_(turbo_code_sensor_, turbo_ ? 1.0f : 0.0f);
  publish_sensor_if_changed_(eco_code_sensor_, eco_ ? 1.0f : 0.0f);
  publish_sensor_if_changed_(swing_ud_sensor_, updown ? 1.0f : 0.0f);
  publish_sensor_if_changed_(swing_lr_sensor_, leftright ? 1.0f : 0.0f);
  publish_sensor_if_changed_(compressor_freq_actual_sensor_, b[IDX_COMP_FREQ_ACTUAL]);
  publish_sensor_if_changed_(compressor_freq_set_sensor_, b[IDX_COMP_FREQ_SET]);
  publish_sensor_if_changed_(compressor_freq_command_sensor_, b[IDX_COMP_FREQ_COMMAND]);
  // Backward-compatible byte-43 sensor for existing YAML configurations.
  publish_sensor_if_changed_(compressor_freq_sensor_, b[IDX_COMP_FREQ_COMMAND]);

  // Outdoor temperatures are signed!
  if (outdoor_temp_sensor_ != nullptr) {
    int8_t t = static_cast<int8_t>(b[IDX_OUTDOOR_TEMP]);
    publish_sensor_if_changed_(outdoor_temp_sensor_, static_cast<float>(t));
  }
  if (outdoor_cond_temp_sensor_ != nullptr) {
    int8_t t = static_cast<int8_t>(b[IDX_OUTDOOR_COND_TEMP]);
    publish_sensor_if_changed_(outdoor_cond_temp_sensor_, static_cast<float>(t));
  }
  publish_sensor_if_changed_(compressor_exhaust_temp_sensor_,
                             static_cast<float>(b[IDX_COMPRESSOR_EXHAUST_TEMP]));
  // Humidity entities always exist, but their availability follows the
  // ProductType reply. Before capabilities are known, leave them unavailable.
  if (capabilities_.valid && capabilities_.humidity) {
    const uint8_t humidity_setting = b[IDX_INDOOR_HUMIDITY_SETTING];
    const uint8_t humidity = b[IDX_INDOOR_HUMIDITY];
    if (humidity_setting <= 100)
      publish_sensor_if_changed_(indoor_humidity_setting_sensor_, static_cast<float>(humidity_setting));
    else if (indoor_humidity_setting_sensor_ != nullptr)
      indoor_humidity_setting_sensor_->publish_state(NAN);
    if (humidity <= 100)
      publish_sensor_if_changed_(indoor_humidity_sensor_, static_cast<float>(humidity));
    else if (indoor_humidity_sensor_ != nullptr)
      indoor_humidity_sensor_->publish_state(NAN);
  }

#endif

#ifdef USE_TEXT_SENSOR
  publish_text_sensor_if_changed_(power_status_text_, power_on_ ? "ON" : "OFF");
#endif

  ESP_LOGD(TAG,
           "Parsed: power=%s, mode=%s (raw_mode=%u), action=%s, fan=%s, swing=%s, target=%u°C, heat8=%s (status77=0x%02X status66=0x%02X raw_target=%u target_marker=%s), current=%.1f°C, outdoor=%d°C, "
           "compressor_actual=%uHz, compressor_set=%uHz, compressor_command=%uHz, exhaust=%u°C",
           power_on_ ? "ON" : "OFF",
           LOG_STR_ARG(climate::climate_mode_to_string(mode_)),
           (unsigned) raw_mode_code_,
           LOG_STR_ARG(climate::climate_action_to_string(this->action)),
           LOG_STR_ARG(climate::climate_fan_mode_to_string(fan_)),
           LOG_STR_ARG(climate::climate_swing_mode_to_string(swing_)),
           (unsigned) (heat_8c_ ? 8 : target_c_), heat_8c_ ? "ON" : "OFF",
           b.size() > IDX_RX_HEAT_8C ? b[IDX_RX_HEAT_8C] : 0,
           b.size() > IDX_RX_HEAT_8C_COMPANION ? b[IDX_RX_HEAT_8C_COMPANION] : 0,
           (unsigned) raw_target_c, heat_8c_target_marker ? "YES" : "NO", current_temperature,
           static_cast<int8_t>(b[IDX_OUTDOOR_TEMP]),
           b[IDX_COMP_FREQ_ACTUAL], b[IDX_COMP_FREQ_SET], b[IDX_COMP_FREQ_COMMAND],
           b[IDX_COMPRESSOR_EXHAUST_TEMP]);

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
    // Turbo/ECO/Quiet bits when the front display is off may be acknowledged
    // indirectly via target temperature and raw fan/wind code. Sleep is
    // deliberately excluded: only Sleep Mode Code > 0 may expose the Sleep
    // preset in Home Assistant. If a non-Sleep mode still matches, keep it
    // visible in HA instead of immediately publishing Preset=None on the next
    // status frame.
    bool out_turbo = turbo_;
    bool out_eco = eco_;
    bool out_quiet = quiet_;
    bool out_heat_8c = heat_8c_;
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

    // Learn normal COOL/HEAT setpoints only while the unit is powered on.
    // When this model is OFF it reports a service/default value of 26°C in
    // COOL, which must never overwrite the last real cooling setpoint.
    if (power_on_ && !out_turbo && !out_eco && !out_heat_8c && out_sleep_stage == 0) {
      remember_target_for_mode_(mode_, target_c_);
    }

    const uint8_t published_target = out_heat_8c
        ? 8
        : (power_on_ ? target_c_ : target_for_mode_(mode_, d_target_c_));

    this->mode = power_on_ ? mode_ : climate::CLIMATE_MODE_OFF;
    this->target_temperature = published_target;
    // In COOL the BOOST preset uses the physical Turbo airflow and should be
    // exposed as the custom Turbo fan mode. In HEAT this model keeps the fan
    // under automatic control even though the BOOST flag is active, so do not
    // misreport the fan as Turbo.
    const bool boost_uses_turbo_fan = out_turbo && mode_ != climate::CLIMATE_MODE_HEAT;
    publish_fan_state_(out_fan_turbo || boost_uses_turbo_fan, out_fan);
    this->swing_mode = swing_;
    if (enable_presets_) {
      if (out_heat_8c && (!capabilities_.valid || capabilities_.heat_8c || capabilities_.enable_8heat)) this->set_custom_preset_(CUSTOM_PRESET_HEAT_8C);
      else if (out_turbo) this->set_preset_(climate::CLIMATE_PRESET_BOOST);
      else if (out_eco && (!capabilities_.valid || capabilities_.power_save)) this->set_preset_(climate::CLIMATE_PRESET_ECO);
      else if (out_sleep_stage > 0) this->set_preset_(climate::CLIMATE_PRESET_SLEEP);
      else if (out_quiet && (!capabilities_.valid || capabilities_.fan_mute)) this->set_custom_preset_(CUSTOM_PRESET_QUIET);
      else this->set_preset_(climate::CLIMATE_PRESET_NONE);
    }
    // Sync desired with the effective published state, not only with raw flags.
    // This keeps indirect display-off presets stable while still allowing real
    // remote/HA changes to clear them when the indirect state no longer matches.
    d_power_on_    = power_on_;
    d_mode_        = mode_;
    // +8 °C is an action-style frost mode, not a normal setpoint. Preserve the
    // last normal HEAT target so Preset=None can return to it.
    if (!out_heat_8c) d_target_c_ = published_target;
    d_fan_         = out_fan;
    d_fan_turbo_   = out_fan_turbo;
    d_swing_       = swing_;
    d_eco_         = out_eco;
    d_turbo_       = out_turbo;
    d_quiet_       = out_quiet;
    d_heat_8c_     = out_heat_8c;
    if (!(sleep_led_restore_pending_ && !led_))
      d_led_ = led_;
    d_sleep_stage_ = out_sleep_stage;
    recalc_desired_sig_();
  } else {
    // Publish desired state while enforcing. A confirmed Sleep program is
    // authoritative for the fan unless the user explicitly requested another
    // fan mode; therefore show the real QUIET fan instead of stale desired AUTO.
    this->mode = d_power_on_ ? d_mode_ : climate::CLIMATE_MODE_OFF;
    this->target_temperature = d_heat_8c_ ? 8 : d_target_c_;
    if (d_turbo_) {
      // Show the expected fan state immediately while BOOST is being confirmed:
      // Turbo airflow in COOL, but AUTO airflow in HEAT.
      publish_fan_state_(d_mode_ != climate::CLIMATE_MODE_HEAT, d_fan_);
    } else if (sleep_stage_ > 0 && !sleep_fan_override_pending_) {
      publish_fan_state_(fan_turbo_, fan_);
    } else {
      publish_fan_state_(d_fan_turbo_, d_fan_);
    }
    this->swing_mode = d_swing_;
    if (enable_presets_) {
      if (d_heat_8c_ && (!capabilities_.valid || capabilities_.heat_8c || capabilities_.enable_8heat)) this->set_custom_preset_(CUSTOM_PRESET_HEAT_8C);
      else if (d_turbo_) this->set_preset_(climate::CLIMATE_PRESET_BOOST);
      else if (d_eco_ && (!capabilities_.valid || capabilities_.power_save)) this->set_preset_(climate::CLIMATE_PRESET_ECO);
      else if (sleep_stage_ > 0) this->set_preset_(climate::CLIMATE_PRESET_SLEEP);
      else if (d_quiet_ && (!capabilities_.valid || capabilities_.fan_mute)) this->set_custom_preset_(CUSTOM_PRESET_QUIET);
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

void ACHIClimate::update_sleep_program_select_state_() {
  if (sleep_program_select_ == nullptr) return;
  sleep_program_select_->publish_state(sleep_program_for_stage(selected_sleep_stage_));
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
    queue_retry_fields_from_state_();
    if (pending_command_fields_ == CMD_FIELD_NONE) {
      // The only remaining mismatch can be an action-style field such as
      // Sleep or display. Do not generate a full-state retry.
      ha_priority_active_ = false;
      accept_remote_changes_ = true;
      ESP_LOGD(TAG, "Desired/actual mismatch has no retryable one-shot fields; accepting status");
      return;
    }
    ESP_LOGD(TAG, "Enforcing desired state with neutral one-shot fields=0x%03X",
             (unsigned) pending_command_fields_);
    pending_control_ = true;
    last_control_ms_ = millis();   // restart debounce
  }
}

// ---- Signature computation ----
uint32_t ACHIClimate::compute_control_signature_(bool power, climate::ClimateMode mode,
                                                 climate::ClimateFanMode fan, bool fan_turbo,
                                                 climate::ClimateSwingMode swing,
                                                 bool eco, bool turbo, bool quiet, bool heat_8c, bool led,
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
    heat_8c = false;
    led = true;
    sleep_stage = 0;
    target_c = 24;
  }

  // SMART/AUTO owns subsequent target changes and the indoor fan. A target
  // adjustment from Home Assistant is sent once, then these values are normalized
  // so valid AUTO status (codes 4/5/6/7) cannot trigger an enforcement loop when
  // the indoor unit later changes its SMART branch or calculated temperature.
  if (power && mode == climate::CLIMATE_MODE_AUTO) {
    fan = climate::CLIMATE_FAN_AUTO;
    fan_turbo = false;
    target_c = 24;
  }

  // +8 °C is an action-style HEAT sub-mode. Its reported internal setpoint and
  // fan may differ by model, so normalize them while retaining the dedicated
  // status bit as the convergence key.
  if (heat_8c) {
    power = true;
    mode = climate::CLIMATE_MODE_HEAT;
    fan = climate::CLIMATE_FAN_AUTO;
    fan_turbo = false;
    eco = false;
    turbo = false;
    quiet = false;
    sleep_stage = 0;
    target_c = 8;
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
  mix(heat_8c ? 1u : 0u);
  mix(led ? 1u : 0u);
  mix(static_cast<uint32_t>(sleep_stage & 0x0Fu));
  mix(static_cast<uint32_t>(std::max<uint8_t>(16, std::min<uint8_t>(30, target_c))));
  return h;
}

void ACHIClimate::recalc_desired_sig_() {
  desired_sig_ = compute_control_signature_(d_power_on_, d_mode_, d_fan_, d_fan_turbo_, d_swing_,
                                            d_eco_, d_turbo_, d_quiet_, d_heat_8c_, d_led_,
                                            d_sleep_stage_, d_target_c_);
}

void ACHIClimate::recalc_actual_sig_() {
  bool effective_eco = eco_;
  bool effective_turbo = turbo_;
  bool effective_quiet = quiet_;
  bool effective_heat_8c = heat_8c_;
  uint8_t effective_sleep_stage = sleep_stage_;
  auto effective_fan = fan_;
  bool effective_fan_turbo = fan_turbo_;

  // A confirmed Sleep program owns the fan. Mask its automatic QUIET value
  // from signature comparison unless a user explicitly changed the fan while
  // Sleep was active. This prevents an unrelated command from creating an
  // endless AUTO-vs-QUIET mismatch and cancelling Sleep.
  if (sleep_stage_ > 0 && !sleep_fan_override_pending_) {
    effective_fan = d_fan_;
    effective_fan_turbo = d_fan_turbo_;
    effective_quiet = d_quiet_;
  }

  // Some Hisense indoor units acknowledge special modes only indirectly in
  // status frames. For HA-priority convergence, accept those indirect states
  // without changing the normal published UI state. This stops repeated silent
  // writes after BOOST/ECO while preserving the original preset display.
  // Sleep is never accepted indirectly; it requires Sleep Mode Code > 0.
  if (ha_priority_active_ && d_power_on_ && power_on_ && mode_ == d_mode_) {
    if (d_fan_turbo_ && target_c_ == d_target_c_ && last_raw_wind_ == 18) {
      effective_fan_turbo = true;
      effective_turbo = false;
      effective_eco = false;
      effective_quiet = false;
      effective_sleep_stage = 0;
      effective_heat_8c = false;
      effective_fan = d_fan_;
    } else if (d_turbo_ && target_c_ == d_target_c_) {
      effective_turbo = true;
      effective_eco = false;
      effective_quiet = false;
      effective_sleep_stage = 0;
      effective_heat_8c = false;
      effective_fan_turbo = false;
      effective_fan = d_fan_;
    } else if (d_eco_ && target_c_ == d_target_c_ && fan_ == climate::CLIMATE_FAN_QUIET) {
      effective_eco = true;
      effective_turbo = false;
      effective_quiet = false;
      effective_sleep_stage = 0;
      effective_heat_8c = false;
      effective_fan_turbo = false;
      effective_fan = d_fan_;

    } else if (d_quiet_ && fan_ == climate::CLIMATE_FAN_QUIET) {
      effective_quiet = true;
      effective_eco = false;
      effective_turbo = false;
      effective_sleep_stage = 0;
      effective_heat_8c = false;
      effective_fan_turbo = false;
      effective_fan = d_fan_;
    }
  }

  actual_sig_ = compute_control_signature_(power_on_, mode_, effective_fan, effective_fan_turbo, swing_,
                                           effective_eco, effective_turbo, effective_quiet,
                                           effective_heat_8c, led_, effective_sleep_stage, target_c_);
}

void ACHIClimate::log_sig_diff_() const {
  // Optional verbose diff – can be enabled for debugging
}

// ---- External LED control ----
void ACHIClimate::set_desired_led(bool on) {
  d_led_ = on;

  // If the user explicitly changes the display while Sleep is active, this is
  // the preference that must remain after Sleep ends.
  if (sleep_restore_led_valid_ &&
      (sleep_stage_ > 0 || d_sleep_stage_ > 0 || sleep_confirmation_pending_)) {
    sleep_restore_led_ = on;
  }
  sleep_led_restore_pending_ = false;

  // Dependency: turning the display OFF also turns command sound ON.
  // This keeps HA from showing an unsupported combination for this protocol.
  if (!on && !command_sound_enabled_) {
    command_sound_enabled_ = true;
    ESP_LOGD(TAG, "Command sound forced ON because display switch was turned OFF");
    update_sound_switch_state_();
  }

  led_command_pending_ = true;
  pending_command_fields_ |= CMD_FIELD_LED;
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
  // Fan values in a neutral one-shot 0x65 frame are action codes.
  // AUTO is the special case: its action value is 0x01, not the
  // status-derived value 0x02 that was accepted in the old full-state frame.
  if (turbo_fan)
    return 0x13;

  switch (f) {
    case climate::CLIMATE_FAN_AUTO:   return 0x01;
    case climate::CLIMATE_FAN_LOW:    return 0x0D;
    case climate::CLIMATE_FAN_MEDIUM: return 0x0F;
    case climate::CLIMATE_FAN_HIGH:   return 0x11;
    case climate::CLIMATE_FAN_QUIET:  return 0x0B;
    default:                          return 0x01;
  }
}

uint8_t ACHIClimate::encode_sleep_byte_(uint8_t stage) {
  // The indoor unit reports programs as status codes 2/4/6/8, while the
  // corresponding one-shot action values are 3/5/7/9. Stage 0 uses action 1.
  stage = std::min<uint8_t>(stage, 4);
  return static_cast<uint8_t>((stage << 1) | 0x01);
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

  publish_sensor_if_changed_(heap_free_sensor_, static_cast<float>(heap_free));
  if (heap_total > 0) publish_sensor_if_changed_(heap_total_sensor_, static_cast<float>(heap_total));
  if (heap_total > 0) publish_sensor_if_changed_(heap_used_sensor_, static_cast<float>(heap_used));
  if (heap_min_free > 0) publish_sensor_if_changed_(heap_min_free_sensor_, static_cast<float>(heap_min_free));
  if (heap_max_alloc > 0) publish_sensor_if_changed_(heap_max_alloc_sensor_, static_cast<float>(heap_max_alloc));
  if (heap_frag_pct >= 0) publish_sensor_if_changed_(heap_fragmentation_sensor_, static_cast<float>(heap_frag_pct));
  if (psram_total > 0) publish_sensor_if_changed_(psram_total_sensor_, static_cast<float>(psram_total));
  if (psram_free > 0) publish_sensor_if_changed_(psram_free_sensor_, static_cast<float>(psram_free));
#endif
}

}  // namespace ac_hi
}  // namespace esphome