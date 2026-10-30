#pragma once

namespace esphome {
namespace ac_hi {

// An LED-off indoor unit must always have command confirmation sound enabled.
// Sound OFF is allowed only while the desired display is ON.
constexpr bool led_sound_safe_command_sound(bool led_on, bool requested_sound_on) {
  return !led_on || requested_sound_on;
}

}  // namespace ac_hi
}  // namespace esphome
