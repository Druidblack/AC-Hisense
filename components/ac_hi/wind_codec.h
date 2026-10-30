#pragma once

#include <cstdint>

namespace esphome {
namespace ac_hi {

// The wind speed is independent of the Quiet preset flag (0x66 byte 36 bit 2).
// A QUIET speed is status wind=10, while the separate Quiet preset is reported
// with wind=2 and the Quiet Mode flag set. Do not combine these two fields.
enum class WindSpeed : uint8_t { AUTO, QUIET, LOW, MEDIUM, HIGH, TURBO };

constexpr WindSpeed decode_wind_code(uint8_t raw_wind) {
  switch (raw_wind) {
    case 10: return WindSpeed::QUIET;
    case 12: return WindSpeed::LOW;
    case 14: return WindSpeed::MEDIUM;
    case 16: return WindSpeed::HIGH;
    case 18: return WindSpeed::TURBO;
    default: return WindSpeed::AUTO;  // 0/1/2 and unknown values
  }
}

// The one-shot 0x65 TX wind byte is a command code, not the 0x66 status code.
constexpr uint8_t encode_wind_action(WindSpeed speed) {
  switch (speed) {
    case WindSpeed::QUIET:  return 0x0B;  // status 10
    case WindSpeed::LOW:    return 0x0D;  // status 12
    case WindSpeed::MEDIUM: return 0x0F;  // status 14
    case WindSpeed::HIGH:   return 0x11;  // status 16
    case WindSpeed::TURBO:  return 0x13;  // status 18
    default:                return 0x01;  // status 0/1/2
  }
}

}  // namespace ac_hi
}  // namespace esphome
