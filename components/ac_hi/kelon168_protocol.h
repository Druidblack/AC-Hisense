#pragma once

#include <array>
#include <cstdint>

#include "esphome/components/remote_base/remote_base.h"
#include "esphome/core/optional.h"

// Kelon168 protocol support adapted for ac_hi iFeel IR transmitter

namespace esphome {
namespace ac_hi {

static const uint8_t KELON168_STATE_LENGTH = 21;

enum Kelon168Command : uint8_t {
  KELON168_COMMAND_LIGHT = 0x00,
  KELON168_COMMAND_POWER = 0x01,
  KELON168_COMMAND_TEMP = 0x02,
  KELON168_COMMAND_SLEEP = 0x03,
  KELON168_COMMAND_SUPER = 0x04,
  KELON168_COMMAND_ON_TIMER = 0x05,
  KELON168_COMMAND_MODE = 0x06,
  KELON168_COMMAND_SWING = 0x07,
  KELON168_COMMAND_HORIZONTAL_SWING = 0x08,
  KELON168_COMMAND_IFEEL = 0x0D,
  KELON168_COMMAND_FAN_SPEED = 0x11,
  KELON168_COMMAND_OFF_TIMER = 0x1D,
};

enum Kelon168Mode : uint8_t {
  KELON168_MODE_HEAT = 0,
  KELON168_MODE_AUTO = 1,
  KELON168_MODE_COOL = 2,
  KELON168_MODE_DRY = 3,
  KELON168_MODE_FAN = 4,
};

enum Kelon168Fan : uint8_t {
  KELON168_FAN_AUTO = 0,
  KELON168_FAN_MIN = 1,
  KELON168_FAN_LOW = 2,
  KELON168_FAN_MEDIUM = 3,
  KELON168_FAN_HIGH = 4,
  KELON168_FAN_MAX = 5,
};

struct Kelon168Data {
  std::array<uint8_t, KELON168_STATE_LENGTH> state{};

  bool operator==(const Kelon168Data &rhs) const { return this->state == rhs.state; }
  uint8_t command() const { return this->state[15]; }
};

class Kelon168Protocol : public remote_base::RemoteProtocol<Kelon168Data> {
 public:
  void encode(remote_base::RemoteTransmitData *dst, const Kelon168Data &data);
  optional<Kelon168Data> decode(remote_base::RemoteReceiveData src);
  void dump(const Kelon168Data &data);

  static Kelon168Data make_default();
  static bool valid_checksum(const Kelon168Data &data);
  static void checksum(Kelon168Data *data);
  static uint8_t reverse_bits(uint8_t value);

 protected:
  void encode_section_(remote_base::RemoteTransmitData *dst, const uint8_t *data, uint8_t size, bool header,
                       uint32_t footer_space);
  bool decode_section_(remote_base::RemoteReceiveData *src, uint8_t *data, uint8_t size, bool header,
                       uint32_t footer_space);
};

}  // namespace ac_hi
}  // namespace esphome
