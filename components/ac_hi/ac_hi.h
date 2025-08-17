#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace ac_hi {

// ========================= Protocol constants (legacy) =========================

// Frame envelope
static const uint8_t PFX0 = 0xF4;
static const uint8_t PFX1 = 0xF5;
static const uint8_t PFX2 = 0x00;
static const uint8_t PFX3 = 0x40;

static const uint8_t SUFFIX0 = 0xF4;
static const uint8_t SUFFIX1 = 0xFB;

// Common command codes
static const uint8_t CMD_STATUS = 0x66;   // query and response tag context
static const uint8_t CMD_WRITE  = 0x65;   // write/settings frame

// Indices used by our fixed-size legacy WRITE frame (50 bytes total)
enum : uint8_t {
  WR_IDX_HDR_0 = 0,             // F4
  WR_IDX_HDR_1 = 1,             // F5
  WR_IDX_HDR_2 = 2,             // 00
  WR_IDX_HDR_3 = 3,             // 40
  WR_IDX_LEN   = 4,             // 0x29
  WR_IDX_TAG0  = 13,            // 0x65 (CMD_WRITE)
  WR_IDX_TAG1  = 14,            // 0x00
  WR_IDX_TAG2  = 15,            // 0x00
  WR_IDX_FSET0 = 16,            // feature/mode set high
  WR_IDX_FSET1 = 17,            // feature/mode set low
  WR_IDX_MODEP = 18,            // b18: mode_hi | power_lo
  WR_IDX_SETPT = 19,            // b19: setpoint encoding
  // a lot of zeros / options-space ...
  WR_IDX_CRC_HI = 46,
  WR_IDX_CRC_LO = 47,
  WR_IDX_TAIL_0 = 48,           // 0xF4
  WR_IDX_TAIL_1 = 49            // 0xFB
};

// Indices used by received STATUS frames (we rely only on a few)
enum : uint8_t {
  ST_IDX_HDR_0 = 0,             // F4
  ST_IDX_HDR_1 = 1,             // F5
  ST_IDX_HDR_2 = 2,             // 00
  ST_IDX_HDR_3 = 3,             // 40
  ST_IDX_LEN   = 4,             // variable (depends on model)
  ST_IDX_TAG0  = 13,            // 0x66 or 0x65 (некоторые блоки отвечают 0x65)
  // payload offsets we read (по логам):
  ST_IDX_B18   = 18,            // b18: mode_hi|power_lo
  ST_IDX_B19   = 19,            // b19: setpoint
  // в логах ещё фигурировали b20/b21 и пр., но нам для логики климата достаточно b18/b19
};

// Mode encoding high nibble in b18
static const uint8_t B18_MODE_HI_MASK = 0xF0;
static const uint8_t B18_POWER_LO_MASK = 0x0F;

// Known power low-nibble spellings in the field
static const uint8_t POWER_LO_OFF = 0x00;
static const uint8_t POWER_LO_ON_0C = 0x0C;  // «классический» вариант
static const uint8_t POWER_LO_ON_08 = 0x08;  // встречается у ряда блоков (как у тебя)

// High nibble values for modes (по наблюдениям из логов и совместимости)
static const uint8_t MODE_HI_FAN  = 0x00;
static const uint8_t MODE_HI_HEAT = 0x10;
static const uint8_t MODE_HI_COOL = 0x20;
static const uint8_t MODE_HI_DRY  = 0x30;

// ========================= Class =========================

class ACHIClimate : public climate::Climate, public PollingComponent, public uart::UARTDevice {
 public:
  // Конструктор с периодом опроса 1 секунда
  ACHIClimate() : PollingComponent(1000) {}

  // user config
  void set_pipe_sensor(sensor::Sensor *s) { this->pipe_sensor_ = s; }

  // Component
  void setup() override;
  void loop() override {}
  void update() override;

  // Climate
  climate::ClimateTraits traits() override;
  void control(const climate::ClimateCall &call) override;

 protected:
  // ========================= Protocol helpers =========================
  // Build
  void build_legacy_write_(std::vector<uint8_t> &frame);
  void build_status_query_(std::vector<uint8_t> &frame);

  // Send/receive
  void send_frame_(const std::vector<uint8_t> &frame);
  bool read_frame_(std::vector<uint8_t> &frame, uint32_t timeout_ms);
  bool verify_and_classify_(const std::vector<uint8_t> &frame, uint8_t &cmd);

  // Parse
  void parse_status_legacy_(const std::vector<uint8_t> &frame);

  // Encoding helpers
  uint8_t encode_mode_hi_write_legacy_(climate::ClimateMode mode) const;
  uint8_t clamp16_30_(int v) const { if (v < 16) return 16; if (v > 30) return 30; return (uint8_t) v; }

  // CRC helpers (SUM16: hi then lo)
  uint16_t sum16_(const uint8_t *data, size_t len) const;

  // ========================= State =========================
  // «подсказка» от последнего STATUS: какой low-nibble «нравится» блоку, 0x08 или 0x0C
  uint8_t power_lo_hint_ {POWER_LO_ON_0C};

  // формат уставки в STATUS (если true → b19 уже «прямые °C»; если false → (2*C+1)>>1)
  bool status_temp_even_ {false};

  // последнее, что отправляли (для implicit-ACK)
  uint8_t pending_b18_ {0};
  uint8_t pending_b19_ {0};
  bool waiting_ack_ {false};
  uint32_t last_tx_ms_ {0};

  // сенсоры
  sensor::Sensor *pipe_sensor_ {nullptr};

  // кеш по STATUS
  float last_pipe_c_ {NAN};
};

}  // namespace ac_hi
}  // namespace esphome
