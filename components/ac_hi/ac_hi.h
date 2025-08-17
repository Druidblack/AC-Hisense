#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/sensor/sensor.h"
#include <vector>

namespace esphome {
namespace ac_hi {

// ========================= Protocol constants (legacy) =========================

static const uint8_t PFX0 = 0xF4;
static const uint8_t PFX1 = 0xF5;
static const uint8_t PFX2 = 0x00;
static const uint8_t PFX3 = 0x40;

static const uint8_t SUFFIX0 = 0xF4;
static const uint8_t SUFFIX1 = 0xFB;

static const uint8_t CMD_STATUS = 0x66;
static const uint8_t CMD_WRITE  = 0x65;

enum : uint8_t {
  WR_IDX_HDR_0 = 0,
  WR_IDX_HDR_1 = 1,
  WR_IDX_HDR_2 = 2,
  WR_IDX_HDR_3 = 3,
  WR_IDX_LEN   = 4,
  WR_IDX_TAG0  = 13,
  WR_IDX_TAG1  = 14,
  WR_IDX_TAG2  = 15,
  WR_IDX_FSET0 = 16,
  WR_IDX_FSET1 = 17,
  WR_IDX_MODEP = 18,
  WR_IDX_SETPT = 19,
  WR_IDX_CRC_HI = 46,
  WR_IDX_CRC_LO = 47,
  WR_IDX_TAIL_0 = 48,
  WR_IDX_TAIL_1 = 49
};

enum : uint8_t {
  ST_IDX_HDR_0 = 0,
  ST_IDX_HDR_1 = 1,
  ST_IDX_HDR_2 = 2,
  ST_IDX_HDR_3 = 3,
  ST_IDX_LEN   = 4,
  ST_IDX_TAG0  = 13,
  ST_IDX_B18   = 18,
  ST_IDX_B19   = 19,
};

static const uint8_t B18_MODE_HI_MASK  = 0xF0;
static const uint8_t B18_POWER_LO_MASK = 0x0F;

static const uint8_t POWER_LO_OFF   = 0x00;
static const uint8_t POWER_LO_ON_0C = 0x0C;
static const uint8_t POWER_LO_ON_08 = 0x08;

static const uint8_t MODE_HI_FAN  = 0x00;
static const uint8_t MODE_HI_HEAT = 0x10;
static const uint8_t MODE_HI_COOL = 0x20;
static const uint8_t MODE_HI_DRY  = 0x30;

class ACHIClimate : public climate::Climate, public PollingComponent, public uart::UARTDevice {
 public:
  ACHIClimate() : PollingComponent(1000) {}

  void set_pipe_sensor(sensor::Sensor *s) { this->pipe_sensor_ = s; }

  // совместимость с твоим main.cpp
  void set_enable_presets(bool) {}  // no-op

  // Component
  void setup() override;
  void loop() override {}
  void update() override;

  // Climate
  climate::ClimateTraits traits() override;
  void control(const climate::ClimateCall &call) override;

 protected:
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
  uint8_t power_lo_hint_ {POWER_LO_ON_0C};  // 0x08/0x0C в статусе
  bool status_temp_even_ {false};           // формат b19 в STATUS

  // implicit-ACK ожидания
  uint8_t pending_b18_ {0};
  uint8_t pending_b19_ {0};
  bool waiting_ack_ {false};
  uint32_t last_tx_ms_ {0};

  // сенсоры
  sensor::Sensor *pipe_sensor_ {nullptr};
  float last_pipe_c_ {NAN};
};

}  // namespace ac_hi
}  // namespace esphome
