#pragma once

#include "esphome/components/climate/climate.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"  // esphome::millis()

// Подключаем заголовок сенсоров только если платформа sensor реально присутствует в билде
#ifdef USE_SENSOR
  #include "esphome/components/sensor/sensor.h"
#endif

#include <vector>
#include <cstddef>
#include <cstdint>

namespace esphome {
namespace sensor {
class Sensor;  // форвард, если USE_SENSOR не определён
}  // namespace sensor
}  // namespace esphome

namespace esphome {
namespace ac_hi {

// Кадры Hisense:
// Header: 0xF4 0xF5
// Tail  : 0xF4 0xFB
// bytes[4] — «декларированная длина», полный размер кадра = bytes[4] + 9
static constexpr uint8_t HI_HDR0 = 0xF4;
static constexpr uint8_t HI_HDR1 = 0xF5;
static constexpr uint8_t HI_TAIL0 = 0xF4;
static constexpr uint8_t HI_TAIL1 = 0xFB;

// Ограничители, чтобы loop() не блокировал цикл приложения
static constexpr uint8_t  MAX_FRAMES_PER_LOOP = 2;    // не более 2 кадров за один проход loop()
static constexpr uint32_t MAX_PARSE_TIME_MS   = 20;   // и не более 20 мс парсинга за проход
static constexpr size_t   RX_COMPACT_THRESHOLD = 512; // после потребления >512 байт — compaction

class ACHIClimate : public climate::Climate, public PollingComponent, public uart::UARTDevice {
 public:
  ACHIClimate() = default;

  // Config
  void set_enable_presets(bool v) { enable_presets_ = v; }
#ifdef USE_SENSOR
  void set_pipe_sensor(sensor::Sensor *s) { pipe_sensor_ = s; }
#else
  void set_pipe_sensor(void *) {}
#endif

  void setup() override;
  void loop() override;
  void update() override;

  // Climate API
  void control(const climate::ClimateCall &call) override;
  climate::ClimateTraits traits() override;

 protected:
  // ---- Протокол/транспорт ----
  void send_query_status_();    // короткий запрос состояния (cmd 0x66)
  void send_write_changes_();   // полная посылка состояния с CRC
  void calc_and_patch_crc_(std::vector<uint8_t> &buf);
  bool validate_crc_(const std::vector<uint8_t> &buf, uint16_t *out_sum = nullptr) const;

  // RX фреймер/парсер
  void try_parse_frames_from_buffer_(uint32_t budget_ms = MAX_PARSE_TIME_MS); // сканер потока с бюджетом
  bool extract_next_frame_(std::vector<uint8_t> &frame);                       // достаёт [F4 F5 ... F4 FB]
  void handle_frame_(const std::vector<uint8_t> &frame);
  void parse_status_102_(const std::vector<uint8_t> &b);
  void handle_ack_101_();

  // Буфер входящего потока (скользящее окно: rx_start_ — смещение начала данных)
  std::vector<uint8_t> rx_;
  size_t rx_start_{0};

  bool writing_lock_{false};
  bool pending_write_{false};

  // Базовый кадр записи (как в YAML initial vector)
  std::vector<uint8_t> tx_bytes_ = {
      0xF4, 0xF5, 0x00, 0x40, 0x29, 0x00, 0x00, 0x01, 0x01, 0xFE, 0x01, 0x00, 0x00,
      0x65, 0x00, 0x00, 0x00, // 0..16
      0x00, // [17] sleep
      0x00, // [18] power+mode
      0x00, // [19] set temp (°C, напрямую)
      0x00, // [20] current temp (RO)
      0x00, // [21] pipe temp (RO)
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 22..29
      0x00, 0x00, // 30..31
      0x00, // [32] swing UD/LR
      0x00, // [33] turbo/eco (tx)
      0x00, // [34]
      0x00, // [35] quiet
      0x00, // [36] LED + misc
      0x00, // [37]
      0x00, // [38]
      0x00, 0x00, 0x00, 0x00, 0x00, // 39..43
      0x00, 0x00, // 44..45
      0x00, 0x00, // 46..47 CRC (будут пропатчены)
      0xF4, 0xFB   // tail
  };

  // Короткий запрос статуса (cmd 0x66) — CRC уже «правильный» для этого шаблона
  const std::vector<uint8_t> query_ = {
      0xF4, 0xF5, 0x00, 0x40, 0x0C, 0x00, 0x00, 0x01, 0x01,
      0xFE, 0x01, 0x00, 0x00, 0x66, 0x00, 0x00, 0x00, 0x01,
      0xB3, 0xF4, 0xFB
  };

  // ---- Отражение состояния ----
  bool power_on_{false};
  uint8_t target_c_{24}; // 16..30
  climate::ClimateMode mode_{climate::CLIMATE_MODE_OFF};
  climate::ClimateFanMode fan_{climate::CLIMATE_FAN_AUTO};
  climate::ClimateSwingMode swing_{climate::CLIMATE_SWING_OFF};
  bool turbo_{false};
  bool eco_{false};
  bool quiet_{false};
  bool led_{true};
  uint8_t sleep_stage_{0}; // 0..4

  // Для подавления повторов статуса (по сумме байтов)
  uint16_t last_status_crc_{0};

  // ---- Кодирование полей ----
  uint8_t encode_temp_(uint8_t c) {
    return static_cast<uint8_t>(std::max<uint8_t>(16, std::min<uint8_t>(30, c)));
  }
  uint8_t encode_mode_hi_nibble_(climate::ClimateMode m);
  uint8_t encode_fan_byte_(climate::ClimateFanMode f);
  uint8_t encode_sleep_byte_(uint8_t stage);
  uint8_t encode_swing_ud_(bool on);
  uint8_t encode_swing_lr_(bool on);

  // Сенсор трубки (опционально)
#ifdef USE_SENSOR
  sensor::Sensor *pipe_sensor_{nullptr};
#else
  void *pipe_sensor_{nullptr};
#endif

  // Флаги
  bool enable_presets_{true};
};

}  // namespace ac_hi
}  // namespace esphome