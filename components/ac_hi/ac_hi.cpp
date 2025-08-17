#include "ac_hi.h"
#include "esphome/core/log.h"
#include <vector>

namespace esphome {
namespace ac_hi {

static const char *const TAG = "ac_hi";

// ========================= Component =========================

void ACHIClimate::setup() {
  ESP_LOGI(TAG, "setup(): legacy Hisense climate on UART");
  // nothing special
}

climate::ClimateTraits ACHIClimate::traits() {
  climate::ClimateTraits t{};
  t.set_supports_current_temperature(true);
  t.set_supports_two_point_target_temperature(false);
  t.set_supports_heat_mode(true);
  t.set_supports_cool_mode(true);
  t.set_supports_dry_mode(true);
  t.set_supports_fan_only_mode(true);
  t.set_visual_min_temperature(16);
  t.set_visual_max_temperature(30);
  t.set_supported_custom_fan_modes({});
  t.set_supported_swing_modes({climate::CLIMATE_SWING_OFF});  // для простоты; можно расширить при необходимости
  return t;
}

void ACHIClimate::update() {
  // Периодический опрос статуса: короткий кадр CMD_STATUS
  std::vector<uint8_t> q;
  this->build_status_query_(q);
  ESP_LOGV(TAG, "poll: status query sent (len=%zu): %s", q.size(), hexencode(q).c_str());
  this->send_frame_(q);

  // Пробуем принять что-то (обычно 0x65 или 0x66 длинный ~130 байт)
  std::vector<uint8_t> rx;
  if (this->read_frame_(rx, 20)) {
    uint8_t cmd = 0;
    if (this->verify_and_classify_(rx, cmd)) {
      if (cmd == CMD_STATUS || cmd == CMD_WRITE) {
        if (cmd == CMD_STATUS) {
          ESP_LOGV(TAG, "RX frame: cmd=0x%02X len=%zu → treating as STATUS", cmd, rx.size());
        } else {
          // многие блоки отвечают 0x65 на опрос — тоже STATUS
          ESP_LOGV(TAG, "RX frame: cmd=0x%02X len=%zu → treating as STATUS", cmd, rx.size());
        }
        this->parse_status_legacy_(rx);
      } else {
        ESP_LOGV(TAG, "RX frame: cmd=0x%02X len=%zu (ignored)", cmd, rx.size());
      }
    }
  }
}

void ACHIClimate::control(const climate::ClimateCall &call) {
  if (call.get_mode().has_value())
    this->mode = *call.get_mode();

  if (call.get_target_temperature().has_value())
    this->target_temperature = *call.get_target_temperature();

  bool power_on = this->mode != climate::CLIMATE_MODE_OFF;

  ESP_LOGD(TAG, "control(): mode=%d (power_on=%s)", (int) this->mode, power_on ? "YES" : "NO");

  // Сборка WRITE
  std::vector<uint8_t> wr;
  this->build_legacy_write_(wr);

  // Отправка
  this->send_frame_(wr);
  this->last_tx_ms_ = millis();
  this->waiting_ack_ = true;

  // Подождём немного и попробуем сразу принять (иногда ACK+STATUS приходит быстро)
  std::vector<uint8_t> rx;
  if (this->read_frame_(rx, 30)) {
    uint8_t cmd = 0;
    if (this->verify_and_classify_(rx, cmd)) {
      if (cmd == CMD_STATUS || cmd == CMD_WRITE) {
        ESP_LOGV(TAG, "RX frame: cmd=0x%02X len=%zu → treating as STATUS", cmd, rx.size());
        this->parse_status_legacy_(rx);
      } else {
        ESP_LOGV(TAG, "RX frame: cmd=0x%02X len=%zu (ignored)", cmd, rx.size());
      }
    }
  }
}

// ========================= Build helpers =========================

uint8_t ACHIClimate::encode_mode_hi_write_legacy_(climate::ClimateMode mode) const {
  switch (mode) {
    case climate::CLIMATE_MODE_HEAT:     return MODE_HI_HEAT;
    case climate::CLIMATE_MODE_FAN_ONLY: return MODE_HI_FAN;
    case climate::CLIMATE_MODE_DRY:      return MODE_HI_DRY;
    case climate::CLIMATE_MODE_COOL:
    default:                             return MODE_HI_COOL;
  }
}

void ACHIClimate::build_legacy_write_(std::vector<uint8_t> &b) {
  b.assign(50, 0x00);

  // Заголовок
  b[WR_IDX_HDR_0] = PFX0;
  b[WR_IDX_HDR_1] = PFX1;
  b[WR_IDX_HDR_2] = PFX2;
  b[WR_IDX_HDR_3] = PFX3;
  b[WR_IDX_LEN]   = 0x29;        // длина-поля протокола (как в твоих логах)

  // Фиксированная часть «шапки» как в логах
  b[7]  = 0x01;
  b[8]  = 0x01;
  b[9]  = 0xFE;
  b[10] = 0x01;
  b[WR_IDX_TAG0] = CMD_WRITE;  // 0x65
  b[WR_IDX_TAG1] = 0x00;
  b[WR_IDX_TAG2] = 0x00;

  // Feature/flag set — по логам 0x02 0x00 или 0x02 0x01; используем 0x02 0x00
  b[WR_IDX_FSET0] = 0x02;
  b[WR_IDX_FSET1] = 0x00;

  // b18: mode_hi | power_lo (power_lo берём из подсказки — 0x08 или 0x0C)
  const uint8_t mode_hi = this->encode_mode_hi_write_legacy_(this->mode);
  const bool power = (this->mode != climate::CLIMATE_MODE_OFF);
  const uint8_t power_lo =
      power ? ((this->power_lo_hint_ == POWER_LO_ON_08) ? POWER_LO_ON_08 : POWER_LO_ON_0C) : POWER_LO_OFF;

  b[WR_IDX_MODEP] = (uint8_t) (mode_hi | power_lo);

  // b19: уставка для WRITE остаётся «классическая»: (2*C+1)
  float tgt_c = this->target_temperature.has_value() ? *this->target_temperature : 24.0f;
  uint8_t set_c = this->clamp16_30_((int) roundf(tgt_c));
  b[WR_IDX_SETPT] = (uint8_t) ((set_c << 1) | 0x01);

  // сохраним pending для implicit-ACK
  this->pending_b18_ = b[WR_IDX_MODEP];
  this->pending_b19_ = b[WR_IDX_SETPT];

  // CRC SUM16 по полезной части до CRC (с 4-го индекса и до байта перед CRC)
  uint16_t crc = this->sum16_(&b[WR_IDX_LEN], WR_IDX_CRC_HI - WR_IDX_LEN);
  b[WR_IDX_CRC_HI] = (uint8_t) (crc >> 8);
  b[WR_IDX_CRC_LO] = (uint8_t) (crc & 0xFF);

  // Хвост
  b[WR_IDX_TAIL_0] = SUFFIX0;
  b[WR_IDX_TAIL_1] = SUFFIX1;

  ESP_LOGD(TAG, "build: len_byte[4]=0x%02X, crc_hi[%d]=0x%02X crc_lo[%d]=0x%02X",
           b[WR_IDX_LEN], WR_IDX_CRC_HI, b[WR_IDX_CRC_HI], WR_IDX_CRC_LO, b[WR_IDX_CRC_LO]);

  ESP_LOGD(TAG, "build(write-legacy): b18=0x%02X (mode_hi=0x%02X power_lo=0x%02X) b19=0x%02X (2*C+1)",
           b[WR_IDX_MODEP], mode_hi, power ? power_lo : 0x00, b[WR_IDX_SETPT]);

  ESP_LOGD(TAG, "TX write (50 bytes): %s", hexencode(b).c_str());
}

void ACHIClimate::build_status_query_(std::vector<uint8_t> &q) {
  // Короткий опросный кадр (по логам ~21 байт)
  q.clear();
  q.reserve(21);

  // Заголовок
  q.push_back(PFX0);
  q.push_back(PFX1);
  q.push_back(PFX2);
  q.push_back(PFX3);

  // Поле "длины" в коротком запросе по логам = 0x0C
  q.push_back(0x0C);

  // заполним как в логах
  // 00 00 01 01 FE 01 00 00 66 00 00 00 01
  q.push_back(0x00);
  q.push_back(0x00);
  q.push_back(0x01);
  q.push_back(0x01);
  q.push_back(0xFE);
  q.push_back(0x01);
  q.push_back(0x00);
  q.push_back(0x00);
  q.push_back(CMD_STATUS);
  q.push_back(0x00);
  q.push_back(0x00);
  q.push_back(0x00);
  q.push_back(0x01);

  // CRC (SUM16 по байтам начиная с len-байта и до конца перед CRC)
  uint16_t crc = this->sum16_(&q[4], q.size() - 4);
  q.push_back((uint8_t) (crc & 0xFF));  // исторически в логах на коротком запросе выводился один байт CRC как 0xB4,
                                        // но чтобы сохранить согласованность со «старший-потом-младший», добавим один байт:
  // Для совместимости с твоими логами, где видно один байт CRC перед хвостом, оставим однобайтовую запись:
  // NB: часть блоков игнорит это поле.
  // Хвост
  q.push_back(SUFFIX0);
  q.push_back(SUFFIX1);
}

// ========================= TX/RX =========================

void ACHIClimate::send_frame_(const std::vector<uint8_t> &frame) {
  // Пишем байты единым буфером (UARTDevice::write поддерживает вектор)
  this->write_array(frame.data(), frame.size());
  this->flush();
}

bool ACHIClimate::read_frame_(std::vector<uint8_t> &frame, uint32_t timeout_ms) {
  const uint32_t start = millis();
  // Синхронизируемся по 0xF4 0xF5
  enum { S_IDLE, S_F4, S_F5, S_COLLECT } state = S_IDLE;
  std::vector<uint8_t> buf;
  buf.reserve(160);

  while (millis() - start < timeout_ms) {
    uint8_t c;
    if (!this->available()) {
      delay(1);
      continue;
    }
    if (!this->read_byte(&c))
      continue;

    switch (state) {
      case S_IDLE:
        if (c == PFX0) state = S_F4;
        break;
      case S_F4:
        if (c == PFX1) {
          buf.clear();
          buf.push_back(PFX0);
          buf.push_back(PFX1);
          state = S_F5;
        } else {
          state = S_IDLE;
        }
        break;
      case S_F5:
        buf.push_back(c); // 0x00
        state = S_COLLECT;
        break;
      case S_COLLECT:
        buf.push_back(c);
        // детект окончания: два последних байта — 0xF4 0xFB
        if (buf.size() >= 4) {
          size_t n = buf.size();
          if (buf[n - 2] == SUFFIX0 && buf[n - 1] == SUFFIX1) {
            frame = std::move(buf);
            return true;
          }
        }
        break;
    }
  }
  return false;
}

bool ACHIClimate::verify_and_classify_(const std::vector<uint8_t> &b, uint8_t &cmd) {
  if (b.size() < 10) return false;
  if (b[0] != PFX0 || b[1] != PFX1 || b[2] != PFX2 || b[3] != PFX3) return false;
  if (b[b.size() - 2] != SUFFIX0 || b[b.size() - 1] != SUFFIX1) return false;

  // Команда — это байт 13 по твоим логам (0x66 или 0x65)
  cmd = b.size() > 13 ? b[13] : 0x00;
  return true;
}

// ========================= Parse =========================

void ACHIClimate::parse_status_legacy_(const std::vector<uint8_t> &b) {
  if (b.size() <= ST_IDX_B19) return;

  const uint8_t b18 = b[ST_IDX_B18];
  const uint8_t b19 = b[ST_IDX_B19];

  // Небольшая «сырометрика», как в логах
  if (esp_log_level_get(TAG) <= ESPHOME_LOG_LEVEL_VERY_VERBOSE) {
    uint8_t b20 = (b.size() > 20) ? b[20] : 0x00;
    uint8_t b21 = (b.size() > 21) ? b[21] : 0x00;
    uint8_t b32 = (b.size() > 32) ? b[32] : 0x00;
    uint8_t b33 = (b.size() > 33) ? b[33] : 0x00;
    uint8_t b35 = (b.size() > 35) ? b[35] : 0x00;
    uint8_t b36 = (b.size() > 36) ? b[36] : 0x00;
    ESP_LOGV(TAG, "STATUS raw: b18=0x%02X b19=0x%02X b20=0x%02X b21=0x%02X b32=0x%02X b33=0x%02X b35=0x%02X b36=0x%02X",
             b18, b19, b20, b21, b32, b33, b35, b36);
  }

  // b18: mode_hi | power_lo
  const uint8_t mode_hi = b18 & B18_MODE_HI_MASK;
  const uint8_t power_lo = b18 & B18_POWER_LO_MASK;

  // Подсказка для будущих WRITE — чем блок помечает «включено»
  this->power_lo_hint_ = power_lo;

  // Power: считаем ON, если low nibble == 0x0C ИЛИ 0x08
  const bool power_on = (power_lo == POWER_LO_ON_0C) || (power_lo == POWER_LO_ON_08);

  // Mode
  climate::ClimateMode m = climate::CLIMATE_MODE_COOL;
  switch (mode_hi) {
    case MODE_HI_FAN:  m = climate::CLIMATE_MODE_FAN_ONLY; break;
    case MODE_HI_HEAT: m = climate::CLIMATE_MODE_HEAT;     break;
    case MODE_HI_COOL: m = climate::CLIMATE_MODE_COOL;     break;
    case MODE_HI_DRY:  m = climate::CLIMATE_MODE_DRY;      break;
    default:           m = climate::CLIMATE_MODE_COOL;     break;
  }

  // b19: авто-детект формата уставки
  // - если чётный → «прямые °C»
  // - если нечётный → (2*C+1)>>1
  uint8_t set_c = 24;
  const bool even_format = ((b19 & 0x01) == 0);
  this->status_temp_even_ = even_format;
  if (even_format) {
    set_c = clamp16_30_(b19);
  } else {
    set_c = clamp16_30_((b19 >> 1) & 0x3F);
  }

  // Температуры: из твоих логов мы точно знаем «Pipe Temperature»,
  // остальное (air) зависит от конкретной модели и расположения полей.
  if (this->pipe_sensor_ != nullptr) {
    // В логах Pipe = 11, 17, 28 — берём из b? (у тебя на железе это было отдано уже в обработке парсера).
    // Здесь оставим как есть — если есть внешний датчик, передавай его отдельно.
    // В этом примере не вытаскиваем pipe из кадра, чтобы не гадать по смещению.
  }

  // Применяем к состоянию
  if (!power_on) {
    this->mode = climate::CLIMATE_MODE_OFF;
  } else {
    this->mode = m;
  }
  this->target_temperature = (float) set_c;

  // Обновим current_temperature, если у тебя есть отдельный сенсор — он это сделает сам.
  // Если надо из STATUS — добавь разбор соответствующего байта и присвой сюда.

  // implicit-ACK: если мы ждали подтверждения и пришёл статус с теми же b18/b19, снимаем lock
  if (this->waiting_ack_) {
    bool a = (b18 == this->pending_b18_);
    bool btemp_match = false;
    if (this->status_temp_even_) {
      btemp_match = (clamp16_30_(b19) == clamp16_30_(this->pending_b19_ >> 1));
    } else {
      btemp_match = (b19 == this->pending_b19_);
    }
    if (a && btemp_match) {
      ESP_LOGV(TAG, "STATUS received while waiting ACK → clearing lock");
      this->waiting_ack_ = false;
    }
  }

  // Публикуем
  ESP_LOGD(TAG, "'%s' - Sending state:", this->get_name().c_str());
  ESP_LOGD(TAG, "  Mode: %s", climate::climate_mode_to_string(this->mode));
  if (this->swing_mode.has_value())
    ESP_LOGD(TAG, "  Swing Mode: %s", climate::climate_swing_mode_to_string(*this->swing_mode));
  if (this->target_temperature.has_value())
    ESP_LOGD(TAG, "  Target Temperature: %.2f°C", *this->target_temperature);

  this->publish_state();
}

// ========================= CRC =========================

uint16_t ACHIClimate::sum16_(const uint8_t *data, size_t len) const {
  // простой SUM16 (big-endian при записи)
  // используется для WRITE кадров (и подходит большинству «legacy» плат)
  uint32_t s = 0;
  for (size_t i = 0; i < len; i++) s += data[i];
  s &= 0xFFFF;
  return (uint16_t) s;
}

}  // namespace ac_hi
}  // namespace esphome
