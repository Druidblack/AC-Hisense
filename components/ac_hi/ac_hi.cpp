#include "ac_hi.h"
#include <cmath>
#include <algorithm>
#include <cinttypes>
#include <cstdio>

namespace esphome {
namespace ac_hi {

static const char *const TAG = "ac_hi.climate";

// ---- Локальные хелперы для (де)кодирования режима ----
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
  switch (m) {
    case climate::CLIMATE_MODE_FAN_ONLY: return 0x01;
    case climate::CLIMATE_MODE_HEAT:     return 0x03;
    case climate::CLIMATE_MODE_COOL:     return 0x05;
    case climate::CLIMATE_MODE_DRY:      return 0x07;
    default:                             return 0x05; // AUTO нет, используем COOL
  }
}

// ---- Fingerprint helpers (FNV-1a 32-bit) ----
static inline uint32_t fnv1a32_step_(uint32_t h, uint8_t b) {
  h ^= b;
  h *= 16777619u;
  return h;
}

uint32_t ACHIClimate::make_control_fingerprint_from_fields_(bool power, climate::ClimateMode m, uint8_t t,
                                                            climate::ClimateFanMode f, climate::ClimateSwingMode s,
                                                            bool turbo, bool eco, bool quiet, bool led, uint8_t sleep) {
  uint32_t h = 2166136261u;
  h = fnv1a32_step_(h, power ? 1 : 0);
  h = fnv1a32_step_(h, static_cast<uint8_t>(m));
  h = fnv1a32_step_(h, t);
  h = fnv1a32_step_(h, static_cast<uint8_t>(f));
  h = fnv1a32_step_(h, static_cast<uint8_t>(s));
  h = fnv1a32_step_(h, turbo ? 1 : 0);
  h = fnv1a32_step_(h, eco ? 1 : 0);
  h = fnv1a32_step_(h, quiet ? 1 : 0);
  h = fnv1a32_step_(h, led ? 1 : 0);
  h = fnv1a32_step_(h, sleep);
  return h;
}

uint32_t ACHIClimate::actual_fingerprint_() const {
  return const_cast<ACHIClimate*>(this)->make_control_fingerprint_from_fields_(
    this->power_on_, this->mode_, this->target_c_, this->fan_, this->swing_,
    this->turbo_, this->eco_, this->quiet_, this->led_, this->sleep_stage_);
}

uint32_t ACHIClimate::desired_fingerprint_() const {
  if (!desired_valid_) return 0;
  return const_cast<ACHIClimate*>(this)->make_control_fingerprint_from_fields_(
    desired_.power_on, desired_.mode, desired_.target_c, desired_.fan, desired_.swing,
    desired_.turbo, desired_.eco, desired_.quiet, desired_.led, desired_.sleep_stage);
}

// ---- Debug: hex dump & diffs ----
void ACHIClimate::log_frame_hex_(const char *title, const std::vector<uint8_t> &buf, size_t max_len) const {
  ESP_LOGV(TAG, "%s (len=%u, show<=%u)", title, (unsigned)buf.size(), (unsigned)max_len);
  char line[128];
  size_t shown = std::min(max_len, buf.size());
  for (size_t i = 0; i < shown; i += 16) {
    int n = snprintf(line, sizeof(line), "  %03u: ", (unsigned)i);
    for (size_t j = 0; j < 16 && (i + j) < shown; j++) {
      n += snprintf(line + n, sizeof(line) - n, "%02X ", buf[i + j]);
    }
    ESP_LOGV(TAG, "%s", line);
  }
  if (buf.size() > shown) {
    ESP_LOGV(TAG, "  ... (%u bytes more)", (unsigned)(buf.size() - shown));
  }
}

void ACHIClimate::log_diff_desired_actual_() const {
  if (!this->desired_valid_) {
    ESP_LOGD(TAG, "No desired state captured; nothing to diff.");
    return;
  }
  bool any = false;
  if (this->power_on_ != this->desired_.power_on) {
    ESP_LOGW(TAG, "DIFF power_on: actual=%d desired=%d", this->power_on_, this->desired_.power_on);
    any = true;
  }
  if (this->mode_ != this->desired_.mode) {
    ESP_LOGW(TAG, "DIFF mode: actual=%d desired=%d", (int)this->mode_, (int)this->desired_.mode);
    any = true;
  }
  if (this->target_c_ != this->desired_.target_c) {
    ESP_LOGW(TAG, "DIFF setpoint: actual=%u desired=%u", (unsigned)this->target_c_, (unsigned)this->desired_.target_c);
    any = true;
  }
  if (this->fan_ != this->desired_.fan) {
    ESP_LOGW(TAG, "DIFF fan: actual=%d desired=%d", (int)this->fan_, (int)this->desired_.fan);
    any = true;
  }
  if (this->swing_ != this->desired_.swing) {
    ESP_LOGW(TAG, "DIFF swing: actual=%d desired=%d", (int)this->swing_, (int)this->desired_.swing);
    any = true;
  }
  if (this->turbo_ != this->desired_.turbo) {
    ESP_LOGW(TAG, "DIFF turbo: actual=%d desired=%d", this->turbo_, this->desired_.turbo);
    any = true;
  }
  if (this->eco_ != this->desired_.eco) {
    ESP_LOGW(TAG, "DIFF eco: actual=%d desired=%d", this->eco_, this->desired_.eco);
    any = true;
  }
  if (this->quiet_ != this->desired_.quiet) {
    ESP_LOGW(TAG, "DIFF quiet: actual=%d desired=%d", this->quiet_, this->desired_.quiet);
    any = true;
  }
  if (this->led_ != this->desired_.led) {
    ESP_LOGW(TAG, "DIFF led: actual=%d desired=%d", this->led_, this->desired_.led);
    any = true;
  }
  if (this->sleep_stage_ != this->desired_.sleep_stage) {
    ESP_LOGW(TAG, "DIFF sleep_stage: actual=%u desired=%u", (unsigned)this->sleep_stage_, (unsigned)this->desired_.sleep_stage);
    any = true;
  }
  if (!any) ESP_LOGD(TAG, "No differences between actual and desired control fields.");
}

void ACHIClimate::setup() {
  this->mode = climate::CLIMATE_MODE_OFF;
  this->target_temperature = 24;
  this->fan_mode = climate::CLIMATE_FAN_AUTO;
  this->swing_mode = climate::CLIMATE_SWING_OFF;
  this->publish_state();
  ESP_LOGI(TAG, "Setup done. Presets=%s, initial target=%u°C",
           this->enable_presets_ ? "on" : "off", (unsigned)this->target_c_);
}

void ACHIClimate::update() {
  // Периодический опрос: короткий запрос статуса
  if (!this->writing_lock_) {
    ESP_LOGV(TAG, "TX status query (0x66)");
    this->send_query_status_();
  } else {
    ESP_LOGV(TAG, "Skip query: writing_lock_=true");
    // Диагностика/автовыход из зависшего ожидания ACK
    uint32_t now = esphome::millis();
    uint32_t elapsed = (this->last_write_sent_at_ == 0) ? 0 : (now - this->last_write_sent_at_);
    if (this->last_write_sent_at_ != 0 && elapsed > 900) {
      if (now - this->last_ack_warn_at_ > 1000) {  // не спамить каждую итерацию
        ESP_LOGW(TAG, "Still waiting for ACK(0x65): %u ms elapsed since write. last_cmd_seen=0x%02X, last_ack_at=%u ms ago",
                 (unsigned)elapsed, this->last_cmd_seen_,
                 (unsigned)((this->last_ack_at_ == 0) ? 0 : (now - this->last_ack_at_)));
        this->last_ack_warn_at_ = now;
      }
    }
    // Жёсткий таймаут ACK — снимаем лок и планируем повтор
    if (this->last_write_sent_at_ != 0 && elapsed > this->ack_timeout_ms_) {
      ESP_LOGW(TAG, "ACK timeout (%u ms). Releasing lock and scheduling enforce retry.",
               (unsigned)this->ack_timeout_ms_);
      this->writing_lock_ = false;
      this->pending_write_ = false;
      // ускорим следующий дожим
      this->next_enforce_tx_at_ = now;
      if (this->enforce_backoff_steps_ < 10) this->enforce_backoff_steps_++;
    }
  }

  // Дожим целевого состояния из HA (если активирован)
  if (this->enforce_from_ha_ && !this->writing_lock_) {
    uint32_t now = esphome::millis();
    if (now >= this->next_enforce_tx_at_) {
      ESP_LOGD(TAG, "ENFORCE tick: retry=%u backoff_step=%u next_at=%u ms, desired_fp=0x%08X",
               (unsigned)this->enforce_retry_counter_, (unsigned)this->enforce_backoff_steps_,
               (unsigned)this->next_enforce_tx_at_, (unsigned)this->desired_fp_);
      // Повторяем ту же команду (tx_bytes_ уже сформирован в control())
      this->send_write_changes_();
      uint32_t add = this->enforce_interval_ms_ + static_cast<uint32_t>(this->enforce_backoff_steps_) * 200U;
      this->next_enforce_tx_at_ = now + add;
      if (this->enforce_backoff_steps_ < 10) this->enforce_backoff_steps_++;
      if (this->enforce_retry_counter_ < 255) this->enforce_retry_counter_++;
    }
  }
}

void ACHIClimate::loop() {
  // Накапливаем входящие байты неблокирующе
  uint8_t c;
  while (this->read_byte(&c)) {
    rx_.push_back(c);
  }

  // Скользящее окно/компакция буфера
  if (rx_start_ > RX_COMPACT_THRESHOLD) {
    rx_.erase(rx_.begin(), rx_.begin() + static_cast<std::ptrdiff_t>(rx_start_));
    rx_start_ = 0;
  }
  if (rx_.size() - rx_start_ > 4096) {
    size_t remain = rx_.size() - rx_start_;
    if (remain >= 1 && rx_.back() == HI_HDR0) {
      uint8_t keep = rx_.back();
      rx_.clear();
      rx_.push_back(keep);
      rx_start_ = 0;
    } else {
      rx_.clear();
      rx_start_ = 0;
    }
  }

  // Пробуем извлечь кадры с ограничением по времени/количеству
  this->try_parse_frames_from_buffer_(MAX_PARSE_TIME_MS);
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

  // Логируем приходящие поля
  ESP_LOGD(TAG, "HA control() called: has_mode=%d, has_target=%d, has_fan=%d, has_swing=%d, has_preset=%d",
           call.get_mode().has_value(), call.get_target_temperature().has_value(),
           call.get_fan_mode().has_value(), call.get_swing_mode().has_value(),
           call.get_preset().has_value());

  if (call.get_mode().has_value()) {
    auto m = *call.get_mode();
    if (m == climate::CLIMATE_MODE_OFF) {
      this->power_on_ = false;
    } else {
      this->power_on_ = true;
      this->mode_ = m;
    }
    need_write = true;
  }

  if (call.get_target_temperature().has_value()) {
    auto t = *call.get_target_temperature();
    if (!std::isnan(t)) {
      uint8_t c = static_cast<uint8_t>(std::round(t));
      c = std::max<uint8_t>(16, std::min<uint8_t>(30, c));
      this->target_c_ = c;
      need_write = true;
    }
  }

  if (call.get_fan_mode().has_value()) {
    this->fan_ = *call.get_fan_mode();
    need_write = true;
  }

  if (call.get_swing_mode().has_value()) {
    this->swing_ = *call.get_swing_mode();
    need_write = true;
  }

  if (call.get_preset().has_value()) {
    auto p = *call.get_preset();
    this->eco_ = (p == climate::CLIMATE_PRESET_ECO);
    this->turbo_ = (p == climate::CLIMATE_PRESET_BOOST);
    this->sleep_stage_ = (p == climate::CLIMATE_PRESET_SLEEP) ? 1 : 0;
    need_write = true;
  }

  ESP_LOGD(TAG, "Control resolved: power_on=%d mode=%d target_c=%u fan=%d swing=%d turbo=%d eco=%d quiet=%d led=%d sleep=%u need_write=%d",
           this->power_on_, (int)this->mode_, (unsigned)this->target_c_, (int)this->fan_, (int)this->swing_,
           this->turbo_, this->eco_, this->quiet_, this->led_, (unsigned)this->sleep_stage_, need_write);

  if (need_write) {
    // Сборка TX-кадра
    uint8_t power_bin = this->power_on_ ? 0b00001100 : 0b00000100;  // low nibble
    uint8_t mode_hi   = static_cast<uint8_t>(encode_nibble_from_mode(this->mode_) << 4);
    tx_bytes_[18] = static_cast<uint8_t>(power_bin + mode_hi);

    // запись уставки напрямую
    tx_bytes_[19] = encode_temp_(this->target_c_);

    tx_bytes_[16] = encode_fan_byte_(this->fan_);           // fan
    tx_bytes_[17] = encode_sleep_byte_(this->sleep_stage_); // sleep

    bool v_swing = (this->swing_ == climate::CLIMATE_SWING_VERTICAL) || (this->swing_ == climate::CLIMATE_SWING_BOTH);
    bool h_swing = (this->swing_ == climate::CLIMATE_SWING_HORIZONTAL) || (this->swing_ == climate::CLIMATE_SWING_BOTH);
    uint8_t updown_bin = encode_swing_ud_(v_swing);
    uint8_t leftright_bin = encode_swing_lr_(h_swing);
    tx_bytes_[32] = static_cast<uint8_t>(updown_bin + leftright_bin);

    uint8_t turbo_bin = this->turbo_ ? 0b00001100 : 0b00000100;
    uint8_t eco_bin = this->eco_ ? 0b00110000 : 0b00000000;
    tx_bytes_[33] = (this->turbo_ ? turbo_bin : (eco_bin ? eco_bin : 0));

    this->quiet_ = (this->fan_ == climate::CLIMATE_FAN_QUIET);
    tx_bytes_[35] = this->quiet_ ? 0b00110000 : 0b00000000;

    tx_bytes_[36] = this->led_ ? 0b11000000 : 0b01000000;

    if (this->turbo_) {
      tx_bytes_[19] = this->target_c_; // turbo не трогаем уставку
      tx_bytes_[33] = turbo_bin;
      tx_bytes_[35] = 0;               // override quiet
    }

    ESP_LOGD(TAG, "TX fields: [18]=0x%02X(pwr+mode) [19]=0x%02X(set) [16]=0x%02X(fan) [17]=0x%02X(sleep) [32]=0x%02X(swing) [33]=0x%02X(turbo/eco) [35]=0x%02X(quiet) [36]=0x%02X(LED)",
             tx_bytes_[18], tx_bytes_[19], tx_bytes_[16], tx_bytes_[17], tx_bytes_[32], tx_bytes_[33], tx_bytes_[35], tx_bytes_[36]);

    // === Включаем режим приоритета HA: фиксируем целевое состояние и начинаем дожим ===
    desired_.power_on = this->power_on_;
    desired_.mode = this->mode_;
    desired_.target_c = this->target_c_;
    desired_.fan = this->fan_;
    desired_.swing = this->swing_;
    desired_.turbo = this->turbo_;
    desired_.eco = this->eco_;
    desired_.quiet = this->quiet_;
    desired_.led = this->led_;
    desired_.sleep_stage = this->sleep_stage_;
    desired_valid_ = true;
    desired_fp_ = this->desired_fingerprint_();

    enforce_from_ha_ = true;
    accept_ir_changes_ = false;
    next_enforce_tx_at_ = 0;
    enforce_backoff_steps_ = 0;
    enforce_retry_counter_ = 0;

    ESP_LOGI(TAG, "ENFORCE start: desired_fp=0x%08X power=%d mode=%d set=%u fan=%d swing=%d turbo=%d eco=%d quiet=%d led=%d sleep=%u",
             (unsigned)desired_fp_, desired_.power_on, (int)desired_.mode, (unsigned)desired_.target_c,
             (int)desired_.fan, (int)desired_.swing, desired_.turbo, desired_.eco, desired_.quiet, desired_.led, (unsigned)desired_.sleep_stage);

    this->writing_lock_ = true;
    this->pending_write_ = true;
    this->send_write_changes_();
  }

  // Оптимистично публикуем ожидаемое состояние
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
  this->publish_state();
}

// ---- Кодирование полей ----

uint8_t ACHIClimate::encode_mode_hi_nibble_(climate::ClimateMode m) {
  return static_cast<uint8_t>(encode_nibble_from_mode(m) << 4);
}

uint8_t ACHIClimate::encode_fan_byte_(climate::ClimateFanMode f) {
  // AUTO->1, QUIET->10, LOW->12, MED->14, HIGH->16; при записи +1
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

// ---- Транспорт/CRC ----

void ACHIClimate::send_query_status_() {
  for (auto b : this->query_) this->write_byte(b);
  this->flush();
  ESP_LOGV(TAG, "Sent status query (0x66)");
  this->log_frame_hex_("QUERY hex", this->query_, 32);
}

void ACHIClimate::calc_and_patch_crc_(std::vector<uint8_t> &buf) {
  // сумма байтов с 2 по (len-4)
  const int n = static_cast<int>(buf.size());
  uint16_t csum = 0;
  for (int i = 2; i < n - 4; i++) csum = static_cast<uint16_t>(csum + buf[i]);
  buf[n - 4] = static_cast<uint8_t>((csum & 0xFF00) >> 8);
  buf[n - 3] = static_cast<uint8_t>(csum & 0x00FF);
}

bool ACHIClimate::validate_crc_(const std::vector<uint8_t> &buf, uint16_t *out_sum) const {
  if (buf.size() < 8) return false;
  const size_t n = buf.size();
  uint16_t csum = 0;
  for (size_t i = 2; i < n - 4; i++) csum = static_cast<uint16_t>(csum + buf[i]);
  uint8_t cr1 = static_cast<uint8_t>((csum & 0xFF00) >> 8);
  uint8_t cr2 = static_cast<uint8_t>(csum & 0x00FF);
  if (out_sum) *out_sum = csum;
  return (buf[n - 4] == cr1) && (buf[n - 3] == cr2);
}

void ACHIClimate::send_write_changes_() {
  auto frame = this->tx_bytes_;
  this->calc_and_patch_crc_(frame);
  this->log_frame_hex_("TX WRITE hex (0x65)", frame, 64);
  for (auto b : frame) this->write_byte(b);
  this->flush();
  this->last_write_sent_at_ = esphome::millis();
  ESP_LOGD(TAG, "WRITE sent, writing_lock_=%d pending_write_=%d (t=%u ms)",
           this->writing_lock_, this->pending_write_, (unsigned)this->last_write_sent_at_);
}

// ---- RX сканер/парсер ----

void ACHIClimate::try_parse_frames_from_buffer_(uint32_t budget_ms) {
  std::vector<uint8_t> frame;
  uint8_t handled = 0;
  const uint32_t start = esphome::millis();

  while (handled < MAX_FRAMES_PER_LOOP &&
         (esphome::millis() - start) < budget_ms &&
         this->extract_next_frame_(frame)) {

    uint16_t sum = 0;
    bool crc_ok = this->validate_crc_(frame, &sum);
    uint8_t cmd = (frame.size() > 13 ? frame[13] : 0xFF);
    this->last_cmd_seen_ = cmd;

    ESP_LOGV(TAG, "RX frame: len=%u cmd=0x%02X crc_ok=%d sum=0x%04X",
             (unsigned)frame.size(), cmd, crc_ok, (unsigned)sum);
    this->log_frame_hex_("RX hex", frame, 64);

    // Разбор
    this->handle_frame_(frame);
    handled++;

    // Сохраняем последнюю сумму, чтобы подавлять повторы (как было)
    last_status_crc_ = sum;
  }
}

bool ACHIClimate::extract_next_frame_(std::vector<uint8_t> &frame) {
  frame.clear();

  if (rx_.size() <= rx_start_ + 5) return false;

  // 1) Ищем заголовок F4 F5 начиная с rx_start_
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
    } else {
      rx_.clear();
      rx_start_ = 0;
    }
    return false;
  }

  rx_start_ = i;

  // 2) Попытка среза по объявленной длине: полный размер = bytes[4] + 9
  if (rx_.size() > rx_start_ + 5) {
    uint8_t decl = rx_[rx_start_ + 4];
    size_t expected_total = static_cast<size_t>(decl) + 9U;

    if (rx_.size() >= rx_start_ + expected_total) {
      frame.insert(frame.end(),
                   rx_.begin() + static_cast<std::ptrdiff_t>(rx_start_),
                   rx_.begin() + static_cast<std::ptrdiff_t>(rx_start_ + expected_total));
      rx_start_ += expected_total;
      return true;
    }
  }

  // 3) Иначе — до первого хвоста F4 FB
  size_t j = rx_start_ + 2;
  bool found_tail = false;
  for (; j + 1 < rx_.size(); j++) {
    if (rx_[j] == HI_TAIL0 && rx_[j + 1] == HI_TAIL1) {
      found_tail = true;
      break;
    }
  }
  if (!found_tail) return false;

  frame.insert(frame.end(),
               rx_.begin() + static_cast<std::ptrdiff_t>(rx_start_),
               rx_.begin() + static_cast<std::ptrdiff_t>(j + 2));
  rx_start_ = j + 2;
  return true;
}

void ACHIClimate::handle_frame_(const std::vector<uint8_t> &b) {
  if (b.size() < 20) return;

  const uint8_t cmd = b[13];

  if (cmd == 102 /*0x66 status resp*/ ) {
    this->parse_status_102_(b);
    // Если статус пришёл вскоре после записи, считаем это имплицитным ACK
    if (this->writing_lock_ && this->last_write_sent_at_ != 0) {
      uint32_t now = esphome::millis();
      uint32_t elapsed = now - this->last_write_sent_at_;
      if (elapsed <= this->ack_implicit_window_ms_) {
        ESP_LOGW(TAG, "Implicit ACK via STATUS (0x66) after %u ms. Releasing lock.", (unsigned)elapsed);
        this->writing_lock_ = false;
        this->pending_write_ = false;
      }
    }
  } else if (cmd == 101 /*0x65 ack*/) {
    this->handle_ack_101_();
  } else {
    ESP_LOGW(TAG, "Unknown frame cmd=0x%02X len=%u", cmd, (unsigned)b.size());
  }
}

void ACHIClimate::parse_status_102_(const std::vector<uint8_t> &bytes) {
  // Печать «сырых» критичных байтов для быстрой сверки
  if (bytes.size() > 38) {
    ESP_LOGV(TAG, "STATUS RAW: b16=0x%02X b17=0x%02X b18=0x%02X b19=0x%02X b35=0x%02X b36=0x%02X b37=0x%02X",
             bytes[16], bytes[17], bytes[18], bytes[19], bytes[35], bytes[36], bytes[37]);
  }

  // Питание/режим/ветер/сон/уставка/темпы
  bool power = (bytes[18] & 0b00001000) != 0;
  this->power_on_ = power;

  uint8_t nib = static_cast<uint8_t>((bytes[18] >> 4) & 0x0F);
  this->mode_ = decode_mode_from_nibble(nib);

  uint8_t raw_wind = bytes[16];
  climate::ClimateFanMode new_fan = climate::CLIMATE_FAN_AUTO;
  if (raw_wind == 1 || raw_wind == 2) new_fan = climate::CLIMATE_FAN_AUTO;
  else if (raw_wind == 11) new_fan = climate::CLIMATE_FAN_QUIET;
  else if (raw_wind == 13) new_fan = climate::CLIMATE_FAN_LOW;
  else if (raw_wind == 15) new_fan = climate::CLIMATE_FAN_MEDIUM;
  else if (raw_wind == 17) new_fan = climate::CLIMATE_FAN_HIGH;
  this->fan_ = new_fan;

  uint8_t raw_sleep = bytes[17];
  uint8_t code = (raw_sleep >> 1);
  if (code == 0) this->sleep_stage_ = 0;
  else if (code == 1) this->sleep_stage_ = 1;
  else if (code == 2) this->sleep_stage_ = 2;
  else if (code == 4) this->sleep_stage_ = 3;
  else if (code == 8) this->sleep_stage_ = 4;
  else this->sleep_stage_ = 0;

  uint8_t raw_set = bytes[19];
  if (raw_set >= 16 && raw_set <= 30) this->target_c_ = raw_set;
  this->target_temperature = this->target_c_;

  uint8_t tair = bytes[20];
  this->current_temperature = tair;

#ifdef USE_SENSOR
  if (this->pipe_sensor_ != nullptr) this->pipe_sensor_->publish_state(bytes[21]);
#endif

  // Turbo/Eco/Quiet/LED + качание
  uint8_t b35 = bytes[35];
  this->turbo_ = (b35 & 0b00000010) != 0;
  this->eco_   = (b35 & 0b00000100) != 0;
  this->quiet_ = (bytes[36] & 0b00000100) != 0;
  this->led_   = (bytes[37] & 0b10000000) != 0;

  bool updown = (bytes[35] & 0b10000000) != 0;
  bool leftright = (bytes[35] & 0b01000000) != 0;
  if (updown && leftright) this->swing_ = climate::CLIMATE_SWING_BOTH;
  else if (updown) this->swing_ = climate::CLIMATE_SWING_VERTICAL;
  else if (leftright) this->swing_ = climate::CLIMATE_SWING_HORIZONTAL;
  else this->swing_ = climate::CLIMATE_SWING_OFF;

#ifdef USE_SENSOR
  // Доп. сенсоры
  if (this->set_temp_sensor_ != nullptr) this->set_temp_sensor_->publish_state(this->target_c_);
  if (this->room_temp_sensor_ != nullptr) this->room_temp_sensor_->publish_state(tair);
  if (this->wind_sensor_ != nullptr) this->wind_sensor_->publish_state(raw_wind);
  if (this->sleep_sensor_ != nullptr) this->sleep_sensor_->publish_state(this->sleep_stage_);
  if (this->mode_sensor_ != nullptr) this->mode_sensor_->publish_state(nib & 0x0F);
  if (this->quiet_sensor_ != nullptr) this->quiet_sensor_->publish_state(this->quiet_ ? 1 : 0);
  if (this->turbo_sensor_ != nullptr) this->turbo_sensor_->publish_state(this->turbo_ ? 1 : 0);
  if (this->led_sensor_ != nullptr)   this->led_sensor_->publish_state(this->led_ ? 1 : 0);
  if (this->eco_sensor_ != nullptr)   this->eco_sensor_->publish_state(this->eco_ ? 1 : 0);
  if (this->swing_updown_sensor_ != nullptr)    this->swing_updown_sensor_->publish_state(updown ? 1 : 0);
  if (this->swing_leftright_sensor_ != nullptr) this->swing_leftright_sensor_->publish_state(leftright ? 1 : 0);
  if (this->compr_freq_set_sensor_ != nullptr) this->compr_freq_set_sensor_->publish_state(bytes[42]);
  if (this->compr_freq_sensor_ != nullptr)     this->compr_freq_sensor_->publish_state(bytes[43]);
  if (this->outdoor_temp_sensor_ != nullptr)   this->outdoor_temp_sensor_->publish_state(bytes[44]);
  if (this->outdoor_cond_temp_sensor_ != nullptr) this->outdoor_cond_temp_sensor_->publish_state(bytes[45]);
#endif

#ifdef USE_TEXT_SENSOR
  if (this->power_status_text_sensor_ != nullptr) this->power_status_text_sensor_->publish_state(this->power_on_ ? "ON" : "OFF");
#endif

  // Печать разобранных ключевых полей
  ESP_LOGD(TAG, "STATUS: power=%d mode=%d fan=%d sleep=%u set=%u room=%u turbo=%d eco=%d quiet=%d led=%d swingUD=%d swingLR=%d",
           this->power_on_, (int)this->mode_, (int)this->fan_, (unsigned)this->sleep_stage_,
           (unsigned)this->target_c_, (unsigned)tair, this->turbo_, this->eco_, this->quiet_, this->led_, updown, leftright);

  // Публикуем фактическое
  this->mode = this->power_on_ ? this->mode_ : climate::CLIMATE_MODE_OFF;
  this->fan_mode = this->fan_;
  this->swing_mode = this->swing_;
  if (enable_presets_) {
    if (this->turbo_) this->preset = climate::CLIMATE_PRESET_BOOST;
    else if (this->eco_) this->preset = climate::CLIMATE_PRESET_ECO;
    else if (this->sleep_stage_ > 0) this->preset = climate::CLIMATE_PRESET_SLEEP;
    else this->preset = climate::CLIMATE_PRESET_NONE;
  }

  // === Приоритет HA/дожим ===
  this->actual_fp_ = this->actual_fingerprint_();
  ESP_LOGD(TAG, "FP: actual=0x%08X desired=0x%08X last_applied=0x%08X enforce=%d accept_ir=%d",
           (unsigned)this->actual_fp_, (unsigned)this->desired_fp_, (unsigned)this->last_applied_fp_,
           this->enforce_from_ha_, this->accept_ir_changes_);

  if (this->enforce_from_ha_ && this->desired_valid_) {
    uint32_t dfp = this->desired_fingerprint_();
    if (this->actual_fp_ == dfp) {
      this->enforce_from_ha_ = false;
      this->accept_ir_changes_ = true;
      this->last_applied_fp_ = this->actual_fp_;
      this->enforce_backoff_steps_ = 0;
      this->enforce_retry_counter_ = 0;
      ESP_LOGI(TAG, "ENFORCE matched. IR changes accepted again.");
    } else {
      ESP_LOGW(TAG, "ENFORCE pending: actual!=desired (0x%08X != 0x%08X). Will retry at %u ms",
               (unsigned)this->actual_fp_, (unsigned)dfp, (unsigned)this->next_enforce_tx_at_);
      this->log_diff_desired_actual_();
      // Публикуем в UI целевые поля
      this->mode = this->desired_.power_on ? this->desired_.mode : climate::CLIMATE_MODE_OFF;
      this->target_temperature = this->desired_.target_c;
      this->fan_mode = this->desired_.fan;
      this->swing_mode = this->desired_.swing;
      if (enable_presets_) {
        if (this->desired_.turbo) this->preset = climate::CLIMATE_PRESET_BOOST;
        else if (this->desired_.eco) this->preset = climate::CLIMATE_PRESET_ECO;
        else if (this->desired_.sleep_stage > 0) this->preset = climate::CLIMATE_PRESET_SLEEP;
        else this->preset = climate::CLIMATE_PRESET_NONE;
      }
    }
  } else {
    if (this->accept_ir_changes_) {
      if (this->actual_fp_ != this->last_applied_fp_) {
        ESP_LOGI(TAG, "IR/state change accepted: last_applied 0x%08X -> 0x%08X",
                 (unsigned)this->last_applied_fp_, (unsigned)this->actual_fp_);
        this->last_applied_fp_ = this->actual_fp_;
      }
    } else {
      if (this->desired_valid_) {
        ESP_LOGW(TAG, "IR changes blocked temporarily; publishing desired to HA.");
        this->mode = this->desired_.power_on ? this->desired_.mode : climate::CLIMATE_MODE_OFF;
        this->target_temperature = this->desired_.target_c;
        this->fan_mode = this->desired_.fan;
        this->swing_mode = this->desired_.swing;
        if (enable_presets_) {
          if (this->desired_.turbo) this->preset = climate::CLIMATE_PRESET_BOOST;
          else if (this->desired_.eco) this->preset = climate::CLIMATE_PRESET_ECO;
          else if (this->desired_.sleep_stage > 0) this->preset = climate::CLIMATE_PRESET_SLEEP;
          else this->preset = climate::CLIMATE_PRESET_NONE;
        }
      }
    }
  }

  this->publish_state();
}

void ACHIClimate::handle_ack_101_() {
  uint32_t now = esphome::millis();
  this->last_ack_at_ = now;
  ESP_LOGD(TAG, "ACK (0x65) received at %u ms. writing_lock_=%d -> false, pending_write_=%d -> false",
           (unsigned)now, this->writing_lock_, this->pending_write_);
  this->writing_lock_ = false;
  this->pending_write_ = false;
}

} // namespace ac_hi
} // namespace esphome
