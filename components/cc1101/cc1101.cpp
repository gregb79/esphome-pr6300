#include "cc1101.h"
#include "cc1101pa.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include <climits>
#include <cmath>
#include <cstdio>

namespace esphome {
namespace cc1101 {

static const char *const TAG = "cc1101";

CC1101Component::CC1101Component() {
  this->gdo0_ = nullptr;
  this->gdo0_adc_ = nullptr;
  this->reset_ = false;
  this->output_power_requested_ = OUTPUT_POWER_MAX;
  this->output_power_effective_ = OUTPUT_POWER_MAX;
  memset(this->pa_table_, 0, sizeof(pa_table_));
  memset(&this->state_, 0, sizeof(this->state_));

  // datasheet defaults (non-listed fields are zero)

  this->state_.GDO2_CFG = 0x29;
  this->state_.GDO1_CFG = 0x2E;
  this->state_.GDO0_CFG = 0x3F;
  this->state_.FIFO_THR = 7;
  this->state_.SYNC1 = 0xD3;
  this->state_.SYNC0 = 0x91;
  this->state_.PKTLEN = 0xFF;
  this->state_.APPEND_STATUS = 1;
  this->state_.LENGTH_CONFIG = 1;
  this->state_.CRC_EN = 1;
  this->state_.WHITE_DATA = 1;
  this->state_.FREQ_IF = 0x0F;
  this->state_.FREQ2 = 0x1E;
  this->state_.FREQ1 = 0xC4;
  this->state_.FREQ0 = 0xEC;
  this->state_.DRATE_E = 0x0C;
  this->state_.CHANBW_E = 0x02;
  this->state_.DRATE_M = 0x22;
  this->state_.SYNC_MODE = 2;
  this->state_.CHANSPC_E = 2;
  this->state_.NUM_PREAMBLE = 2;
  this->state_.CHANSPC_M = 0xF8;
  this->state_.DEVIATION_M = 7;
  this->state_.DEVIATION_E = 4;
  this->state_.RX_TIME = 7;
  this->state_.CCA_MODE = 3;
  this->state_.PO_TIMEOUT = 1;
  this->state_.FOC_LIMIT = 2;
  this->state_.FOC_POST_K = 1;
  this->state_.FOC_PRE_K = 2;
  this->state_.FOC_BS_CS_GATE = 1;
  this->state_.BS_POST_KP = 1;
  this->state_.BS_POST_KI = 1;
  this->state_.BS_PRE_KP = 2;
  this->state_.BS_PRE_KI = 1;
  this->state_.MAGN_TARGET = 3;
  this->state_.AGC_LNA_PRIORITY = 1;
  this->state_.FILTER_LENGTH = 1;
  this->state_.WAIT_TIME = 1;
  this->state_.HYST_LEVEL = 2;
  this->state_.WOREVT1 = 0x87;
  this->state_.WOREVT0 = 0x6B;
  this->state_.RC_CAL = 1;
  this->state_.EVENT1 = 7;
  this->state_.RC_PD = 1;
  this->state_.MIX_CURRENT = 2;
  this->state_.LODIV_BUF_CURRENT_RX = 1;
  this->state_.LNA2MIX_CURRENT = 1;
  this->state_.LNA_CURRENT = 1;
  this->state_.LODIV_BUF_CURRENT_TX = 1;
  this->state_.FSCAL3_LO = 9;
  this->state_.CHP_CURR_CAL_EN = 2;
  this->state_.FSCAL3_HI = 2;
  this->state_.FSCAL2 = 0x0A;
  this->state_.FSCAL1 = 0x20;
  this->state_.FSCAL0 = 0x0D;
  this->state_.RCCTRL1 = 0x41;
  this->state_.FSTEST = 0x59;
  this->state_.PTEST = 0x7F;
  this->state_.AGCTEST = 0x3F;
  this->state_.TEST2 = 0x88;
  this->state_.TEST1 = 0x31;
  this->state_.TEST0_LO = 1;
  this->state_.VCO_SEL_CAL_EN = 1;
  this->state_.TEST0_HI = 2;

  // old settings which make no sense to me

  /*
  // MDMCFG1
  this->state_.NUM_PREAMBLE = 0;  // ?
  this->state_.PO_TIMEOUT = 2;    // Approx. 149 – 155 μs
  // FOCCFG
  this->state_.FOC_BS_CS_GATE = 0; // ?
  // BSCFG
  this->state_.BS_PRE_KP = 1; // ?
  // FSCAL3
  this->state_.FSCAL3_HI = 3; // ?
  // FSCAL2
  this->state_.VCO_CORE_H_EN = 1; // Choose high (1) / low (0) VCO ?
  // FSCAL1
  this->state_.FSCAL1 = 0; // ?
  // FSCAL0
  this->state_.FSCAL0 = 31; // ?
  // TEST0
  this->state_.VCO_SEL_CAL_EN = 0; // ?
  */

  // old settings that kinda make sense

  // TODO: this->set_mode_(false)

  // IOCFGx
  this->state_.GDO2_CFG = 0x0D;  // Async serial output (TODO: enum)
  this->state_.GDO0_CFG = 0x0D;

  // PKTCTRL0
  // Setting PKTCTRL0.PKT_FORMAT to 3 enables asynchronous serial mode. In TX, the
  // GDO0 pin is used for data input (TX data). Data output can be on GDO0, GDO1, or
  // GDO2. This is set by the IOCFG0.GDO0_CFG, IOCFG1.GDO1_CFG
  // and IOCFG2.GDO2_CFG fields.
  this->state_.PKT_FORMAT = 3;  // (TODO: enum)
  // With PKTCTRL0.LENGTH_CONFIG=2, the packet length is set to infinite and transmission
  // and reception will continue until turned off manually. As described in the next section,
  // this can be used to support packet formats with different length configuration than natively
  // supported by CC1101
  this->state_.LENGTH_CONFIG = 2;  // (TODO: enum)

  // MDMCFG0 (TODO: this->set_..)
  this->state_.FS_AUTOCAL = 1;  // When going from IDLE to RX or TX (or FSTXON)

  this->set_tuner_frequency(433920);
  this->set_tuner_if_frequency(153);
  this->set_tuner_bandwidth(203);
  this->set_tuner_channel(0);
  this->set_tuner_channel_spacing(200);
  this->set_tuner_symbol_rate(5000);
  this->set_tuner_sync_mode(SyncMode::SYNC_MODE_NONE);
  this->set_tuner_carrier_sense_above_threshold(true);
  this->set_tuner_modulation(Modulation::MODULATION_ASK_OOK);
  // this->set_tuner_fsk_deviation(47.607f);
  // this->set_tuner_msk_deviation(0);
  this->set_agc_magn_target(MagnTarget::MAGN_TARGET_42DB);  // ?
  this->set_agc_max_lna_gain(MaxLnaGain::MAX_LNA_GAIN_DEFAULT);
  this->set_agc_max_dvga_gain(MaxDvgaGain::MAX_DVGA_GAIN_MINUS_3);  // ?
  this->set_agc_lna_priority(false);                                // ?
  this->set_agc_wait_time(WaitTime::WAIT_TIME_32_SAMPLES);          // ?
}

// overrides

void CC1101Component::setup() {
  if (this->gdo0_ != nullptr) {
#ifdef USE_ESP8266
    // ESP8266 GDO0 generally input, switched to output for TX
    // ESP32 GDO0 output, GDO2 input
    // if there is an ADC, GDO0 is input only while reading temperature
    this->gdo0_->setup();
    this->gdo0_->pin_mode(gpio::FLAG_INPUT);
#endif
  }

  // datasheet 19.1.2

  this->cs_->digital_write(true);
  delayMicroseconds(1);
  this->cs_->digital_write(false);
  delayMicroseconds(1);
  this->cs_->digital_write(true);
  delayMicroseconds(41);
  this->cs_->digital_write(false);
  delayMicroseconds(5000);

  this->spi_setup();

  // reset

  this->send_(Command::RES);

  // Read part number and version

  this->read_(Register::PARTNUM);
  this->read_(Register::VERSION);

  if (this->state_.VERSION == 0) {
    mark_failed();
    ESP_LOGE(TAG, "Failed to reset CC1101 modem. Check connection.");
    return;
  }

  char buff[32] = {0};
  snprintf(buff, sizeof(buff), "%02X%02X", this->state_.PARTNUM, this->state_.VERSION);
  this->chip_id_ = buff;

  ESP_LOGD(TAG, "%s was found", this->chip_id_.c_str());

  for (uint8_t i = 0; i <= 0x2E; i++) {
    this->write_((Register) i);
  }

  this->write_(Register::PATABLE, this->pa_table_, sizeof(this->pa_table_));

  ESP_LOGV(TAG, "verify");

  for (uint8_t i = 0; i <= 0x2E; i++) {
    uint8_t old_value = this->regs_[i];
    this->read_((Register) i);
    ESP_LOGV(TAG, "[%d] %02X vs %02X%s", i, old_value, this->regs_[i], old_value != this->regs_[i] ? " ***" : "");
  }

  this->send_(Command::RX);

  this->publish_output_power();
  this->publish_rx_attenuation();
  this->publish_dc_blocking_filter();
  this->publish_tuner_frequency();
  this->publish_tuner_if_frequency();
  this->publish_tuner_bandwidth();
  this->publish_tuner_channel();
  this->publish_tuner_channel_spacing();
  this->publish_tuner_fsk_deviation();
  this->publish_tuner_msk_deviation();
  this->publish_tuner_symbol_rate();
  this->publish_tuner_sync_mode();
  this->publish_tuner_carrier_sense_above_threshold();
  this->publish_tuner_modulation();
  this->publish_agc_magn_target();
  this->publish_agc_max_lna_gain();
  this->publish_agc_max_dvga_gain();
  this->publish_agc_carrier_sense_abs_thr();
  this->publish_agc_carrier_sense_rel_thr();
  this->publish_agc_lna_priority();
  this->publish_agc_filter_length_fsk_msk();
  this->publish_agc_filter_length_ask_ook();
  this->publish_agc_freeze();
  this->publish_agc_wait_time();
  this->publish_agc_hyst_level();

  this->publish_chip_id_text_sensor();
  this->publish_rssi_sensor();
  this->publish_lqi_sensor();
  this->publish_temperature_sensor();
}

void CC1101Component::dump_config() {
  ESP_LOGCONFIG(TAG, "CC1101: ");
  // LOG_SPI_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "failed!");
  }
  ESP_LOGCONFIG(TAG, "  Chip: %s", this->chip_id_.c_str());
  ESP_LOGCONFIG(TAG, "  Tuner:");
  ESP_LOGCONFIG(TAG, "    Output Power: %.1f dBm", this->get_output_power());
  ESP_LOGCONFIG(TAG, "    Rx Attenuation: %d dB", (int) this->get_rx_attenuation() * 6);
  ESP_LOGCONFIG(TAG, "    DC Blocking Filter: %d", (int) this->get_dc_blocking_filter());
  ESP_LOGCONFIG(TAG, "    Frequency: %d KHz", (int) this->get_tuner_frequency());
  ESP_LOGCONFIG(TAG, "    IF Frequency: %d KHz", (int) this->get_tuner_if_frequency());
  ESP_LOGCONFIG(TAG, "    Bandwith: %d KHz", (int) this->get_tuner_bandwidth());
  ESP_LOGCONFIG(TAG, "    Channel: %d", (int) this->get_tuner_channel());
  ESP_LOGCONFIG(TAG, "    Channel Spacing: %d KHz", (int) this->get_tuner_channel_spacing());
  ESP_LOGCONFIG(TAG, "    FSK Deviation: %d KHz", (int) this->get_tuner_fsk_deviation());
  ESP_LOGCONFIG(TAG, "    MSK Deviation: %d KHz", (int) this->get_tuner_msk_deviation());
  ESP_LOGCONFIG(TAG, "    Symbol Rate: %d Baud", (int) this->get_tuner_symbol_rate());
  ESP_LOGCONFIG(TAG, "    Sync Mode: %d", (int) this->get_tuner_sync_mode());
  ESP_LOGCONFIG(TAG, "    Carrier Sense Above Threshold: %d", (int) this->get_tuner_carrier_sense_above_threshold());
  ESP_LOGCONFIG(TAG, "    Modulation: %d", (int) this->get_tuner_modulation());
  // TODO: ...and everything else...
  LOG_PIN("  CS Pin: ", this->cs_);
  LOG_PIN("  GDO0: ", this->gdo0_);
  LOG_TEXT_SENSOR("  ", "Chip ID", this->chip_id_text_sensor_);
  LOG_SENSOR("  ", "RSSI", this->rssi_sensor_);
  LOG_SENSOR("  ", "LQI", this->lqi_sensor_);
  LOG_SENSOR("  ", "Temperature sensor", this->temperature_sensor_);
  LOG_UPDATE_INTERVAL(this);
}

void CC1101Component::update() {
  this->read_(Register::RSSI);
  this->publish_rssi_sensor();

  this->read_(Register::LQI);
  this->publish_lqi_sensor();

  if (this->gdo0_ != nullptr && this->gdo0_adc_ != nullptr) {
    // TODO: read temperature if possible
    this->publish_temperature_sensor();
  }
}

void CC1101Component::loop() {}

// actions

void CC1101Component::begin_tx() {
  this->send_(Command::TX);

  if (this->gdo0_ != nullptr) {
#ifdef USE_ESP8266
#ifdef USE_ARDUINO
    noInterrupts();  // NOLINT
#else                // USE_ESP_IDF
    portDISABLE_INTERRUPTS()
#endif
    this->gdo0_->pin_mode(gpio::FLAG_OUTPUT);
#endif
  }
}

void CC1101Component::end_tx() {
  if (this->gdo0_ != nullptr) {
#ifdef USE_ESP8266
#ifdef USE_ARDUINO
    interrupts();  // NOLINT
#else              // USE_ESP_IDF
    portENABLE_INTERRUPTS()
#endif
    this->gdo0_->pin_mode(gpio::FLAG_INPUT);
#endif
  }

  this->send_(Command::RX);
}

// protected

void CC1101Component::strobe_(Command cmd) {
  uint8_t index = (uint8_t) cmd;
  if (cmd < Command::RES || cmd > Command::NOP) {
    ESP_LOGE(TAG, "%s(0x%02X) invalid register address", __func__, index);
    return;
  }

  if (!this->reset_) {
    if (this->get_component_state() & COMPONENT_STATE_LOOP) {
      ESP_LOGE(TAG, "%s(0x%02X) device was not reset", __func__, index);
      return;
    }
  }

  this->enable();
  this->write_byte(index);
  this->disable();

  ESP_LOGV(TAG, "%s(0x%02X)", __func__, index);
}

void CC1101Component::write_(Register reg) {
  uint8_t index = (uint8_t) reg;
  if (reg > Register::TEST0 || reg == Register::FSTEST || reg == Register::AGCTEST) {
    ESP_LOGE(TAG, "%s(0x%02X) invalid register address", __func__, index);
    return;
  }

  if (!this->reset_) {
    if (this->get_component_state() & COMPONENT_STATE_LOOP) {
      ESP_LOGE(TAG, "%s(0x%02X) device was not reset", __func__, index);
      return;
    }
  }

  uint8_t value = this->regs_[index];

  this->enable();
  this->write_byte(index);
  this->transfer_array(&value, 1);
  this->disable();

  ESP_LOGV(TAG, "%s(0x%02X) = 0x%02X", __func__, index, this->regs_[index]);
}

void CC1101Component::write_(Register reg, uint8_t value) {
  uint8_t index = (uint8_t) reg;
  if (reg > Register::TEST0 || reg == Register::FSTEST || reg == Register::AGCTEST) {
    ESP_LOGE(TAG, "%s(0x%02X) invalid register address", __func__, index);
    return;
  }

  this->regs_[index] = value;
  this->write_(reg);
}

void CC1101Component::write_(Register reg, uint8_t *buffer, size_t length) {
  uint8_t index = (uint8_t) reg;
  if (reg != Register::PATABLE && reg != Register::FIFO) {
    ESP_LOGE(TAG, "%s(0x%02X) invalid register address", __func__, index);
    return;
  }

  if (!this->reset_) {
    if (this->get_component_state() & COMPONENT_STATE_LOOP) {
      ESP_LOGE(TAG, "%s(0x%02X) device was not reset", __func__, index);
      return;
    }
  }

  this->enable();
  this->write_byte(index | BUS_WRITE | BUS_BURST);
  this->transfer_array(buffer, length);
  this->disable();

  ESP_LOGV(TAG, "%s(0x%02X) %zu", __func__, index, length);
}

bool CC1101Component::read_(Register reg) {
  uint8_t index = (uint8_t) reg;
  if (reg > Register::RCCTRL0_STATUS) {
    ESP_LOGE(TAG, "%s(0x%02X) invalid register address", __func__, index);
    return false;
  }

  this->enable();
  this->write_byte(index | BUS_READ | BUS_BURST);
  this->regs_[index] = this->transfer_byte(0);
  this->disable();

  return true;
}

bool CC1101Component::read_(Register reg, uint8_t *buffer, size_t length) {
  uint8_t index = (uint8_t) reg;
  if (reg != Register::PATABLE && reg != Register::FIFO) {
    ESP_LOGE(TAG, "%s(0x%02X) invalid register address", __func__, index);
    return false;
  }

  this->enable();
  this->write_byte(index | BUS_READ | BUS_BURST);
  this->read_array(buffer, length);
  this->disable();

  return true;
}

void CC1101Component::send_(Command cmd) {
  if (cmd == Command::TX || cmd == Command::RX || cmd == Command::PWD) {
    this->send_(Command::IDLE);
  }

  ESP_LOGV(TAG, "%s(0x%02X)", __func__, (uint8_t) cmd);

  this->strobe_(cmd);
  this->wait_(cmd);

  if (cmd == Command::RES) {
    this->reset_ = true;
  }
}

bool CC1101Component::wait_(Command cmd) {
  uint32_t start = millis();
  while ((millis() - start) < 1000) {
    this->read_(Register::MARCSTATE);
    State s = (State) this->state_.MARC_STATE;
    if (cmd == Command::IDLE || cmd == Command::RES) {
      if (s == State::IDLE)
        return true;
    } else if (cmd == Command::RX) {
      if (s == State::RX || s == State::RX_END || s == State::RXTX_SWITCH)
        return true;
    } else if (cmd == Command::TX) {
      if (s == State::TX || s == State::TX_END || s == State::TXRX_SWITCH)
        return true;
    } else {
      return true;  // else if TODO
    }
    delayMicroseconds(1);
  }
  mark_failed();
  ESP_LOGE(TAG, "CC1101 modem wait state timeout. Check connection.");
  return false;
}

// config

template<typename T> T GET_ENUM_LAST(T value) { return T::LAST; }

#define CHECK_ENUM(value) \
  if ((value) >= GET_ENUM_LAST(value)) { \
    ESP_LOGE(TAG, "%s(%d) invalid", __func__, (int) (value)); \
    return; \
  }

#define CHECK_FLOAT_RANGE(value, min_value, max_value) \
  if ((value) < (min_value) || (value) > (max_value)) { \
    ESP_LOGE(TAG, "%s(%.2f) invalid (%.2f - %.2f)", __func__, value, min_value, max_value); \
    return; \
  }

#define CHECK_INT_RANGE(value, min_value, max_value) \
  if ((value) < (min_value) || (value) > (max_value)) { \
    ESP_LOGE(TAG, "%s(%d) invalid (%d - %d)", __func__, (int) (value), (int) (min_value), (int) (max_value)); \
    return; \
  }

#define CHECK_TEXT_RANGE(value, max_size) \
  if ((value).size() > (max_size)) { \
    ESP_LOGW(TAG, "%s(%s) trimmed (max %d characters)", __func__, (value).c_str(), max_size); \
    (value).resize(max_size); \
  }

/*
static int mapint(int x, int in_min, int in_max, int out_min, int out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static int32_t mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
*/

static void split_float(float value, int mbits, uint8_t &e, uint32_t &m) {
  if (value < 0) {
    ESP_LOGE(TAG, "split_float(%f, %d): positive values only", value, mbits);
  }

  int e_tmp;
  float m_tmp = std::frexp(value, &e_tmp);

  if (e_tmp <= mbits) {
    ESP_LOGW(TAG, "split_float(%f, %d): exponent would be negative, set to minimum", value, mbits);
    e = 0;
    m = 0;
    return;
  }

  // frexp returns 2^e * 0.m
  // but we want the original 2^e * 1.m format with the hidden bit

  e = (uint8_t) (e_tmp - mbits - 1);
  m = (uint32_t) ((m_tmp * 2 - 1) * (1 << (mbits + 1)) + 1) >> 1;

  // mantissa might overflow due to rounding, even the datasheet recommends this:
  // "If DRATE_M is rounded to the nearest integer and becomes 256,
  //  increment DRATE_E and use DRATE_M = 0."

  if (m == (1 << mbits)) {
    e = e + 1;
    m = 0;
  }
}

void CC1101Component::set_output_power(float value) {
  CHECK_FLOAT_RANGE(value, OUTPUT_POWER_MIN, OUTPUT_POWER_MAX)

  this->output_power_requested_ = value;

  int freq = (int) this->get_tuner_frequency();
  Modulation modulation = this->get_tuner_modulation();
  uint8_t a = 0xC0;

  if (freq >= 300000 && freq <= 348000) {
    a = PowerTable::find(PA_TABLE_315, sizeof(PA_TABLE_315) / sizeof(PA_TABLE_315[0]), value);
  } else if (freq >= 378000 && freq <= 464000) {
    a = PowerTable::find(PA_TABLE_433, sizeof(PA_TABLE_433) / sizeof(PA_TABLE_433[0]), value);
  } else if (freq >= 779000 && freq < 900000) {
    a = PowerTable::find(PA_TABLE_868, sizeof(PA_TABLE_868) / sizeof(PA_TABLE_868[0]), value);
  } else if (freq >= 900000 && freq <= 928000) {
    a = PowerTable::find(PA_TABLE_915, sizeof(PA_TABLE_915) / sizeof(PA_TABLE_915[0]), value);
  } else {
    ESP_LOGE(TAG, "frequency out of range: %d", freq);
  }

  if (modulation == Modulation::MODULATION_ASK_OOK) {
    this->pa_table_[0] = 0;
    this->pa_table_[1] = a;
  } else {
    this->pa_table_[0] = a;
    this->pa_table_[1] = 0;
  }

  this->output_power_effective_ = value;

  ESP_LOGD(TAG, "set_output_power(%.1f) %d", value, a);

  this->publish_output_power();

  if (!this->reset_) {
    return;
  }

  this->write_(Register::PATABLE, this->pa_table_, sizeof(this->pa_table_));
}

float CC1101Component::get_output_power() { return this->output_power_effective_; }

void CC1101Component::set_rx_attenuation(RxAttenuation value) {
  CHECK_ENUM(value)

  this->state_.CLOSE_IN_RX = (uint8_t) value;

  ESP_LOGD(TAG, "set_rx_attenuation(%d)", (int) value);

  this->publish_rx_attenuation();

  if (!this->reset_) {
    return;
  }

  this->write_(Register::FIFOTHR);
}

RxAttenuation CC1101Component::get_rx_attenuation() { return (RxAttenuation) this->state_.CLOSE_IN_RX; }

void CC1101Component::set_dc_blocking_filter(bool value) {
  this->state_.DEM_DCFILT_OFF = value ? 0 : 1;  // inverted logic, 0 is Enable

  ESP_LOGD(TAG, "set_dc_blocking_filter(%d)", value ? 1 : 0);

  this->publish_dc_blocking_filter();

  if (!this->reset_) {
    return;
  }

  this->write_(Register::MDMCFG2);
}

bool CC1101Component::get_dc_blocking_filter() { return this->state_.DEM_DCFILT_OFF == 0; }

// tuner_*

void CC1101Component::set_tuner_frequency(float value) {
  CHECK_FLOAT_RANGE(value, FREQUENCY_MIN, FREQUENCY_MAX)

  // strange frequencies like 360MHz also seem to work, not sure about the quality

  int freq = (int) (value * (1 << 16) / XTAL_FREQUENCY);

  this->state_.FREQ2 = (uint8_t) (freq >> 16);
  this->state_.FREQ1 = (uint8_t) (freq >> 8);
  this->state_.FREQ0 = (uint8_t) freq;

  this->set_output_power(this->output_power_requested_);  // frequency and modulation dependent

  ESP_LOGD(TAG, "set_tuner_frequency(%f) FREQ = %02X%02X%02X", value, this->state_.FREQ2, this->state_.FREQ1,
           this->state_.FREQ0);

  this->publish_tuner_frequency();

  if (!this->reset_) {
    return;
  }

  // If any frequency programming register is altered when the frequency synthesizer is
  // running, the synthesizer may give an undesired response. Hence, the frequency
  // programming should only be updated when the radio is in the IDLE state.

  // TODO: this may also apply when setting other registers
  // channel (CHANNR.CHAN)
  // channel_spacing (MDMCFG1.CHANSPC_*)
  // if_frequency (FSCTRL1.FREQ_IF)

  this->send_(Command::IDLE);

  this->write_(Register::FREQ2);
  this->write_(Register::FREQ1);
  this->write_(Register::FREQ0);

  // calibration
  /*
    if (freq >= 300000 && freq <= 348000) {
      this->write_(Register::FSCTRL0, (uint8_t) mapint(freq, 300000, 348000, 24, 28));

      if (freq < 322880) {
        this->write_(Register::TEST0, 0x0B);
      } else {
        this->write_(Register::TEST0, 0x09);
        this->read_(Register::FSCAL2);
        if(this->state_.VCO_CORE_H_EN == 0) {
          this->state_.VCO_CORE_H_EN = 1;
          this->write_(Register::FSCAL2);
        }
      }
    } else if (freq >= 378000 && freq <= 464000) {
      this->write_(Register::FSCTRL0, (uint8_t) mapint(freq, 378000, 464000, 31, 38));

      if (freq < 430500) {
        this->write_(Register::TEST0, 0x0B);
      } else {
        this->write_(Register::TEST0, 0x09);
        this->read_(Register::FSCAL2);
        if(this->state_.VCO_CORE_H_EN == 0) {
          this->state_.VCO_CORE_H_EN = 1;
          this->write_(Register::FSCAL2);
        }
      }
    } else if (freq >= 779000 && freq < 900000) {
      this->write_(Register::FSCTRL0, (uint8_t) mapint(freq, 779000, 899000, 65, 76));

      if (freq < 861000) {
        this->write_(Register::TEST0, 0x0B);
      } else {
        this->write_(Register::TEST0, 0x09);
        this->read_(Register::FSCAL2);
        if(this->state_.VCO_CORE_H_EN == 0) {
          this->state_.VCO_CORE_H_EN = 1;
          this->write_(Register::FSCAL2);
        }
      }
    } else if (freq >= 900000 && freq <= 928000) {
      this->write_(Register::FSCTRL0, (uint8_t) mapint(freq, 900000, 928000, 77, 79));
      //if (freq < ) {
      //  this->write_(Register::TEST0, 0x0B);
      //} else {
      //  this->write_(Register::TEST0, 0x09);
        this->read_(Register::FSCAL2);
        if(this->state_.VCO_CORE_H_EN == 0) {
          this->state_.VCO_CORE_H_EN = 1;
          this->write_(Register::FSCAL2);
        }
      //}
    }
  */

  this->send_(Command::RX);
}

float CC1101Component::get_tuner_frequency() {
  uint32_t freq = 0;
  freq |= (uint32_t) this->state_.FREQ2 << 16;
  freq |= (uint32_t) this->state_.FREQ1 << 8;
  freq |= (uint32_t) this->state_.FREQ0;
  return roundf((float) XTAL_FREQUENCY * freq / (1 << 16));
}

void CC1101Component::set_tuner_if_frequency(float value) {
  CHECK_FLOAT_RANGE(value, IF_FREQUENCY_MIN, IF_FREQUENCY_MAX)

  this->state_.FREQ_IF = value * (1 << 10) / XTAL_FREQUENCY;

  ESP_LOGD(TAG, "set_tuner_if_frequency(%f)", value);

  this->publish_tuner_if_frequency();

  if (!this->reset_) {
    return;
  }

  this->write_(Register::FSCTRL1);
}

float CC1101Component::get_tuner_if_frequency() { return XTAL_FREQUENCY * this->state_.FREQ_IF / (1 << 10); }

void CC1101Component::set_tuner_bandwidth(float value) {
  CHECK_FLOAT_RANGE(value, BANDWIDTH_MIN, BANDWIDTH_MAX)

  uint8_t e;
  uint32_t m;
  split_float(XTAL_FREQUENCY / (value * 8), 2, e, m);
  this->state_.CHANBW_E = (uint8_t) e;
  this->state_.CHANBW_M = (uint8_t) m;

  ESP_LOGD(TAG, "set_tuner_bandwidth(%f) CHANBW_E = %d CHANBW_M = %d", value, this->state_.CHANBW_E,
           this->state_.CHANBW_M);

  this->publish_tuner_bandwidth();

  if (!this->reset_) {
    return;
  }

  this->write_(Register::MDMCFG4);
}

float CC1101Component::get_tuner_bandwidth() {
  float chanbw = (float) (4 + this->state_.CHANBW_M) * (1 << this->state_.CHANBW_E);
  return roundf(XTAL_FREQUENCY / (8 * chanbw));
}

void CC1101Component::set_tuner_channel(uint8_t value) {
  this->state_.CHANNR = (uint8_t) value;

  ESP_LOGD(TAG, "set_tuner_channel(%d)", (int) value);

  this->publish_tuner_channel();

  if (!this->reset_) {
    return;
  }

  this->write_(Register::CHANNR);
}

uint8_t CC1101Component::get_tuner_channel() { return (uint8_t) this->state_.CHANNR; }

void CC1101Component::set_tuner_channel_spacing(float value) {
  CHECK_FLOAT_RANGE(value, CHANNEL_SPACING_MIN, CHANNEL_SPACING_MAX)

  uint8_t e;
  uint32_t m;
  split_float(value * (1 << 18) / XTAL_FREQUENCY, 8, e, m);
  this->state_.CHANSPC_E = (uint8_t) e;
  this->state_.CHANSPC_M = (uint8_t) m;

  ESP_LOGD(TAG, "set_tuner_channel_spacing(%f) CHANSPC_E = %d CHANSPC_M = %d", value, this->state_.CHANSPC_E,
           this->state_.CHANSPC_M);

  this->publish_tuner_channel_spacing();

  if (!this->reset_) {
    return;
  }

  this->write_(Register::MDMCFG1);
  this->write_(Register::MDMCFG0);
}

float CC1101Component::get_tuner_channel_spacing() {
  float chanspc = (float) (256 + this->state_.CHANSPC_M) * (1 << this->state_.CHANSPC_E);
  return XTAL_FREQUENCY * chanspc / (1 << 18);
}

void CC1101Component::set_tuner_fsk_deviation(float value) {
  CHECK_FLOAT_RANGE(value, FSK_DEVIATION_MIN, FSK_DEVIATION_MAX)

  uint8_t e;
  uint32_t m;
  split_float(value * (1 << 17) / XTAL_FREQUENCY, 3, e, m);
  this->state_.DEVIATION_E = (uint8_t) e;
  this->state_.DEVIATION_M = (uint8_t) m;

  ESP_LOGD(TAG, "set_tuner_fsk_deviation(%f) DEVIATION_E = %d DEVIATION_M = %d", value, this->state_.DEVIATION_E,
           this->state_.DEVIATION_M);

  this->publish_tuner_fsk_deviation();

  if (!this->reset_) {
    return;
  }

  this->write_(Register::DEVIATN);
}

float CC1101Component::get_tuner_fsk_deviation() {
  float deviation = (float) (8 + this->state_.DEVIATION_M) * (1 << this->state_.DEVIATION_E);
  return XTAL_FREQUENCY * deviation / (1 << 17);
}

void CC1101Component::set_tuner_msk_deviation(uint8_t value) {
  CHECK_INT_RANGE(value, MSK_DEVIATION_MIN, MSK_DEVIATION_MAX)

  this->state_.DEVIATION_E = 0;
  this->state_.DEVIATION_M = value - 1;  // Specifies the fraction of symbol period (1/8-8/8)

  ESP_LOGD(TAG, "set_tuner_msk_deviation(%d)", value);

  this->publish_tuner_msk_deviation();

  if (!this->reset_) {
    return;
  }

  this->write_(Register::DEVIATN);
}

uint8_t CC1101Component::get_tuner_msk_deviation() { return this->state_.DEVIATION_M + 1; }

void CC1101Component::set_tuner_symbol_rate(float value) {
  CHECK_FLOAT_RANGE(value, SYMBOL_RATE_MIN, SYMBOL_RATE_MAX)

  uint8_t e;
  uint32_t m;
  split_float(value * (1 << 28) / (XTAL_FREQUENCY * 1000), 8, e, m);
  this->state_.DRATE_E = (uint8_t) e;
  this->state_.DRATE_M = (uint8_t) m;

  ESP_LOGD(TAG, "set_tuner_symbol_rate(%f) CHANBW_E = %d CHANBW_M = %d", value, this->state_.DRATE_E,
           this->state_.DRATE_M);

  this->publish_tuner_symbol_rate();

  if (!this->reset_) {
    return;
  }

  this->write_(Register::MDMCFG4);
  this->write_(Register::MDMCFG3);
}

float CC1101Component::get_tuner_symbol_rate() {
  float drate = (float) (256 + this->state_.DRATE_M) * (1 << this->state_.DRATE_E);
  return (XTAL_FREQUENCY * 1000) * drate / (1 << 28);
}

void CC1101Component::set_tuner_sync_mode(SyncMode value) {
  CHECK_ENUM(value)

  this->state_.SYNC_MODE = (uint8_t) value;

  ESP_LOGD(TAG, "set_tuner_sync_mode(%d)", (int) value);

  this->publish_tuner_sync_mode();

  if (!this->reset_) {
    return;
  }

  this->write_(Register::MDMCFG2);
}

SyncMode CC1101Component::get_tuner_sync_mode() { return (SyncMode) this->state_.SYNC_MODE; }

void CC1101Component::set_tuner_carrier_sense_above_threshold(bool value) {
  this->state_.CARRIER_SENSE_ABOVE_THRESHOLD = value ? 1 : 0;

  ESP_LOGD(TAG, "set_tuner_carrier_sense_above_threshold(%d)", value ? 1 : 0);

  this->publish_tuner_carrier_sense_above_threshold();

  if (!this->reset_) {
    return;
  }

  this->write_(Register::MDMCFG2);
}

bool CC1101Component::get_tuner_carrier_sense_above_threshold() {
  return this->state_.CARRIER_SENSE_ABOVE_THRESHOLD == 1;
}

void CC1101Component::set_tuner_modulation(Modulation value) {
  CHECK_ENUM(value)

  this->state_.MOD_FORMAT = (uint8_t) value;
  // In OOK/ASK mode, this selects the PATABLE index to use when transmitting a ‘1’.
  // PATABLE index zero is used in OOK/ASK when transmitting a ‘0’.
  this->state_.PA_POWER = value == Modulation::MODULATION_ASK_OOK ? 1 : 0;

  this->set_output_power(this->output_power_requested_);  // frequency and modulation dependent

  ESP_LOGD(TAG, "set_tuner_modulation(%d)", (int) value);

  this->publish_tuner_modulation();

  if (!this->reset_) {
    return;
  }

  this->write_(Register::MDMCFG2);
  this->write_(Register::FREND0);
}

Modulation CC1101Component::get_tuner_modulation() { return (Modulation) this->state_.MOD_FORMAT; }

// agc_*

void CC1101Component::set_agc_magn_target(MagnTarget value) {
  CHECK_ENUM(value)

  this->state_.MAGN_TARGET = (uint8_t) value;

  ESP_LOGD(TAG, "set_agc_magn_target(%d)", (int) value);

  this->publish_agc_magn_target();

  if (!this->reset_) {
    return;
  }

  this->write_(Register::AGCCTRL2);
}

MagnTarget CC1101Component::get_agc_magn_target() { return (MagnTarget) this->state_.MAGN_TARGET; }

void CC1101Component::set_agc_max_lna_gain(MaxLnaGain value) {
  CHECK_ENUM(value)

  this->state_.MAX_LNA_GAIN = (uint8_t) value;

  ESP_LOGD(TAG, "set_agc_max_lna_gain(%d)", (int) value);

  this->publish_agc_max_lna_gain();

  if (!this->reset_) {
    return;
  }

  this->write_(Register::AGCCTRL2);
}

MaxLnaGain CC1101Component::get_agc_max_lna_gain() { return (MaxLnaGain) this->state_.MAX_LNA_GAIN; }

void CC1101Component::set_agc_max_dvga_gain(MaxDvgaGain value) {
  CHECK_ENUM(value)

  this->state_.MAX_DVGA_GAIN = (uint8_t) value;

  ESP_LOGD(TAG, "set_agc_max_dvga_gain(%d)", (int) value);

  this->publish_agc_max_dvga_gain();

  if (!this->reset_) {
    return;
  }

  this->write_(Register::AGCCTRL2);
}

MaxDvgaGain CC1101Component::get_agc_max_dvga_gain() { return (MaxDvgaGain) this->state_.MAX_DVGA_GAIN; }

void CC1101Component::set_agc_carrier_sense_abs_thr(int8_t value) {
  CHECK_INT_RANGE(value, CARRIER_SENSE_ABS_THR_MIN, CARRIER_SENSE_ABS_THR_MAX)

  this->state_.CARRIER_SENSE_ABS_THR = (uint8_t) (value & 0b1111);

  ESP_LOGD(TAG, "set_agc_carrier_sense_abs_thr(%d)", (int) value);

  this->publish_agc_carrier_sense_abs_thr();

  if (!this->reset_) {
    return;
  }

  this->write_(Register::AGCCTRL1);
}

int8_t CC1101Component::get_agc_carrier_sense_abs_thr() {
  return (int8_t) (this->state_.CARRIER_SENSE_ABS_THR << 4) >> 4;
}

void CC1101Component::set_agc_carrier_sense_rel_thr(CarrierSenseRelThr value) {
  CHECK_ENUM(value)

  this->state_.CARRIER_SENSE_REL_THR = (uint8_t) value;

  ESP_LOGD(TAG, "set_agc_agc_carrier_sense_rel_thr(%d)", (int) value);

  this->publish_agc_carrier_sense_rel_thr();

  if (!this->reset_) {
    return;
  }

  this->write_(Register::AGCCTRL1);
}

CarrierSenseRelThr CC1101Component::get_agc_carrier_sense_rel_thr() {
  return (CarrierSenseRelThr) this->state_.CARRIER_SENSE_REL_THR;
}

void CC1101Component::set_agc_lna_priority(bool value) {
  this->state_.AGC_LNA_PRIORITY = value ? 1 : 0;

  ESP_LOGD(TAG, "set_agc_lna_priority(%d)", value ? 1 : 0);

  this->publish_agc_lna_priority();

  if (!this->reset_) {
    return;
  }

  this->write_(Register::AGCCTRL1);
}

bool CC1101Component::get_agc_lna_priority() { return this->state_.AGC_LNA_PRIORITY == 1; }

void CC1101Component::set_agc_filter_length_fsk_msk(FilterLengthFskMsk value) {
  CHECK_ENUM(value)

  this->state_.FILTER_LENGTH = (uint8_t) value;

  ESP_LOGD(TAG, "set_agc_filter_length_fsk_msk(%d)", (int) value);

  this->publish_agc_filter_length_fsk_msk();
  this->publish_agc_filter_length_ask_ook();

  if (!this->reset_) {
    return;
  }

  this->write_(Register::AGCCTRL0);
}

FilterLengthFskMsk CC1101Component::get_agc_filter_length_fsk_msk() {
  return (FilterLengthFskMsk) this->state_.FILTER_LENGTH;
}

void CC1101Component::set_agc_filter_length_ask_ook(FilterLengthAskOok value) {
  CHECK_ENUM(value)

  this->state_.FILTER_LENGTH = (uint8_t) value;

  ESP_LOGD(TAG, "set_agc_filter_length_ask_ook(%d)", (int) value);

  this->publish_agc_filter_length_ask_ook();
  this->publish_agc_filter_length_fsk_msk();

  if (!this->reset_) {
    return;
  }

  this->write_(Register::AGCCTRL0);
}

FilterLengthAskOok CC1101Component::get_agc_filter_length_ask_ook() {
  return (FilterLengthAskOok) this->state_.FILTER_LENGTH;
}

void CC1101Component::set_agc_freeze(Freeze value) {
  CHECK_ENUM(value)

  this->state_.AGC_FREEZE = (uint8_t) value;

  ESP_LOGD(TAG, "set_agc_freeze(%d)", (int) value);

  this->publish_agc_freeze();

  if (!this->reset_) {
    return;
  }

  this->write_(Register::AGCCTRL0);
}

Freeze CC1101Component::get_agc_freeze() { return (Freeze) this->state_.AGC_FREEZE; }

void CC1101Component::set_agc_wait_time(WaitTime value) {
  CHECK_ENUM(value)

  this->state_.WAIT_TIME = (uint8_t) value;

  ESP_LOGD(TAG, "set_agc_wait_time(%d)", (int) value);

  this->publish_agc_wait_time();

  if (!this->reset_) {
    return;
  }

  this->write_(Register::AGCCTRL0);
}

WaitTime CC1101Component::get_agc_wait_time() { return (WaitTime) this->state_.WAIT_TIME; }

void CC1101Component::set_agc_hyst_level(HystLevel value) {
  CHECK_ENUM(value)

  this->state_.HYST_LEVEL = (uint8_t) value;

  ESP_LOGD(TAG, "set_agc_hyst_level(%d)", (int) value);

  this->publish_agc_hyst_level();

  if (!this->reset_) {
    return;
  }

  this->write_(Register::AGCCTRL0);
}

HystLevel CC1101Component::get_agc_hyst_level() { return (HystLevel) this->state_.HYST_LEVEL; }

// sensors

std::string CC1101Component::get_chip_id_text_sensor() { return this->chip_id_; }

float CC1101Component::get_rssi_sensor() { return (float) this->state_.RSSI / 2 - 74; }

float CC1101Component::get_lqi_sensor() { return (float) this->state_.LQI_EST; }

float CC1101Component::get_temperature_sensor() { return (float) 0; }  // TODO

template<class S, class T> void CC1101Component::publish_(S *s, T state) {
  if (s != nullptr) {
    if (!s->has_state() || s->state != state) {
      s->publish_state(state);
    }
  }
}

void CC1101Component::publish_switch_(switch_::Switch *s, bool state) {
  if (s != nullptr) {
    if (s->state != state) {  // ?
      s->publish_state(state);
    }
  }
}

void CC1101Component::publish_select_(select::Select *s, size_t index) {
  if (s != nullptr) {
    if (auto state = s->at(index)) {
      if (!s->has_state() || s->state != *state) {
        s->publish_state(*state);
      }
    }
  }
}

}  // namespace cc1101
}  // namespace esphome
