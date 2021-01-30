// Display Library for SPI e-paper panels from Dalian Good Display and boards from Waveshare.
// Requires HW SPI and Adafruit_GFX. Caution: these e-papers require 3.3V supply AND data lines!
//
// based on Demo Example from Good Display: http://www.e-paper-display.com/download_list/downloadcategoryid=34&isMode=false.html
//
// Author: Jean-Marc Zingg
//
// Version: see library.properties
//
// Library: https://github.com/ZinggJM/GxEPD2

#include "GxEPD2_EPD.h"

#include <pgmspace.h>

#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace gxepd2_epaper {

static const char *TAG = "gxepd2_epaper";

GxEPD2_EPD::GxEPD2_EPD(int8_t cs, int8_t dc, int8_t rst, int8_t busy, int8_t busy_level, uint32_t busy_timeout,
                       uint16_t w, uint16_t h, GxEPD2::Panel p, bool c, bool pu, bool fpu) :
  WIDTH(w), HEIGHT(h), panel(p), hasColor(c), hasPartialUpdate(pu), hasFastPartialUpdate(fpu),
  _cs(cs), _dc(dc), _rst(rst), _busy(busy), _busy_level(busy_level), _busy_timeout(busy_timeout), _diag_enabled(false)
{
  _initial_write = true;
  _initial_refresh = true;
  _power_is_on = false;
  _using_partial_mode = false;
  _hibernating = false;
  _reset_duration = 20;
  _reverse = (p == GxEPD2::GDE0213B1); // only GDE0213B1 is reversed
}

// esphome
void HOT GxEPD2_EPD::display(bool partial_update_mode) {
  ESP_LOGD(TAG, "Refreshing display");
  this->writeImage(this->buffer_,0,0,this->WIDTH,this->HEIGHT, false, this->_reverse);
  ESP_LOGD(TAG, "writeImage");
  this->refresh(partial_update_mode);
  ESP_LOGD(TAG, "refresh");
  if (this->hasFastPartialUpdate) {
    this->writeImageAgain(this->buffer_,0,0,this->WIDTH,this->HEIGHT, false, this->_reverse);
    ESP_LOGD(TAG, "writeImageAgain");
  }
  if (!partial_update_mode) {
    this->powerOff();
    ESP_LOGD(TAG, "powerOff");
  }
  ESP_LOGD(TAG, "Done refreshing display");
}

void GxEPD2_EPD::initialize() {
  this->init();
}

void GxEPD2_EPD::deep_sleep() {
  if (!this->manual_display_) {
    this->display(false);
  }
  this->hibernate();
}

const char* GxEPD2_EPD::model_name() {
  switch(this->panel) {
    case GxEPD2::GDEP015OC1 :
      return "GDEP0150C1";
    case GxEPD2::GDEH0154D67 :
      return "GDEH0154D67";
    case GxEPD2::GDEW0154T8 :
      return "GDEW0154T8";
    case GxEPD2::GDEW0154M09 :
      return "GDEW0154M09";
    case GxEPD2::GDEW0154M10 :
      return "GDEW0154M10";
    case GxEPD2::GDE0213B1 :
      return "GDE0213B1";
    case GxEPD2::GDEH0213B72 :
      return "GDEH0213B72";
    case GxEPD2::GDEH0213B73 :
      return "GDEH0213B73";
    case GxEPD2::GDEW0213I5F :
      return "GDEW0213I5F";
    case GxEPD2::GDEW026T0 :
      return "GDEW026T0";
    case GxEPD2::GDEH029A1 :
      return "GDEH029A1";
    case GxEPD2::GDEW029T5 :
      return "GDEW029T5";
    case GxEPD2::GDEW027W3 :
      return "GDEW027W3";
    case GxEPD2::GDEW0371W7 :
      return "GDEW0371W7";
    case GxEPD2::GDEW042T2 :
      return "GDEW042T2";
    case GxEPD2::GDEW0583T7 :
      return "GDEW0583T7";
    case GxEPD2::GDEW0583T8 :
      return "GDEW0583T8";
    case GxEPD2::GDEW075T8 :
      return "GDEW075T8";
    case GxEPD2::GDEW075T7 :
      return "GDEW075T7";
    case GxEPD2::GDEW1248T3 :
      return "GDEW1248T3";
    case GxEPD2::ED060SCT :
      return "ED060SCT";
    case GxEPD2::GDEW0154Z04 :
      return "GDEW0154Z04";
    case GxEPD2::GDEH0154Z90 :
      return "GDEH0154Z90";
    case GxEPD2::GDEW0213Z16 :
      return "GDEW0213Z16";
    case GxEPD2::GDEW029Z10 :
      return "GDEW029Z10";
    case GxEPD2::GDEW027C44 :
      return "GDEW027C44";
    case GxEPD2::GDEW042Z15 :
      return "GDEW042Z15";
    case GxEPD2::GDEW0583Z21 :
      return "GDEW0583Z21";
    case GxEPD2::ACeP565 :
      return "ACeP565";
    case GxEPD2::GDEW075Z09 :
      return "GDEW075Z09";
    case GxEPD2::GDEW075Z08 :
      return "GDEW075Z08";
    case GxEPD2::GDEH075Z90 :
      return "GDEH075Z90";
    default :
      return "UNKNOWN";
  }
}
void GxEPD2_EPD::dump_config() {
  LOG_DISPLAY("", "GxEPD2 (Waveshare) E-Paper", this);
  ESP_LOGCONFIG(TAG, "  Model: %s", this->model_name());
  LOG_PIN("  Reset Pin: ", this->reset_pin_);
  LOG_PIN("  DC Pin: ", this->dc_pin_);
  LOG_PIN("  Busy Pin: ", this->busy_pin_);
  LOG_UPDATE_INTERVAL(this);
}

void GxEPD2_EPD::update() {
  bool partial_update_mode = false;
  this->do_update_();
  if (!this->manual_display_) {
    if (this->full_update_every_ >= 2) {
      partial_update_mode = (this->at_update_ != 0);
      this->at_update_ = (this->at_update_ + 1) % this->full_update_every_;
    }
    ESP_LOGD(TAG, "%s update (full update every %d, at update %d)",
      partial_update_mode ? "Partial" : "Full",
      this->full_update_every_, this->at_update_);
    this->display(partial_update_mode);
    this->hibernate();
  } else {
    ESP_LOGD(TAG, "Manual update");
  }
}

void GxEPD2_EPD::next_update_full(bool full) {
  this->at_update_ = (full ? 0 : 1);
}

void GxEPD2_EPD::fill(Color color) {
  const uint8_t fill = color.is_on() ? 0x00 : 0xFF;
  for (uint32_t i = 0; i < this->get_buffer_length_(); i++)
    this->buffer_[i] = fill;
}

void GxEPD2_EPD::setup() {
  this->init_internal_(this->get_buffer_length_());
  this->dc_pin_->setup();  // OUTPUT
  this->dc_pin_->digital_write(false);
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->setup();  // OUTPUT
    this->reset_pin_->digital_write(true);
  }
  if (this->busy_pin_ != nullptr) {
    this->busy_pin_->setup();  // INPUT
  }
  this->spi_setup();
}

void GxEPD2_EPD::on_safe_shutdown() {
  this->deep_sleep();
}

void GxEPD2_EPD::set_full_update_every(uint32_t full_update_every) {
  this->full_update_every_ = full_update_every;
}

void GxEPD2_EPD::set_manual_display(bool setting) {
  this->manual_display_ = setting;
}

void HOT GxEPD2_EPD::draw_absolute_pixel_internal(int x, int y, Color color) {
  if (x >= this->get_width_internal() || y >= this->get_height_internal() || x < 0 || y < 0)
    return;

  const uint32_t pos = (x + y * this->get_width_internal()) / 8u;
  const uint8_t subpos = x & 0x07;
  // flip logic
  if (!color.is_on())
    this->buffer_[pos] |= 0x80 >> subpos;
  else
    this->buffer_[pos] &= ~(0x80 >> subpos);
}

int GxEPD2_EPD::get_width_internal() {
  return (int) this->WIDTH;
}
int GxEPD2_EPD::get_height_internal() {
  return (int) this->HEIGHT;
}
uint32_t GxEPD2_EPD::get_buffer_length_() {
  return this->WIDTH * this->HEIGHT / 8u;
}
// esphome end


void GxEPD2_EPD::init(uint32_t serial_diag_bitrate)
{
  init(serial_diag_bitrate, true, 20, false);
}

void GxEPD2_EPD::init(uint32_t serial_diag_bitrate, bool initial, uint16_t reset_duration, bool pulldown_rst_mode)
{
  _initial_write = initial;
  _initial_refresh = initial;
  _pulldown_rst_mode = pulldown_rst_mode;
  _power_is_on = false;
  _using_partial_mode = false;
  _hibernating = false;
  _reset_duration = reset_duration;
  _reset();
}

void GxEPD2_EPD::_reset()
{
  if (_pulldown_rst_mode)
  {
    this->reset_pin_->digital_write(false);
    this->reset_pin_->pin_mode(OUTPUT);
    delay(_reset_duration);
    this->reset_pin_->pin_mode(INPUT_PULLUP);
    delay(200);
  }
  else
  {
    this->reset_pin_->digital_write(true);
    this->reset_pin_->pin_mode(OUTPUT);
    delay(20);
    this->reset_pin_->digital_write(false);
    delay(_reset_duration);
    this->reset_pin_->digital_write(true);
    delay(200);
  }
  _hibernating = false;
}

void GxEPD2_EPD::_waitWhileBusy(const char* comment, uint16_t busy_time)
{
  if (_busy >= 0)
  {
    delay(1); // add some margin to become active
    unsigned long start = micros();
    while (1)
    {
      if (this->busy_pin_->digital_read() != _busy_level) break;
      delay(1);
      if (micros() - start > _busy_timeout)
      {
        ESP_LOGW(TAG, "Busy Timeout!");
        break;
      }
    }
    if (comment)
    (void) start;
  }
  else delay(busy_time);
}

void GxEPD2_EPD::_writeCommand(uint8_t c)
{
  this->dc_pin_->digital_write(false);
  this->enable();
  this->write_byte(c);
  this->disable();
  this->dc_pin_->digital_write(true);
}

void GxEPD2_EPD::_writeData(uint8_t d)
{
  this->enable();
  this->write_byte(d);
  this->disable();
}

void GxEPD2_EPD::_writeData(const uint8_t* data, uint16_t n)
{
  this->enable();
  this->write_array(data, (size_t) n);
  this->disable();
}

void GxEPD2_EPD::_writeDataPGM(const uint8_t* data, uint16_t n, int16_t fill_with_zeroes)
{
  this->enable();
  for (uint16_t i = 0; i < n; i++)
  {
    this->write_byte(pgm_read_byte(&*data++));
  }
  while (fill_with_zeroes > 0)
  {
    this->write_byte(0x00);
    fill_with_zeroes--;
  }
  this->disable();
}

void GxEPD2_EPD::_writeDataPGM_sCS(const uint8_t* data, uint16_t n, int16_t fill_with_zeroes)
{
  for (uint8_t i = 0; i < n; i++)
  {
    this->enable();
    this->write_byte(pgm_read_byte(&*data++));
    this->disable();
  }
  while (fill_with_zeroes > 0)
  {
    this->enable();
    this->write_byte(0x00);
    fill_with_zeroes--;
    this->disable();
  }
}

void GxEPD2_EPD::_writeCommandData(const uint8_t* pCommandData, uint8_t datalen)
{
  this->dc_pin_->digital_write(false);
  this->enable();
  this->write_byte(*pCommandData++);
  this->dc_pin_->digital_write(true);
  for (uint8_t i = 0; i < datalen - 1; i++)  // sub the command
  {
    this->write_byte(*pCommandData++);
  }
  this->disable();
}

void GxEPD2_EPD::_writeCommandDataPGM(const uint8_t* pCommandData, uint8_t datalen)
{
  this->dc_pin_->digital_write(false);
  this->enable();
  this->write_byte(pgm_read_byte(&*pCommandData++));
  this->dc_pin_->digital_write(true);
  for (uint8_t i = 0; i < datalen - 1; i++)  // sub the command
  {
    this->write_byte(pgm_read_byte(&*pCommandData++));
  }
  this->disable();
}

void GxEPD2_EPD::_startTransfer()
{
  this->enable();
}

void GxEPD2_EPD::_transfer(uint8_t value)
{
  this->write_byte(value);
}

void GxEPD2_EPD::_endTransfer()
{
  this->disable();
}

}  // namespace gxepd2_epaper
}  // namespace esphome
