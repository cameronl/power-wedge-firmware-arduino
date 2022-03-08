#include "IS31FL3726A.h"

IS31FL3726A::IS31FL3726A(uint8_t serial, uint8_t enable, uint8_t latch, uint8_t clock)
{
  init(serial, enable, latch, clock);
}

void IS31FL3726A::init(uint8_t serial, uint8_t enable, uint8_t latch, uint8_t clock)
{
  _serial_pin = serial;
  _enable_pin = enable;
  _latch_pin = latch;
  _clock_pin = clock;
  
  begin();
}

void IS31FL3726A::begin() {
  pinMode(_serial_pin, OUTPUT);
  pinMode(_enable_pin, OUTPUT);
  pinMode(_latch_pin, OUTPUT);
  pinMode(_clock_pin, OUTPUT);
  // Start OFF
  digitalWrite(_clock_pin, LOW);
  digitalWrite(_latch_pin, LOW);
  digitalWrite(_enable_pin, HIGH);
}

/// Shift out 16 bits
void IS31FL3726A::set(uint16_t outs) {
  pinMode(_serial_pin, OUTPUT);
  pinMode(_enable_pin, OUTPUT);
  pinMode(_latch_pin, OUTPUT);
  pinMode(_clock_pin, OUTPUT);

  // Setup device for input
  digitalWrite(_enable_pin, HIGH);
  digitalWrite(_latch_pin, LOW);

  // Shift out the data bits
  for (int i=0; i<16; ++i) {
    digitalWrite(_clock_pin, LOW);
    delayMicroseconds(1);
    digitalWrite(_serial_pin, (outs >> i) & 0x1);
    delayMicroseconds(1);
    digitalWrite(_clock_pin, HIGH);
    delayMicroseconds(1);
  }
  digitalWrite(_clock_pin, LOW);
  delayMicroseconds(1);

  // Latch pulse
  digitalWrite(_latch_pin, HIGH);
  delayMicroseconds(1);
  digitalWrite(_latch_pin, LOW);
  delayMicroseconds(1);

  // Set enable
  digitalWrite(_enable_pin, _displayOn ? LOW : HIGH);
}

void IS31FL3726A::displayOn(bool enable) {
  if (enable) {
    displayOn();
  } else {
    displayOff();
  }
}

void IS31FL3726A::displayOn() {
  digitalWrite(_enable_pin, LOW);
  _displayOn = true;
}

void IS31FL3726A::displayOff() {
  digitalWrite(_enable_pin, HIGH);
  _displayOn = false;
}

bool IS31FL3726A::isDisplayOn() {
  return _displayOn;
}

