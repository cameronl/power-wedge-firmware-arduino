#ifndef IS31FL3726A_h
#define IS31FL3726A_h

#include <inttypes.h>

/// LED Driver IC 16 Output Linear Shift Register
class IS31FL3726A {
public:
  IS31FL3726A(uint8_t serial, uint8_t enable, uint8_t latch, uint8_t clock);

  void init(uint8_t serial, uint8_t enable, uint8_t latch, uint8_t clock);

  void begin();

  void set(uint16_t outs);

  void displayOff();
  void displayOn();
  
private:
  uint8_t _serial_pin; // Serial data for data shift register.
  uint8_t _enable_pin; // Outputs enabled when LOW.
  uint8_t _latch_pin;  // LOW: Data strobe (not latched); HIGH: data is latched.
  uint8_t _clock_pin;  // Clock for data shift on rising edge.

  bool _displayOn = true;
};

#endif
