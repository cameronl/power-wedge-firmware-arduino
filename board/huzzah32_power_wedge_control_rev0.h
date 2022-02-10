#ifndef BOARD__HUZZAH32_POWER_WEDGE_CONTROL_REV0
#define BOARD__HUZZAH32_POWER_WEDGE_CONTROL_REV0

// VIN or VREF (to angle sensor and ADC) affects analog read
// ADC resolution (number of bits) sets max analog read value

// Resolution: 12bit (0 - 4095)
// TODO Check ADC voltage ref
#define readAngleSensorV1()  (double) analogRead(A2) / 4095.0 * 1.0
#define readAngleSensorV2()  (double) analogRead(A3) / 4095.0 * 1.0
// #define readAngleSensorV1()  (double) analogRead(A2) / 4095.0 * 3.3 * 2.0
// #define readAngleSensorV2()  (double) analogRead(A3) / 4095.0 * 3.3 * 2.0

#define RELAY_UP_PIN  32
#define RELAY_DN_PIN  33

#define SWIN_UP_PIN  14
#define SWIN_DN_PIN  15

#error "Eeprom not supported yet."

#endif
