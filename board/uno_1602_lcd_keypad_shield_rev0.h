#ifndef BOARD__UNO_1602_LCD_KEYPAD_SHIELD_REV0
#define BOARD__UNO_1602_LCD_KEYPAD_SHIELD_REV0

// Control Board (PCB): breadboard prototype

// VIN or VREF (to angle sensor and ADC) affects analog read
// ADC resolution (number of bits) sets max analog read value

#define readAngleSensorV1()  (double) analogRead(A4) / 1023.0 * 5.0
#define readAngleSensorV2()  (double) analogRead(A5) / 1023.0 * 5.0

#define RELAY_UP_PIN  11
#define RELAY_DN_PIN  12

#define controlEnableEepromAddr 998 // Where to store control enable. Read on reboot.
#define setpointEepromAddr  999     // Where to store selected set point. Read on reboot.
                                    // 0-1023 on Arduino Uno

#endif
