#ifndef BOARD__UNO_1602_LCD_KEYPAD_SHIELD_REV1
#define BOARD__UNO_1602_LCD_KEYPAD_SHIELD_REV1

// Control Board (PCB): power-wedge-control_rev1

// VIN or VREF (to angle sensor and ADC) affects analog read
// ADC resolution (number of bits) sets max analog read value

#define readAngleSensorV1()  (double) analogRead(A2) / 1023.0 * 5.0
#define readAngleSensorV2()  (double) analogRead(A3) / 1023.0 * 5.0

#define RELAY_UP_PIN   3
#define RELAY_DN_PIN   2

#define EN_MANUAL_PIN A1
#define BTN_DN_PIN    A4
#define BTN_UP_PIN    A5

// 7-segment UI panel
#define UI_SERIAL_PIN  6  // shared with LCD D2
#define UI_ENABLE_PIN  1  // shared with TX
#define UI_LATCH_PIN   0  // shared with RX
#define UI_CLOCK_PIN   7  // shared with LCD D3

// 1602 LCD
#define LCD_RS_PIN  8
#define LCD_EN_PIN  9
#define LCD_D0_PIN  4
#define LCD_D1_PIN  5
#define LCD_D2_PIN  6
#define LCD_D3_PIN  7
// LCD backlight is on pin 10 and pulled up by the 1602 LCD shield

#define setpointEepromAddr  999     // Where to store selected set point. Read on reboot.
                                    // 0-1023 on Arduino Uno
#define useAngleSensorEepromAddr 998

#endif
