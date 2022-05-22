#ifndef UI7SEG_REV1B_H
#define UI7SEG_REV1B_H

// UI board: power-wedge-ui-leds_rev1b
// 16 outputs

// Character to segment outputs

// ############# ::1L_EFA____BGCDR2
#define UICHAR_L 0b0001100000000100 // Character L
#define UICHAR_P 0b0001110000110000 // Character P
#define UICHAR_F 0b0001110000010000 // Character F
#define UICHAR_E 0b0001110000010100 // Character E
#define UICHAR_D 0b0001000000111100 // Character d
#define UICHAR_C 0b0001110000000100 // Character C
#define UICHAR_B 0b0001100000011100 // Character b
#define UICHAR_A 0b0001110000111000 // Character A
#define UICHAR_9 0b0000110000111100
#define UICHAR_8 0b0001110000111100
#define UICHAR_7 0b0000010000101000
#define UICHAR_6 0b0001110000011100
#define UICHAR_5 0b0000110000011100
#define UICHAR_4 0b0000100000111000
#define UICHAR_3 0b0000010000111100
#define UICHAR_2 0b0001010000110100
#define UICHAR_1 0b0000000000101000
#define UICHAR_0 0b0001110000101100
#define UICHAR__ 0b0000000000000100 // _

// ############ ::1L_EFA____BGCDR2
#define UI_NONE 0b0000000000000000 // Blank: all segments off
#define UI_LED1 0b1000000000000000 // LED1
#define UI_LED2 0b0000000000000001 // LED2
#define UI_LDP  0b0100000000000000 // Left  decimal point
#define UI_RDP  0b0000000000000010 // Right decimal point

#endif
