/**
 * Power wedge control using 1602 LCD keypad shield on Arduino Uno.
 */

#include "coord_interp.h"

// Calibration

// VIN or VREF (to angle sensor and ADC) affects analog read
// ADC resolution (number of bits) sets max analog read value
// Mounting angle offset to calibrated angle?

#define readAngleSensorV1()  (double) analogRead(A4) / 1023.0 * 5.0
#define readAngleSensorV2()  (double) analogRead(A5) / 1023.0 * 5.0

// Separate calibration for each angle sensor...

// Calibration: angle sensor 1 -- voltage to angle
int numCalibPoints1 = 8; // match next line
coord_t voltToAngle1[8] = 
{
    {0.0000, -15},
    {0.3533, -12},
    {0.5189, -11},
    {3.8300,   9},
    {4.0000,  10},
    {4.4480,  90},
    {4.5040, 100},
    {5.2880, 240}
};

// Calibration: angle sensor 2 -- voltage to angle
int numCalibPoints2 = 8; // match next line
coord_t voltToAngle2[8] = 
{
    {0.0000, -15},
    {0.4017, -12},
    {0.5644, -11},
    {3.8200,   9},
    {4.0000,  10},
    {4.4373,  90},
    {4.4920, 100},
    {5.2573, 240}
};


// ----------------------------------------------------------

// TODO: Store and reference double volts or int millivolts?

#include <LiquidCrystal.h>

LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // select the pins used on the LCD panel

unsigned long tepTimer ;

enum KEY {
  KEY_NONE = 0,
  KEY_SELECT,
  KEY_LEFT,
  KEY_DOWN,
  KEY_UP,
  KEY_RIGHT
};

KEY keyState = KEY_NONE;            // set every time a debounced key change occurs
KEY lastRealKey = KEY_NONE;         // set every time a key change is read (no software debounce)
// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastRealKeyTime = 0;  // last time key changed (for debounce)
unsigned long debounceDelay = 50;   // the debounce time; increase if the output flickers

int counter = 0;
int oldCounter = 0;                 // for determining dirty state needing redraw
bool redraw = true;                 // should redraw the lcd screen

void setup() {
  lcd.begin(16, 2); // start the library
}

void loop() {
  // Read sensor voltage
  double v1 = readAngleSensorV1();
  double v2 = readAngleSensorV2();

  // Convert voltage to calibrated angle
  double angle1 = interp(voltToAngle1, v1, numCalibPoints1);
  double angle2 = interp(voltToAngle2, v2, numCalibPoints2);

  // Convert angle to position for display

  // Choose one value from two sensors
  // TODO: What if both sensors don't agree?

  
  // Handle buttons
  KEY key = readKeyInput();

  // TODO: Only update on key change?
  // TODO: Wait for a key repeat or release timer?
  if (key != keyState) {
    // A change
    keyState = key;

    // Handle the change
    if (key == KEY_NONE) {
      // Do nothing
    } else if (key == KEY_SELECT) {
      counter = 0;
    } else if (key == KEY_LEFT) {
      counter = counter * 2;
    } else if (key == KEY_DOWN) {
      counter--;
    } else if (key == KEY_UP) {
      counter++;
    } else if (key == KEY_RIGHT) {
      counter = -counter;
    } else {
      // Should never happen.
    }

    if (counter != oldCounter) {
      redraw = true;
      oldCounter = counter;
    }
  }

  if (redraw || millis() - tepTimer > 500) {  // output a temperature value per 500ms
    redraw = false;
    tepTimer = millis();

    // Print V1, angle1
    lcd.setCursor(0, 0); // set the LCD cursor position
    lcd.print(v1);
    lcd.print("V ");

    if (abs(angle1) < 10) {
      lcd.print(' ');
    }
    if (angle1 >= 0) {
      lcd.print(' ');
    }
    lcd.print(angle1);


    // Print V2, angle2
    lcd.setCursor(0, 1); // set the LCD cursor position
    lcd.print(v2);
    lcd.print("V ");

    if (abs(angle2) < 10) {
      lcd.print(' ');
    }
    if (angle2 >= 0) {
      lcd.print(' ');
    }
    lcd.print(angle2);


//
//    // Print counter
//    lcd.setCursor(9, 0); // set the LCD cursor position
//    if (abs(counter) < 100) {
//      lcd.print(' ');
//    }
//    if (abs(counter) < 10) {
//      lcd.print(' ');
//    }
//    if (counter >= 0) {
//      lcd.print(' ');
//    }
//    lcd.print(counter);


    //  int k = analogRead(A0);            // read the analog in value:
    //  // Print k
    //  lcd.setCursor(7, 1); // set the LCD cursor position
    //  if (abs(k) > 9999) {
    //    lcd.print('_____');
    //  } else {
    //    if (abs(k) < 1000) {
    //      lcd.print(' ');
    //    }
    //    if (abs(k) < 100) {
    //      lcd.print(' ');
    //    }
    //    if (abs(k) < 10) {
    //      lcd.print(' ');
    //    }
    //    if (k >= 0) {
    //      lcd.print(' ');
    //    }
    //    lcd.print(k);
    //  }
  }
}

// Read keypad and handle button debounce
KEY readKeyInput() {
  KEY key;
  // Read key
  int k = analogRead(A0);            // read the analog in value:
  if (k > 900) {
    key = KEY_NONE;
  } else if (k > 700) {
    key = KEY_SELECT;
  } else if (k > 500) {
    key = KEY_LEFT;
  } else if (k > 300) {
    key = KEY_DOWN;
  } else if (k > 100) {
    key = KEY_UP;
  } else {
    key = KEY_RIGHT;
  }

  // Software debounce
  
  // If the switch changed, due to noise or pressing:
  if (key != lastRealKey) {
    // reset the debouncing timer
    lastRealKeyTime = millis();
    lastRealKey = key;
    // return KEY_NONE;
  }
  
  if ((millis() - lastRealKeyTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:
    return key;  
  }
  return KEY_NONE;
}
