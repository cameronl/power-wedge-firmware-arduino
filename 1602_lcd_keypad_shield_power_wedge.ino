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

#define RELAY_UP_PIN  12
#define RELAY_DN_PIN  13

double tolerance = 1.0; // +/- angle (degrees)

// Additional angle (overshoot) due to relay turn off delay, etc.
double stopTimeUp = 0.7; // angle (degrees)
double stopTimeDn = 0.7; // angle (degrees)

// List of user selectable set points
double userSetpointLen = 8;
double userSetpoints[] = {
  90, 10, 6, 3, 0, -4, -8, -10  // angle (degrees)
};

// Error limits
unsigned long maxRelayOnTime = 10000; // milliseconds
unsigned long maxRelayCyclesPer  = 6;
unsigned long maxRelayCyclesTime = 2000; // milliseconds

#define setpointEepromAddr  999     // Where to store selected set point. Read on reboot.
                                    // 0-1023 on Arduino Uno

// ----------------------------------------------------------

// TODO: Store and reference double volts or int millivolts?

#include <EEPROM.h>
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

bool redraw = true;                 // should redraw the lcd screen
int screens = 3;
int screen = 0;

// For debug
unsigned long loopCount = 0;

// Possible error flags
#define ERR_BAD_SENSOR    0b10000000 // Angle sensor out of valid range. Check sensor connection, power.
#define ERR_RELAY_CYCLING 0b00000010 // Relay cycling ON/OFF too much. Control unstable around setpoint.
#define ERR_RELAY_ON_LONG 0b00000001 // Relay has been ON too long.

// For control
bool controlEnable = false;         // Enable automatic control
uint8_t errorFlags = 0;             // Errors which inhibit control
bool raising, lowering = false;

// For checking relay cycling too frequently or on too long
unsigned long lastRelayStart = 0;   // last time a relay turned ON
uint16_t upCount, dnCount = 0;      // how often are the relays turning ON?
uint16_t relayCount = 0;
uint16_t lastRelayCount = 0;
unsigned long relayCycleTimer = 0;  // last check of relay cycling

double setpoint, input, output;
double error;

int userSetpointIndex;              // Current position in list of user setpoints

void prevSetpoint() {
  --userSetpointIndex;
  if (userSetpointIndex < 0) {
    userSetpointIndex = 0; // Clamp to end -- NOT rollover
  }
  goToUserSetpoint(userSetpointIndex);
}

void nextSetpoint() {
  ++userSetpointIndex;
  if (userSetpointIndex > userSetpointLen - 1) {
    userSetpointIndex = userSetpointLen - 1; // Clamp to end -- NOT rollover
  }
  goToUserSetpoint(userSetpointIndex);
}

void goToUserSetpoint(int index) {
  // TODO Don't stop if we're already moving in the right direction.
  moveStop();
  setpoint = userSetpoints[index];
  // TODO Sanity check setpoint
  EEPROM.update(setpointEepromAddr, index & 0xFF);
}

// Initiate movement up
void moveRaise() {
  // Turn ON relay
  digitalWrite(RELAY_DN_PIN, LOW);
  digitalWrite(RELAY_UP_PIN, HIGH);
  raising = true;
  lowering = false;
  ++upCount;
  lastRelayStart = millis();
}

// Initiate movement down
void moveLower() {
  // Turn ON relay
  digitalWrite(RELAY_UP_PIN, LOW);
  digitalWrite(RELAY_DN_PIN, HIGH);
  raising = false;
  lowering = true;
  ++dnCount;
  lastRelayStart = millis();
}

void moveStop() {
  // Turn OFF relays
  digitalWrite(RELAY_UP_PIN, LOW);
  digitalWrite(RELAY_DN_PIN, LOW);
  raising = false;
  lowering = false;
}

void setup() {
  lcd.begin(16, 2); // start the library

  // Relay control pins
  digitalWrite(RELAY_UP_PIN, LOW);
  digitalWrite(RELAY_DN_PIN, LOW);
  pinMode(RELAY_UP_PIN, OUTPUT);
  pinMode(RELAY_DN_PIN, OUTPUT);
  // moveStop();

  // What if MCU reboots? Don't want setpoint to jump. Store/load it from flash.
  // Currently storing index in list of user setpoints -- NOT the actual angle/voltage.
  userSetpointIndex = EEPROM.read(setpointEepromAddr);
  if (userSetpointIndex > userSetpointLen - 1) {
    userSetpointIndex = 0; // Home is probably best place to start if uninitialized.
  }

  //setpoint = 3.500; // V
  goToUserSetpoint(userSetpointIndex);
  controlEnable = true;
}

void loop() {
  ++loopCount;
  
  // Read sensor voltage
  double v1 = readAngleSensorV1();
  double v2 = readAngleSensorV2();

  // TODO Handle noise

  // TODO If control by angle instead of voltage, include angle in check?
  // TODO Separate error flag for each angle sensor?
  // Sanity check input
  if (v1 < 0.50 || v1 > 4.50 || v2 < 0.50 || v2 > 4.50) {
    // Angle sensor out of valid range
    // Check sensor connection, power.
    moveStop();
    errorFlags |= ERR_BAD_SENSOR;      // Set error flag
    // Log this? Count? Sum time in error? Display message?
    // TODO: Should only call moveStop() if err flag not already set?
  } else {
    // Clear error automatically
    // TODO: Log this? Count? Sum time in error? Display message?
    errorFlags &= ~ERR_BAD_SENSOR;
  }

  // Convert voltage to calibrated angle
  double angle1 = interp(voltToAngle1, v1, numCalibPoints1);
  double angle2 = interp(voltToAngle2, v2, numCalibPoints2);

  // Convert angle to position for display

  // Choose one value from two sensors
  // TODO: What if both sensors don't agree?
  //input = (angle1 + angle2)/2.0;
  input = angle1;

  // Check relay ON too long
  if ((raising || lowering) && millis() - lastRelayStart > maxRelayOnTime) {
    moveStop();
    errorFlags |= ERR_RELAY_ON_LONG;   // Set error flag
    // TODO: When should we clear this error and try again?
  }

  // Check relay cycling on/off too much
  if (millis() - relayCycleTimer > maxRelayCyclesTime) {
    relayCycleTimer = millis();
    relayCount = upCount + dnCount;
    if (relayCount - lastRelayCount > maxRelayCyclesPer) {
      // Relay(s) cycling to frequently.
      // Control position not settling on setpoint?
      // Too much overshoot of setpoint on each movement?
      // Try increasing tolerance around setpoint.
      moveStop();
      errorFlags |= ERR_RELAY_CYCLING;   // Set error flag
      // TODO Any logging?
      //      Store relayCount diff that caused the error somewhere?
      // TODO When to reset/clear this error?
    }
    lastRelayCount = relayCount;
  }

  // -------------------------------------------------------------
  // Control
  error = input - setpoint;
  if (controlEnable && errorFlags == 0) {
    // Bang-bang control with hysteresis
    if (raising) {
      if (input < setpoint - stopTimeUp) {
        // Continue
      } else {
        // Stop at setpoint
        moveStop();
      }
    } else if (lowering) {
      if (input > setpoint + stopTimeDn) {
        // Continue
      } else {
        // Stop at setpoint
        moveStop();
      }
    } else {
      // Only move if error > tolerance
      if (abs(error) > tolerance) {
        if (error > 0) {
          moveLower();
        } else {
          moveRaise();
        }
      }
      // Do nothing. We're within tolerance of setpoint.
    }
  }
  // -------------------------------------------------------------

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
    } else if (key == KEY_LEFT) {
      --screen;
      if (screen < 0) {
        screen = screens - 1; // wrap around
      }
      lcd.clear();
      redraw = true;
    } else if (key == KEY_DOWN) {
      prevSetpoint();
    } else if (key == KEY_UP) {
      nextSetpoint();
    } else if (key == KEY_RIGHT) {
      ++screen;
      if (screen > screens - 1) {
        screen = 0; // wrap around
      }
      lcd.clear();
      redraw = true;
    } else {
      // Should never happen.
    }
  }

  if (redraw || millis() - tepTimer > 500) {
    // For debug, calc stepTime
    int stepTime = loopCount;
    loopCount = 0;
    
    redraw = false;
    tepTimer = millis();

    // Pop up errors -- separate from normal screens below.
    if (errorFlags != 0) {
      lcd.clear();
      lcd.setCursor(0, 0); // set the LCD cursor position
      // If we use the whole screen, only show 1 error
      // Could do 1 per line, etc..
      if ((errorFlags & ERR_BAD_SENSOR) > 0) {
        lcd.print("Err Bad Sensor.");
      } else if ((errorFlags & ERR_RELAY_CYCLING) > 0) {
        lcd.print("Err Relays");
        lcd.setCursor(0, 1); // set the LCD cursor position
        lcd.print("Cycling On/Off");
      } else if ((errorFlags & ERR_RELAY_ON_LONG) > 0) {
        lcd.print("Err Relay On");
        lcd.setCursor(0, 1); // set the LCD cursor position
        lcd.print("    Too Long");
      }
      return; // Don't draw screens below
    }

    if (screen == 0) {
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


      // Print relay counters
      lcd.setCursor(12, 0); // set the LCD cursor position
      // lcd.print("u:");
      if (abs(upCount) < 100) {
        lcd.print(' ');
      }
      if (abs(upCount) < 10) {
        lcd.print(' ');
      }
      if (upCount >= 0) {
        lcd.print(' ');
      }
      lcd.print(upCount);

      lcd.setCursor(12, 1); // set the LCD cursor position
      // lcd.print(" d:");
      if (abs(dnCount) < 100) {
        lcd.print(' ');
      }
      if (abs(dnCount) < 10) {
        lcd.print(' ');
      }
      if (dnCount >= 0) {
        lcd.print(' ');
      }
      lcd.print(dnCount);

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
    } else if (screen == 1) {
      // Print setpoint
      lcd.setCursor(0, 0); // set the LCD cursor position
      lcd.print("S");
      if (abs(setpoint) < 100) {
        lcd.print(' ');
      }
      if (abs(setpoint) < 10) {
        lcd.print(' ');
      }
      if (setpoint >= 0) {
       lcd.print(' ');
      }
      lcd.print(setpoint);
      // Print error
      // lcd.setCursor(0, 0); // set the LCD cursor position
      if (abs(error) < 100) {
        lcd.print(' ');
      }
      if (abs(error) < 10) {
        lcd.print(' ');
      }
      if (error >= 0) {
       lcd.print(' ');
      }
      lcd.print(error);
      lcd.print("E");
      // Print input
      lcd.setCursor(0, 1); // set the LCD cursor position
      lcd.print("I");
      if (abs(input) < 100) {
        lcd.print(' ');
      }
      if (abs(input) < 10) {
        lcd.print(' ');
      }
      if (input >= 0) {
       lcd.print(' ');
      }
      lcd.print(input);
      // Print tolerance
      // lcd.setCursor(0, 0); // set the LCD cursor position
      if (abs(tolerance) < 100) {
        lcd.print(' ');
      }
      if (abs(tolerance) < 10) {
        lcd.print(' ');
      }
      if (tolerance >= 0) {
       lcd.print(' ');
      }
      lcd.print(tolerance);
      lcd.print("T");
    } else if (screen == 2) {
      // Print stepTime
      lcd.setCursor(9, 0); // set the LCD cursor position
      if (abs(stepTime) < 100) {
        lcd.print(' ');
      }
      if (abs(stepTime) < 10) {
        lcd.print(' ');
      }
      //if (stepTime >= 0) {
      //  lcd.print(' ');
      //}
      lcd.print(stepTime);
      //lcd.print("ms");
      lcd.print(" stp");
    } else {
      lcd.setCursor(0, 0); // set the LCD cursor position
      lcd.print("Unknown screen.");
    }
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
