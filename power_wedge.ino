/**
 * Power wedge control using 1602 LCD keypad shield on Arduino Uno.
 * 
 * If control enabled,  UP/DOWN keys iterate user set points.
 * If control disabled, UP/DOWN keys are manual relay movement.
 */

#include "coord_interp.h"
#include "board/uno_1602_lcd_keypad_shield_rev1.h"
//#include "board/huzzah32_power_wedge_control_rev0.h"
#include "ui7seg_rev1b.h"

// Aliases
#define UISIG_ERROR   UI_LED1 // Signal: Error
#define UISIG_MANUAL  UI_LED2 // Signal: Manual mode
#define UISIG_ACTIVE  UI_RDP  // Signal: Motor activity

// Calibration

// Mounting angle offset to calibrated angle?

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

double toleranceHighRes = 1.0; // +/- angle (degrees)
double toleranceLowRes = 5.0; // +/- angle (degrees)

// Additional angle (overshoot) due to relay turn off delay, etc.
double stopTimeUp = 0.7; // angle (degrees)
double stopTimeDn = 0.7; // angle (degrees)

// Time to wait after movement for relay to turn off and movement to settle
unsigned long delayAfterMove = 200; // milliseconds

// List of user selectable set points
int userSetpointLen = 8;
double userSetpoints[] = {
  85, 9, 6, 3, 0, -4, -8, -10  // angle (degrees) // Avoids sensor transition area at 10 deg
  // 10, -10  // angle (degrees)
  // 85, 20, 15, 14, 13, 12, 11, 10, 9, 6, 3, 0, -4, -8, -10  // angle (degrees)  // Highlights PROBLEM TOLERANCE near 10 deg
};

// Safety / Error limits
double maxSetpoint = 91.0;          // angle (degrees)
double minSetpoint = -11.0;         // angle (degrees)
double sensorsConvergeToleranceHighRes = 1.0; // angle (degrees)
double sensorsConvergeToleranceLowRes = 10.0; // angle (degrees)
unsigned long maxRelayOnTime = 10000; // milliseconds
unsigned long maxRelayCyclesPer  = 6;
unsigned long maxRelayCyclesTime = 2000; // milliseconds

// Convert wedge angle to single character for display
int angleToCharThresholdLen = 16;
double angleToCharThreshold[] = {
  80, 70, 60, 50, 40, 30, 20, 12, 8, 4.5, 1.5, -1.5, -6, -9, -11, -90
};
char angleChars[] = {
  'F', 'E', 'd', 'c', 'b', 'A', '9', '8', '7', '6', '5', '4', '3', '2', '1', '0'
};

// ----------------------------------------------------------

// #define ENABLE_SERIAL_LOG

// TODO: Store and reference double volts or int millivolts?

#include <EEPROM.h>
#include "IS31FL3726A.h"
#include <LiquidCrystal.h>

IS31FL3726A seg7(UI_SERIAL_PIN, UI_ENABLE_PIN, UI_LATCH_PIN, UI_CLOCK_PIN);
LiquidCrystal lcd(LCD_RS_PIN, LCD_EN_PIN, LCD_D0_PIN, LCD_D1_PIN, LCD_D2_PIN, LCD_D3_PIN);

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
int screens = 4;
int screen = 0;

uint8_t seg7Counter = 0;            // testing 7-segment display

// For debug
unsigned long loopCount = 0;

// Possible error flags
#define ERR_BAD_SENSOR    0b10000000 // Angle sensor out of valid range. Check sensor connection, power.
#define ERR_SENSOR_DIVERG 0b01000000 // Angle sensors diverged. Difference between sensors is too large.
#define ERR_RELAY_CYCLING 0b00000010 // Relay cycling ON/OFF too much. Control unstable around setpoint.
#define ERR_RELAY_ON_LONG 0b00000001 // Relay has been ON too long.

// For control
bool controlEnable = false;         // Enable automatic control
uint8_t errorFlags = 0;             // Errors which inhibit control
bool raising, lowering = false;

// For checking relay cycling too frequently or on too long
unsigned long lastRelayStart = 0;   // last time a relay turned ON
unsigned long lastRelayStop = 0;    // last time a relay turned OFF
unsigned long lastRelayStopLogged = 0; // last time a relay stop was logged
uint16_t upCount, dnCount = 0;      // how often are the relays turning ON?
uint16_t relayCount = 0;
uint16_t lastRelayCount = 0;
unsigned long relayCycleTimer = 0;  // last check of relay cycling

double v1, v2;                      // Current sensor voltage
double angle1, angle2, angleDiff;   // Current sensor calibrated angle

double setpoint, input, output;
double error;
double tolerance;
double sensorsConvergeTolerance;

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
  setpoint = userSetpoints[index];
  // Limit setpoint min/max safe angle
  if (setpoint > maxSetpoint) {
    setpoint = maxSetpoint;
  }
  if (setpoint < minSetpoint) {
    setpoint = minSetpoint;
  }
  // TODO Further sanity check setpoint results in valid voltage?
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
  logMoveStart();
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
  logMoveStart();
  lastRelayStart = millis();
}

void moveStop() {
  // Turn OFF relays
  digitalWrite(RELAY_UP_PIN, LOW);
  digitalWrite(RELAY_DN_PIN, LOW);
  if (raising || lowering) {
    raising = false;
    lowering = false;
    lastRelayStop = millis();
    logMoveStop();
  }
}

void setup() {
#ifdef ENABLE_SERIAL_LOG
  // Serial.begin(9600);
  Serial.begin(115200);
  Serial.println("Hello Computer!");
#endif

  seg7.begin();
  lcd.begin(16, 2);

  // Up/down buttons on dash
  pinMode(EN_MANUAL_PIN, INPUT);
  pinMode(BTN_DN_PIN, INPUT);
  pinMode(BTN_UP_PIN, INPUT);

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
  // controlEnable = true;
  controlEnable = (EEPROM.read(controlEnableEepromAddr) & 0x1);
}

void loop() {
  ++loopCount;
  
  // Read sensor voltage (averaged to reject noise)
  v1 = readAngleSensorV1();
  v2 = readAngleSensorV2();
  for (int i = 0; i < 5; ++i) {
    delayMicroseconds(100);
    v1 += readAngleSensorV1();
    v2 += readAngleSensorV2();
  }
  v1 = v1/6.0;
  v2 = v2/6.0;

  // TODO Handle noise

  // TODO If control by angle instead of voltage, include angle in check?
  // TODO Separate error flag for each angle sensor?
  // Sanity check input
  // Don't use this to limit movement (use maxSetpoint or something else).
  if (v1 < 0.30 || v1 > 4.70 || v2 < 0.30 || v2 > 4.65) {
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
  angle1 = interp(voltToAngle1, v1, numCalibPoints1);
  angle2 = interp(voltToAngle2, v2, numCalibPoints2);

  // Do the sensors agree on angle?
  angleDiff = angle1 - angle2;

  // Handle sensorsConvergeTolerance on two different sensor ranges
  if (angle1 < 10 && angle2 < 10) {
    sensorsConvergeTolerance = sensorsConvergeToleranceHighRes;
  } else {
    sensorsConvergeTolerance = sensorsConvergeToleranceLowRes;
  }
  
  if (abs(angleDiff) > 1) {
    logSensorDiff();
  }
  if (abs(angleDiff) > sensorsConvergeTolerance) {
    moveStop();
    errorFlags |= ERR_SENSOR_DIVERG;      // Set error flag
    // Log this? Count? Sum time in error? Display message?
    // TODO: Should only call moveStop() if err flag not already set?
  } else {
    // Clear error automatically
    // TODO: Log this? Count? Sum time in error? Display message?
    errorFlags &= ~ERR_SENSOR_DIVERG;
  }

  // Choose one value from two sensors
  // TODO: What if both sensors don't agree?
  //input = (angle1 + angle2)/2.0;
  input = angle1;

  // Handle tolerance on two different sensor ranges
  // if (input < 17.0) {
  if (input < 10.0 || setpoint < 10.0) {
    tolerance = toleranceHighRes;
  } else {
    tolerance = toleranceLowRes;
  }

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
      // Only move if: error > tolerance AND been long enough after a move
      if (abs(error) > tolerance && millis() - lastRelayStop > delayAfterMove) {
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

#ifdef ENABLE_SERIAL_LOG
  // Log most recent relay stop
  if (lastRelayStop - lastRelayStopLogged != 0) {
    if (millis() - lastRelayStop > (delayAfterMove * 0.80)) {
      // After a move has had time to settle
      logMoveDone();
      lastRelayStopLogged = lastRelayStop;
    }
  }
#endif

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
      controlEnable = !controlEnable;
      moveStop();
      // TODO Choose nearest user set point on re-enable control.
      // Save this in EEPROM.
      EEPROM.update(controlEnableEepromAddr, controlEnable ? 0x01 : 0x00);
    } else if (key == KEY_LEFT) {
      --screen;
      if (screen < 0) {
        screen = screens - 1; // wrap around
      }
      lcd.clear();
      redraw = true;
    } else if (key == KEY_DOWN) {
      // TODO Invert these 2 keys when we flip the unit and install it on a boat!
      if (controlEnable) {
        prevSetpoint();
      }
    } else if (key == KEY_UP) {
      if (controlEnable) {
        nextSetpoint();
      }
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

  // Manual relay movement
  if (!controlEnable) {
    // Move based on keyState. Don't require key changes -- if key is down, move that direction.
    // TODO Invert these 2 keys when we flip the unit and install it on a boat!
    if (keyState == KEY_DOWN) {
      if (!raising) {
        moveRaise();
      }
    } else if (keyState == KEY_UP) {
      if (!lowering) {
        moveLower();
      }
    } else {
      if (raising || lowering) {
        moveStop();
      }
    }
  }

  if (redraw || millis() - tepTimer > 500) {
    // For debug, calc stepTime
    int stepTime = loopCount;
    loopCount = 0;
    
    redraw = false;
    tepTimer = millis();

    // Update 7-segment display
    // For testing, highlight each segment one at a time
    ++seg7Counter;
    if (seg7Counter > 13) {
      seg7Counter = 0; // rollover
    }
    // seg7.set(0x1 << seg7Counter);
    // seg7.set(UICHAR_E | (seg7Counter % 2 > 0 ? UI_LDP : UI_RDP));
    // seg7.set(UICHAR_7 | (seg7Counter % 2 > 0 ? UI_LED1 : UI_LED2));
    // seg7.set(UICHAR_7 | (seg7Counter % 2 > 0 ? UI_NONE : UI_LED1));
    // seg7.set(UICHAR_7 | (seg7Counter % 2 > 0 ? UI_LDP : UI_LED1));
    // seg7.set(UICHAR_7 | UI_LED2);
    // seg7.set(UI_LDP | (seg7Counter % 2 > 0 ? UI_NONE : UICHAR_7)); // Flash the char
    // seg7.set(UI_RDP | (seg7Counter % 2 > 0 ? UICHAR_6 : UICHAR_7)); // Alternate chars

    // Test pattern
    if (seg7Counter == 0) {
      seg7.set(UICHAR_0);
      // seg7.set(UICHAR_4 | UISIG_ERROR);
      // seg7.set(UI_LED1);
    } else if (seg7Counter == 1) {
      seg7.set(UICHAR_1);
      // seg7.set(UICHAR_F | UISIG_ACTIVE);
      // seg7.set(UI_LDP);
    } else if (seg7Counter == 2) {
      seg7.set(UICHAR_2);
      // seg7.set(UICHAR_0 | UISIG_MANUAL);
      // seg7.set(UI_RDP);
    } else if (seg7Counter == 3) {
      seg7.set(UICHAR_3);
      // seg7.set(UICHAR_A);
      // seg7.set(UI_LED2);
    } else if (seg7Counter == 4) {
      seg7.set(UICHAR_4);
    } else if (seg7Counter == 5) {
      seg7.set(UICHAR_5);
    } else if (seg7Counter == 6) {
      seg7.set(UICHAR_6);
    } else if (seg7Counter == 7) {
      seg7.set(UICHAR_7);
    } else if (seg7Counter == 8) {
      seg7.set(UICHAR_8);
    } else if (seg7Counter == 9) {
      seg7.set(UICHAR_9);
    } else if (seg7Counter == 10) {
      seg7.set(UI_LED1);
    } else if (seg7Counter == 11) {
      seg7.set(UI_LED2);
    } else if (seg7Counter == 12) {
      seg7.set(UI_LDP);
    } else if (seg7Counter == 13) {
      seg7.set(UI_RDP);
    }

    // seg7.set(0b0100000000000001);

    // seg7.set(UICHAR_4 | UISIG_ERROR);
    // seg7.set(UICHAR_F | UISIG_ACTIVE);
    // seg7.set(UICHAR_0 | UISIG_MANUAL);

    // TODO Disable keypad input on error screen.
    // Pop up errors -- separate from normal screens below.
    if (errorFlags != 0) {
      lcd.clear();
      lcd.setCursor(0, 0); // set the LCD cursor position
      // If we use the whole screen, only show 1 error
      // Could do 1 per line, etc..
      if ((errorFlags & ERR_BAD_SENSOR) > 0) {
        lcd.print("Err Bad Sensor.");
      } else if ((errorFlags & ERR_SENSOR_DIVERG) > 0) {
        lcd.print("Err Sensors");
        lcd.setCursor(0, 1); // set the LCD cursor position
        lcd.print("Diverged");
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
      lcd.setCursor(7, 0); // set the LCD cursor position
      if (controlEnable) {
        lcd.print(angleToChar(setpoint));
      } else {
        lcd.print('_');
      }
      lcd.print(' ');
      lcd.print(angleToChar(input));
    } else if (screen == 3) {
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

char angleToChar(double angle) {
  for (int i = 0; i < angleToCharThresholdLen; ++i) {
    if (angle > angleToCharThreshold[i]) {
      return angleChars[i];
    }
  }
  return ' ';
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

// Log to serial a move start
void logMoveStart() {
#ifdef ENABLE_SERIAL_LOG
  if (raising) {
    if (upCount < 10) {
      Serial.print('0');
    }
    Serial.print(upCount);
    Serial.print(", UP, ");
  } else if (lowering) {
    if (dnCount < 10) {
      Serial.print('0');
    }
    Serial.print(dnCount);
    Serial.print(", DN, ");
  } else {
    Serial.print(", UNKNOWN Movement, ");
  }
  
  Serial.print(setpoint);
  Serial.print(", ");
  Serial.print(input);
  Serial.print(", ");
  Serial.print(error);
  Serial.print(", ");
  Serial.print(controlEnable ? "" : "MANUAL");
  Serial.println(", ");
#endif
}

// Log to serial a move stop
void logMoveStop() {
#ifdef ENABLE_SERIAL_LOG
  Serial.print(", STOP, ");
  Serial.print(setpoint);
  Serial.print(", ");
  Serial.print(input);
  Serial.print(", ");
  Serial.print(error);
  Serial.print(", ");
  Serial.print(lastRelayStop - lastRelayStart);
  Serial.println("ms");
#endif
}

// Log to serial a move done
void logMoveDone() {
#ifdef ENABLE_SERIAL_LOG
  Serial.print(", DONE, ");
  Serial.print(setpoint);
  Serial.print(", ");
  Serial.print(input);
  Serial.print(", ");
  Serial.print(error);
  Serial.println(", ");
#endif
}

// Log to serial angle sensor difference
void logSensorDiff() {
#ifdef ENABLE_SERIAL_LOG
  Serial.print("Sensor difference: ");
  Serial.print(angleDiff);
  Serial.print(" = ");
  Serial.print(angle1);
  Serial.print(" - ");
  Serial.println(angle2);
#endif
}