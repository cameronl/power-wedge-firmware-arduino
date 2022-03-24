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
unsigned long maxRelayOnTime = 15000; // milliseconds
unsigned long maxRelayCyclesPer  = 6;
unsigned long maxRelayCyclesTime = 2000; // milliseconds

// Convert wedge angle to single character for display
int angleToCharThresholdLen = 16;
double angleToCharThreshold[] = {
  80, 70, 60, 50, 40, 30, 20, 12, 8, 4.5, 1.5, -1.5, -6, -9, -11, -90
};
/// For printing to LCD
char angleChars[] = {
  'F', 'E', 'd', 'c', 'b', 'A', '9', '8', '7', '6', '5', '4', '3', '2', '1', '0'
};
/// Outputs for UI 7-segment display
uint16_t angleUIChars[] = {
  UICHAR_F,
  UICHAR_E,
  UICHAR_D,
  UICHAR_C,
  UICHAR_B,
  UICHAR_A,
  UICHAR_9,
  UICHAR_8,
  UICHAR_7,
  UICHAR_6,
  UICHAR_5,
  UICHAR_4,
  UICHAR_3,
  UICHAR_2,
  UICHAR_1,
  UICHAR_0
};

// ----------------------------------------------------------

#define modelVersion 0
#define firmwareVersion 0

// #define ENABLE_SERIAL_LOG
#define ENABLE_SDCARD

// TODO: Store and reference double volts or int millivolts?

#include <EEPROM.h>
#include "IS31FL3726A.h"
#include <LiquidCrystal.h>

#ifdef ENABLE_SDCARD
  #include <SD.h>
  File logFile;
  #define log logFile
#endif
#ifdef ENABLE_SERIAL_LOG
  #define log Serial
#endif

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

uint8_t dashBtnState = 0;           // State of dash buttons (UP/DN)
unsigned long dashBtnLastTime = 0;  // last time dash button activated
unsigned long dashBtnRepeatDelay = 800;  // time (ms) before repeating button if held down

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastRealKeyTime = 0;  // last time key changed (for debounce)
unsigned long debounceDelay = 50;   // the debounce time; increase if the output flickers

bool redraw = true;                 // should redraw the lcd screen
int screens = 5;
int screen = 0;
#define SCREEN_PROD_ERR 0
#define SCREEN_ANGLE_READ 1
#define SCREEN_INPUT_VS_SET 2
#define SCREEN_RELAY_COUNTS 3
#define SCREEN_USE_ANGLE_SENS 4

// Possible error flags
#define ERR_BAD_SENSOR1   0b10000000 // Angle sensor out of valid range. Check sensor connection, power.
#define ERR_BAD_SENSOR2   0b01000000 // Angle sensor out of valid range. Check sensor connection, power.
#define ERR_SENSOR_DIVERG 0b00000100 // Angle sensors diverged. Difference between sensors is too large.
#define ERR_RELAY_CYCLING 0b00000010 // Relay cycling ON/OFF too much. Control unstable around setpoint.
#define ERR_RELAY_ON_LONG 0b00000001 // Relay has been ON too long.

// For control
bool controlEnable = false;         // Enable automatic control
uint8_t errorFlags = 0;             // Errors which are currently occurring
uint8_t lastErrorFlags = 0;         // For detecting when error flags change
uint8_t controlErrors = 0xFF;       // Bitmask indicating which errors inhibit control
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
uint8_t useAngleSensor;             // Which angle sensor to use?

void prevAngleSensor() {
  useAngleSensor--;
  if (useAngleSensor < 1) {
    useAngleSensor = 3;
  }
  onAngleSensorChange();
}

void nextAngleSensor() {
  useAngleSensor++;
  if (useAngleSensor > 3) {
    useAngleSensor = 1;
  }
  onAngleSensorChange();
}

// Call when there is a change to which angle sensor to use.
void onAngleSensorChange() {
  moveStop();
  // Which errors should apply to this angle sensor?
  if (useAngleSensor == 1) {
    // If using sensor 1, we don't care about errors from sensor 2
    controlErrors &= ~ERR_BAD_SENSOR2;     // Clear bit
    controlErrors &= ~ERR_SENSOR_DIVERG;   // Clear bit
    // But we do care about errors from sensor 1
    controlErrors |= ERR_BAD_SENSOR1;      // Set bit
  } else if (useAngleSensor == 2) {
    // If using sensor 2, we don't care about errors from sensor 1
    controlErrors &= ~ERR_BAD_SENSOR1;     // Clear bit
    controlErrors &= ~ERR_SENSOR_DIVERG;   // Clear bit
    // But we do care about errors from sensor 2
    controlErrors |= ERR_BAD_SENSOR2;      // Set bit
  } else if (useAngleSensor == 3) {
    // If requiring both sensors, we do care about errors from both
    controlErrors |= ERR_BAD_SENSOR1;      // Set bit
    controlErrors |= ERR_BAD_SENSOR2;      // Set bit
    controlErrors |= ERR_SENSOR_DIVERG;    // Set bit
  }
  // TODO Implement useAngleSensor == 4 (Any valid)

  // Save this config choice in EEPROM
  EEPROM.update(useAngleSensorEepromAddr, useAngleSensor & 0xFF);
}

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

#ifdef ENABLE_SDCARD
  // Note that even if it's not used as the CS pin, the hardware SS pin 
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output 
  // or the SD library functions will not work. 
  pinMode(SDCARD_CS_PIN, OUTPUT);
  SD.begin(SDCARD_CS_PIN);
  logFile = SD.open("log.csv", FILE_WRITE);
#endif

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

  useAngleSensor = EEPROM.read(useAngleSensorEepromAddr);
  if (useAngleSensor > 3 || useAngleSensor < 1) {
    useAngleSensor = 1; // Default to sensor 1
  }
  onAngleSensorChange();

  // What if MCU reboots? Don't want setpoint to jump. Store/load it from flash.
  // Currently storing index in list of user setpoints -- NOT the actual angle/voltage.
  userSetpointIndex = EEPROM.read(setpointEepromAddr);
  if (userSetpointIndex > userSetpointLen - 1) {
    userSetpointIndex = 0; // Home is probably best place to start if uninitialized.
  }

  //setpoint = 3.500; // V
  goToUserSetpoint(userSetpointIndex);
}

void loop() {
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

  // Sanity check input for:
  //   Angle sensor out of valid range
  //   Check sensor connection, power.
  // Don't use this to limit movement (use maxSetpoint or something else).
  if (v1 < 0.30 || v1 > 4.70) {
    errorFlags |= ERR_BAD_SENSOR1;      // Set error flag
    if (controlErrors & ERR_BAD_SENSOR1) {
      moveStop();
    }
  } else {
    // Clear error automatically
    errorFlags &= ~ERR_BAD_SENSOR1;
  }
  if (v2 < 0.30 || v2 > 4.65) {
    errorFlags |= ERR_BAD_SENSOR2;      // Set error flag
    if (controlErrors & ERR_BAD_SENSOR2) {
      moveStop();
    }
  } else {
    // Clear error automatically
    errorFlags &= ~ERR_BAD_SENSOR2;
  }

  // Convert voltage to calibrated angle
  angle1 = interp(voltToAngle1, v1, numCalibPoints1);
  angle2 = interp(voltToAngle2, v2, numCalibPoints2);

  // Do the sensors agree on angle?
  angleDiff = angle1 - angle2;

  // Check if both sensors agree
  // Handle sensorsConvergeTolerance on two different sensor ranges
  if (angle1 < 10 && angle2 < 10) {
    sensorsConvergeTolerance = sensorsConvergeToleranceHighRes;
  } else {
    sensorsConvergeTolerance = sensorsConvergeToleranceLowRes;
  }
  if (abs(angleDiff) > sensorsConvergeTolerance) {
    errorFlags |= ERR_SENSOR_DIVERG;      // Set error flag
    if (controlErrors & ERR_SENSOR_DIVERG) {
      moveStop();
    }
  } else {
    // Clear error automatically
    errorFlags &= ~ERR_SENSOR_DIVERG;
  }

  // Choose one value from two sensors
  if (useAngleSensor == 1) {
    input = angle1;
  } else if (useAngleSensor == 2) {
    input = angle2;
  } else if (useAngleSensor == 3) {
    // TODO: What if both sensors don't agree?
    input = (angle1 + angle2)/2.0;
  }

  // Handle tolerance on two different sensor ranges
  // if (input < 17.0) {
  if (input < 10.0 || setpoint < 10.0) {
    tolerance = toleranceHighRes;
  } else {
    tolerance = toleranceLowRes;
  }

  // TODO Only check this below if control is enabled?
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

  // See if control is enabled?
  bool manualEnable = digitalRead(EN_MANUAL_PIN) == HIGH;
  if (controlEnable != !manualEnable) {
    // A change
    controlEnable = !manualEnable;
    moveStop();
  }

  // -------------------------------------------------------------
  // Control
  error = input - setpoint;
  if (controlEnable && (errorFlags & controlErrors) == 0) {
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

#ifdef log
  // Log most recent relay stop
  if (lastRelayStop - lastRelayStopLogged != 0) {
    if (millis() - lastRelayStop > (delayAfterMove * 0.80)) {
      // After a move has had time to settle
      logMoveDone();
      lastRelayStopLogged = lastRelayStop;
    }
  }
#endif

  // Handle dash buttons
  uint8_t dashBtns = digitalRead(BTN_DN_PIN) | (digitalRead(BTN_UP_PIN) << 1);
  if (dashBtns != dashBtnState || (dashBtns != 0 && millis() - dashBtnLastTime > dashBtnRepeatDelay)) {
    // A change
    dashBtnState = dashBtns;
    dashBtnLastTime = millis();

    // Handle the change
    bool btnDn = dashBtns & 0b00000001;
    bool btnUp = dashBtns & 0b00000010;
    if (!btnDn && !btnUp) {
      // Do nothing
    } else if (btnDn && btnUp) {
      // Ignore
      // OPTION Log this?
    } else if (btnDn) {
      if (controlEnable) {
        prevSetpoint();
      }
    } else if (btnUp) {
      if (controlEnable) {
        nextSetpoint();
      }
    }
  }

  // Handle keypad buttons
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
      // Unused
    } else if (key == KEY_LEFT) {
      --screen;
      if (screen < 0) {
        screen = screens - 1; // wrap around
      }
      // lcd.clear();
      redraw = true;
    } else if (key == KEY_DOWN) {
      if (screen == SCREEN_ANGLE_READ || screen == SCREEN_USE_ANGLE_SENS) {
        nextAngleSensor();
      }
    } else if (key == KEY_UP) {
      if (screen == SCREEN_ANGLE_READ || screen == SCREEN_USE_ANGLE_SENS) {
        prevAngleSensor();
      }
    } else if (key == KEY_RIGHT) {
      ++screen;
      if (screen > screens - 1) {
        screen = 0; // wrap around
      }
      // lcd.clear();
      redraw = true;
    } else {
      // Should never happen.
    }
  }

  // LCD clear screen is slow, so only do so when something changes
  if (errorFlags != lastErrorFlags) {
    lastErrorFlags = errorFlags;
    // A change
    redraw = true;

    // Log error changes? Count? Sum time in error? Display message?
    // TODO: Should only call moveStop() if err flag not already set?
    logError();
  }

  if (redraw || millis() - tepTimer > 200) {
    if (redraw == true) {
      lcd.clear();
    }
    redraw = false;
    tepTimer = millis();

    // Update 7-segment display
    uint16_t uiOut = angleToUIChar(controlEnable ? setpoint : input);
    if (raising) {
      uiOut |= UI_RDP;
    }
    if (lowering) {
      uiOut |= UI_LDP;
    }
    if ((errorFlags & controlErrors) != 0) {
      uiOut |= UI_LED1;
    }
    if (!controlEnable) {
      uiOut |= UI_LED2;
    }
    seg7.set(uiOut);

    // seg7.set(UICHAR_4 | UISIG_ERROR);
    // seg7.set(UICHAR_F | UISIG_ACTIVE);
    // seg7.set(UICHAR_0 | UISIG_MANUAL);

    // TODO Disable keypad input on error screen.

    if (screen == SCREEN_PROD_ERR) {
      lcd.setCursor(0, 0); // set the LCD cursor position
      if (errorFlags == 0) {
          lcd.print("PowerWedge v");
          lcd.print(modelVersion);
          lcd.print('.');
          lcd.print(firmwareVersion);
          lcd.setCursor(3, 1); // set the LCD cursor position
          lcd.print("No Errors");
      } else {
        // One error per line, can only show two.
        if ((errorFlags & ERR_BAD_SENSOR1) > 0) {
          lcd.print("Err Bad Sensor 1"); lcd.setCursor(0, 1); // next line
        }
        if ((errorFlags & ERR_BAD_SENSOR2) > 0) {
          lcd.print("Err Bad Sensor 2"); lcd.setCursor(0, 1); // next line
        }
        if ((errorFlags & ERR_RELAY_CYCLING) > 0) {
          lcd.print("ErrRelay Cycling"); lcd.setCursor(0, 1); // next line
        }
        if ((errorFlags & ERR_RELAY_ON_LONG) > 0) {
          lcd.print("Relay On TooLong"); lcd.setCursor(0, 1); // next line
        }
        if ((errorFlags & ERR_SENSOR_DIVERG) > 0) {
          lcd.print("Err SensDiverged"); lcd.setCursor(0, 1); // next line
        }
      }
    } else if (screen == SCREEN_ANGLE_READ) {
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

      // Print selected angle sensors
      lcd.setCursor(15, 0); // set the LCD cursor position
      if (useAngleSensor == 1 || useAngleSensor == 3) {
        lcd.print('*');
      } else {
        lcd.print(' ');
      }
      lcd.setCursor(15, 1); // set the LCD cursor position
      if (useAngleSensor == 2 || useAngleSensor == 3) {
        lcd.print('*');
      } else {
        lcd.print(' ');
      }

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
    } else if (screen == SCREEN_INPUT_VS_SET) {
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
    } else if (screen == SCREEN_RELAY_COUNTS) {
      // Print relay counters
      lcd.setCursor(0, 0); // set the LCD cursor position
      lcd.print("Relay    up:");
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

      lcd.setCursor(0, 1); // set the LCD cursor position
      lcd.print("Count    dn:");
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
    } else if (screen == SCREEN_USE_ANGLE_SENS) {
      lcd.setCursor(0, 0); // set the LCD cursor position
      lcd.print("Use AngleSensor:");
      lcd.setCursor(0, 1); // set the LCD cursor position
      if (useAngleSensor == 1) {
        lcd.print("1 Blue      ");
      } else if (useAngleSensor == 2) {
        lcd.print("2 Brown     ");
      } else if (useAngleSensor == 3) {
        lcd.print("Require Both");
      } else {
        lcd.print("Unknown     ");
      }
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

uint16_t angleToUIChar(double angle) {
  for (int i = 0; i < angleToCharThresholdLen; ++i) {
    if (angle > angleToCharThreshold[i]) {
      return angleUIChars[i];
    }
  }
  return UICHAR__;
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

// Log a move start
void logMoveStart() {
#ifdef log
  log.print(upCount + dnCount);
  if (raising) {
    log.print(",UP, ");
  } else if (lowering) {
    log.print(",DN, ");
  } else {
    log.print(",UNKNOWN Movement, ");
  }
  
  log.print(setpoint);
  log.print(", ");
  log.print(input);
  log.print(", ");
  log.print(error);
  log.print(", ");
  log.print(controlEnable ? "" : "MANUAL");
  log.print(", ");
#endif
#ifdef ENABLE_SDCARD
  if (logFile) {
    logFile.flush();
  }
#endif
}

// Log a move stop
void logMoveStop() {
#ifdef log
  log.print(",STOP, ");
  log.print(setpoint);
  log.print(", ");
  log.print(input);
  log.print(", ");
  log.print(error);
  log.print(", ");
  log.print(lastRelayStop - lastRelayStart);
  // log.println("ms");
#endif
#ifdef ENABLE_SDCARD
  if (logFile) {
    logFile.flush();
  }
#endif
}

// Log a move done
void logMoveDone() {
#ifdef log
  log.print(",DONE, ");
  log.print(setpoint);
  log.print(", ");
  log.print(input);
  log.print(", ");
  log.print(error);
  log.println(", ");
#endif
#ifdef ENABLE_SDCARD
  if (logFile) {
    logFile.flush();
  }
#endif
}

// Log angle sensor difference
void logSensorDiff() {
#ifdef log
  log.print("Sensor difference: ");
  log.print(angleDiff);
  log.print(" = ");
  log.print(angle1);
  log.print(" - ");
  log.println(angle2);
#endif
#ifdef ENABLE_SDCARD
  if (logFile) {
    logFile.flush();
  }
#endif
}

// Log Error flags
void logError() {
#ifdef log
  if (errorFlags == 0) {
    return;
  }
  log.print("[");
  if ((errorFlags & ERR_BAD_SENSOR1) > 0) {
    log.print("Err Bad Sensor 1 ");
  }
  if ((errorFlags & ERR_BAD_SENSOR2) > 0) {
    log.print("Err Bad Sensor 2 ");
  }
  if ((errorFlags & ERR_RELAY_CYCLING) > 0) {
    log.print("ErrRelay Cycling ");
  }
  if ((errorFlags & ERR_RELAY_ON_LONG) > 0) {
    log.print("Relay On TooLong ");
  }
  if ((errorFlags & ERR_SENSOR_DIVERG) > 0) {
    log.print("Err SensDiverged ");
  }
  log.println("]");
#endif
#ifdef ENABLE_SDCARD
  if (logFile) {
    logFile.flush();
  }
#endif
}
