
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
  int val1;                         // store value coming from analog pin
  double data1;                     // store temperature value from conversion formula

  val1 = analogRead(A4);            // read the analog in value:
  // data1 = (double) val1 * (5/10.24); // temperature conversion formula
  data1 = (double) val1 / 1023.0 * 5.0;

  int val2;                         // store value coming from analog pin
  double data2;                     // store temperature value from conversion formula

  val2 = analogRead(A5);            // read the analog in value:
  // data2 = (double) val2 * (5/10.24); // temperature conversion formula
  data2 = (double) val2 / 1023.0 * 5.0;

  // Handle buttons
  KEY key = readKeyInput();

  // Only update on key change?
  // Wait for a key repeat or release timer?
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
    lcd.setCursor(0, 0); // set the LCD cursor position
    // print the results to the lcd
    //lcd.print("R: ");
    lcd.print(data1);
    lcd.print("V");

    lcd.setCursor(0, 1); // set the LCD cursor position
    // print the results to the lcd
    //lcd.print("R: ");
    lcd.print(data2);
    lcd.print("V");

    // Print counter
    lcd.setCursor(5, 0); // set the LCD cursor position
    if (abs(counter) < 100) {
      lcd.print(' ');
    }
    if (abs(counter) < 10) {
      lcd.print(' ');
    }
    if (counter >= 0) {
      lcd.print(' ');
    }
    lcd.print(counter);

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
