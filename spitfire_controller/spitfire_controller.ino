// Author: Peter Milne
// Date: May 2020

// FlightGear Spitfire IIa controller using MCP23017
// MCP23017 driver adapted from code by Nick Gammon
// https://www.gammon.com.au/forum/?id=10945

#include <Wire.h>
#include <Keyboard.h>

// Spitfire controls
const char LEFT_MAGNETO = '[';       // GPB0 { (SHIFT + [)
const char RIGHT_MAGNETO = ']';      // GPB1 } (SHIFT + ])
const char FLAPS_UP = '[';           // GPB2
const char FLAPS_DOWN = ']';         // GPB2
const char COFFMAN_CARTRIDGE = 'C';  // GPB3
const char PRIMING_PUMP = 'I';       // GPB4
const char COFFMAN_STARTER = 's';    // GPB5
const char CANOPY_CLOSE = 'f';       // GPB6
const char CANOPY_OPEN = 'F';        // GPB6
const char GEAR_UP = 'g';            // GPB7
const char GEAR_DOWN = 'G';          // GPB7

const int start_button = 4;
const int int_pin = 7;

int previousButtonState[16];

// MCP23017 registers (everything except direction defaults to 0)
#define IODIRA 0x00  // IO direction  (0 = output, 1 = input (Default))
#define IODIRB 0x01
#define IOPOLA 0x02  // IO polarity   (0 = normal, 1 = inverse)
#define IOPOLB 0x03
#define GPINTENA 0x04  // Interrupt on change (0 = disable, 1 = enable)
#define GPINTENB 0x05
#define DEFVALA 0x06  // Default comparison for interrupt on change (interrupts on opposite)
#define DEFVALB 0x07
#define INTCONA 0x08  // Interrupt control (0 = interrupt on change from previous, 1 = interrupt on change from DEFVAL)
#define INTCONB 0x09
#define IOCON 0x0A  // IO Configuration: bank/mirror/seqop/disslw/haen/odr/intpol/notimp
//#define IOCON 0x0B  // same as 0x0A
#define GPPUA 0x0C  // Pull-up resistor (0 = disabled, 1 = enabled)
#define GPPUB 0x0D
#define INFTFA 0x0E  // Interrupt flag (read only) : (0 = no interrupt, 1 = pin caused interrupt)
#define INFTFB 0x0F
#define INTCAPA 0x10  // Interrupt capture (read only) : value of GPIO at time of last interrupt
#define INTCAPB 0x11
#define GPIOA 0x12  // Port value. Write to change, read to obtain value
#define GPIOB 0x13
#define OLLATA 0x14  // Output latch. Write to latch output.
#define OLLATB 0x15
#define port 0x20  // MCP23017 is on I2C port 0x20

#define ISR_INDICATOR 12  // pin 12
#define ONBOARD_LED 13    // pin 13

volatile bool keyPressed;

// Write same register setting to both ports
void expanderWriteBoth(const byte reg, const byte data) {
  Wire.beginTransmission(port);
  Wire.write(reg);
  Wire.write(data);  // port A
  Wire.write(data);  // port B
  Wire.endTransmission();
}

// Read a single byte from the expander
unsigned int expanderRead(const byte reg) {
  Wire.beginTransmission(port);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(port, 1);
  return Wire.read();
}

// Interrupt service routine, called when pin D2 goes from 1 to 0
void keypress() {
  digitalWrite(ISR_INDICATOR, HIGH);  // debugging
  keyPressed = true;                  // set flag so main loop knows
}

void setup() {
  pinMode(ISR_INDICATOR, OUTPUT);  // for testing (ISR indicator)
  pinMode(ONBOARD_LED, OUTPUT);    // for onboard LED
  pinMode(start_button, INPUT_PULLUP);

  Wire.begin();
  Serial.begin(9600);

  // Safe start for testing - only start after button press
  while (digitalRead(start_button))
    ;

  Keyboard.begin();
  delay(5000);
  Serial.println("Starting ...");

  expanderWriteBoth(IOCON, 0b01100000);  // mirror interrupts, disable sequential mode
  expanderWriteBoth(GPPUA, 0xFF);        // pull-up resistor for switch - both ports
  expanderWriteBoth(IOPOLA, 0xFF);       // invert polarity of signal - both ports
  expanderWriteBoth(GPINTENA, 0xFF);     // enable interrupts - both ports

  keyPressed = false;  // no interrupt yet
  // read from interrupt capture ports to clear them
  expanderRead(INTCAPA);
  expanderRead(INTCAPB);

  // Must be interrupt capable pin
  attachInterrupt(digitalPinToInterrupt(int_pin), keypress, FALLING);
}

unsigned long time = 0;
unsigned int keyValue = 0;
bool firstRun = true;

// Called interrupts fire
void handleKeypress() {
  delay(50);  // de-bounce before we re-enable interrupts

  keyPressed = false;                // ready for next time through the interrupt service routine
  digitalWrite(ISR_INDICATOR, LOW);  // debugging

  // Read port values, as required. Note that this re-arms the interrupts.
  if (expanderRead(INFTFA)) {
    keyValue &= 0x00FF;
    keyValue |= expanderRead(INTCAPA) << 8;  // read value at time of interrupt
  }
  if (expanderRead(INFTFB)) {
    keyValue &= 0xFF00;
    keyValue |= expanderRead(INTCAPB);  // port B is in low-order byte
  }

  Serial.println("Button states");
  Serial.println("0                   1");
  Serial.println("0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5");

  unsigned int buttonState = 0;
  // display which buttons were down at the time of the interrupt
  for (byte button = 0; button < 16; button++) {
    if (buttonState = (keyValue & (1 << button)) >> button) {
      Serial.print("1 ");
      if (buttonState != previousButtonState[button]) {
        switch (button) {
          case 0:
            // Sends SHIFT + '[' sequence -> '{'
            Keyboard.press(KEY_LEFT_SHIFT);
            Keyboard.write(LEFT_MAGNETO);
            Keyboard.release(KEY_LEFT_SHIFT);
            break;  // Latch
          case 1:
            Keyboard.press(KEY_LEFT_SHIFT);
            Keyboard.write(RIGHT_MAGNETO);
            Keyboard.release(KEY_LEFT_SHIFT);
            break;  // Latch
          case 2: Keyboard.write(GEAR_UP); break;
          case 3: Keyboard.write(FLAPS_DOWN); break;
          case 4: Keyboard.write(CANOPY_CLOSE); break;
          case 5:
            if (!firstRun) {
              Keyboard.press(COFFMAN_CARTRIDGE);  // Latch as momentary
              delay(100);
              Keyboard.release(COFFMAN_CARTRIDGE);
            }
            break;
          case 6:
            if (!firstRun) {
              Keyboard.press(PRIMING_PUMP);
              delay(100);
              Keyboard.release(PRIMING_PUMP);
            }
            break;
          case 7: Keyboard.press(COFFMAN_STARTER); break;
          default: break;
        }
      }
    } else {
      Serial.print(". ");
      if (buttonState != previousButtonState[button]) {
        switch (button) {
          case 0:
            Keyboard.press(KEY_LEFT_SHIFT);
            Keyboard.write(LEFT_MAGNETO);
            Keyboard.release(KEY_LEFT_SHIFT);
            break;
          case 1:
            Keyboard.press(KEY_LEFT_SHIFT);
            Keyboard.write(RIGHT_MAGNETO);
            Keyboard.release(KEY_LEFT_SHIFT);
            break;
          case 2: Keyboard.write(GEAR_DOWN); break;
          case 3: Keyboard.write(FLAPS_UP); break;
          case 4: Keyboard.write(CANOPY_OPEN); break;
          case 5:
            if (!firstRun) {
              Keyboard.press(COFFMAN_CARTRIDGE);
              delay(100);
              Keyboard.release(COFFMAN_CARTRIDGE);
            }
            break;
          case 6:
            if (!firstRun) {
              Keyboard.press(PRIMING_PUMP);
              delay(100);
              Keyboard.release(PRIMING_PUMP);
            }
            break;
          case 7: Keyboard.release(COFFMAN_STARTER); break;
          default: break;
        }
      }
    }
    previousButtonState[button] = buttonState;  // Capture state for next time
  }
  firstRun = false;

  Serial.println("Next");

  // if a switch is now pressed, turn LED on  (key down event)
  if (keyValue) {
    time = millis();                  // remember when
    digitalWrite(ONBOARD_LED, HIGH);  // on-board LED
  }                                   // end if
}  // end of handleKeypress

void loop() {
  // was there an interrupt?
  if (keyPressed)
    handleKeypress();

  // Do other stuff in here
  // turn LED off after 500 ms
  if (millis() > (time + 500) && time != 0) {
    digitalWrite(ONBOARD_LED, LOW);
    time = 0;
  }
}
