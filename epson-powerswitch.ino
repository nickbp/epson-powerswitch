// IR decoder + power switcher for Epson projector power button.
// Intended for use with a PowerSwitch Tail II to forcibly shut off
// an Epson (Home Cinema 2040) projector.
//
// Heavily modified version of this adafruit tutorial code (public domain):
//   https://github.com/adafruit/Raw-IR-decoder-for-Arduino
//
// Copyright Nick Parker 2016
//
// This is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Foobar.  If not, see <http://www.gnu.org/licenses/>.

// Connect IR Sensor to pin 2
#define irPin_PIN PIND
#define irPin 2

// Connect PowerSwitch '-in' to pin 3
#define powerPin 3

// Report power status via on-board LED
#define ledPin 13

// Specific to Epson projector remote:

// bytes expected when the power button is pressed (same message twice)
uint8_t bytesExpect[] = { B11000001, B10101010, B00001001, B11110110 };
uint8_t bytesGot[] = { 0, 0, 0, 0 }; // should be same num elements as bytesExpect
#define BYTES_LEN sizeof(bytesExpect)

// minimum off duration of a header pulse (less than this is a one bit or zero bit)
#define HEADER_PULSE_MICROS 2000
// minimum off duration of a one bit (less than this is a zero bit)
#define ON_BIT_MICROS 1000

// Pulse collection:

// duration in microseconds to wait when polling pulses
#define RESOLUTION_MICROS 20
// the maximum number of pulses to be buffered, should be size of 'pulses'
#define MAX_NUM_PULSES 100
// the maximum pulse value we'll listen for, should be less than uint16_t in 'pulses'
#define MAX_PULSE_DURATION 65000
// pulses[i][0] = high duration, pulses[i][1] = low duration (in number of RESOLUTION_MICROS)
uint16_t pulses[MAX_NUM_PULSES][2];
// current index in pulses
uint8_t currentPulse = 0;

// minimum duration in milliseconds to wait between power switches
#define SWITCH_MIN_WAIT_MILLIS 3000
// current power status
bool power = false;
// time in millis since last power flip
uint32_t lastFlipMillis = 0;

void setup(void) {
  pinMode(powerPin, OUTPUT);
  digitalWrite(powerPin, HIGH); // disable power to start
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
  Serial.print("Ready to decode IR!");
}

void loop(void) {
  uint16_t duration = 0;

  // get duration of HIGH value
  while (irPin_PIN & (1 << irPin)) {
    duration++;
    delayMicroseconds(RESOLUTION_MICROS);

    if ((duration >= MAX_PULSE_DURATION) && (currentPulse != 0)) {
      // this pulse is too long -- give up
      Serial.println("\nExiting long high poll");
      parsePulses();
      return;
    }
  }
  // record duration of HIGH value
  pulses[currentPulse][0] = duration;

  // get duration of LOW value
  duration = 0;
  while (! (irPin_PIN & _BV(irPin))) {
    duration++;
    delayMicroseconds(RESOLUTION_MICROS);

    if ((duration >= MAX_PULSE_DURATION) && (currentPulse != 0)) {
      // this pulse is too long -- give up
      Serial.println("\nExiting long low poll");
      parsePulses();
      return;
    }
  }
  // record duration of LOW value
  pulses[currentPulse][1] = duration;

  // continue to next HIGH/LOW cycle
  if (++currentPulse == MAX_NUM_PULSES) {
    // too much data, flush
    Serial.println("\nBuffer full, exit");
    parsePulses();
  }
}

void parsePulses(void) {
  if (!pulsesMatch()) {
    return;
  }
  uint32_t now = millis();
  if (now < lastFlipMillis) {
    // time has wrapped around (~50 days), reset state
    lastFlipMillis = 0;
  }
  if (lastFlipMillis == 0 || now - lastFlipMillis > SWITCH_MIN_WAIT_MILLIS) {
    if (power) {
      Serial.println("Disabling power.");
      digitalWrite(powerPin, HIGH);
      digitalWrite(ledPin, LOW);
    } else {
      Serial.println("Enabling power.");
      digitalWrite(powerPin, LOW);
      digitalWrite(ledPin, HIGH);
    }
    power = !power;
    lastFlipMillis = now;
  } else {
    Serial.println("Ignoring power message: Too soon since last switch.");
  }
}

bool pulsesMatch(void) {
  if (currentPulse < BYTES_LEN * 8) {
    Serial.print("Ignoring ");
    Serial.print(currentPulse, DEC);
    Serial.print(" pulses: too short for ");
    Serial.print(BYTES_LEN, DEC);
    Serial.println(" expected bytes.");
    currentPulse = 0;
    return false;
  }
  Serial.print("Received ");
  Serial.print(currentPulse, DEC);
  Serial.println(" pulses:");
  for (uint8_t i = 0; i < BYTES_LEN; ++i) {
    bytesGot[i] = 0;
  }
  uint8_t value = 0; // accumulated value
  uint8_t valueBits = 0; // number of bits included in 'value'
  uint8_t nextBytesGotIndex = 0;
  for (uint8_t i = 0; i < currentPulse; ++i) {
    uint32_t micros = pulses[i][0] * RESOLUTION_MICROS;
    if (micros > HEADER_PULSE_MICROS) {
      // ignore header
      valueBits = 0;
      continue;
    }
    value <<= 1;
    if (micros > ON_BIT_MICROS) {
      value |= 0x1;
    }
    if (++valueBits % 8 == 0) {
      bytesGot[nextBytesGotIndex] = value;
      if (++nextBytesGotIndex >= BYTES_LEN) {
        uint8_t ignoredPulses = currentPulse - i - 1;
        if (ignoredPulses > 0) {
          Serial.print("Ignoring ");
          Serial.print(ignoredPulses, DEC);
          Serial.println(" extra pulses");
        }
        break;
      }
    }
  }
  bool allMatch = true;
  for (uint8_t i = 0; i < BYTES_LEN; ++i) {
    Serial.print("byte ");
    Serial.print(i, DEC);
    Serial.print(": got ");
    if (bytesExpect[i] == bytesGot[i]) {
      Serial.print(bytesGot[i], BIN);
      Serial.println(" (MATCH)");
    } else {
      Serial.print(bytesGot[i], BIN);
      Serial.print(" expected ");
      Serial.print(bytesExpect[i], BIN);
      Serial.println(" (MISMATCH)");
      allMatch = false;
    }
  }
  currentPulse = 0;
  return allMatch;
}

