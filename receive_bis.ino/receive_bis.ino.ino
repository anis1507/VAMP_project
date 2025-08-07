#include <Servo.h>

const int DATA_PIN = 1;  // TX = D1
const int STATUS_LED_PIN = LED_BUILTIN;

Servo servos[6];

bool syncEstablished = false;
uint8_t receivedAngles[5];
int receivedByteIndex = 0;

uint8_t readByte() {
  // Wait for start bit
  while (digitalRead(DATA_PIN) == 1);  // wait for falling edge

  delayMicroseconds(1000);  // half-bit delay to center align

  uint8_t value = 0;
  for (int i = 0; i < 8; i++) {
    delayMicroseconds(2000);  // wait for each bit
    int bit = digitalRead(DATA_PIN);
    value |= (bit << i);
  }

  // Wait for stop bit
  delayMicroseconds(2000);
  return value;
}

void applyReceivedAngles() {
  servos[0].write(90);  // fixed position for base (optional)
  for (int i = 0; i < 5; ++i)
    servos[5 - i].write(constrain(receivedAngles[i], 0, 180));
}

void resetSyncState() {
  syncEstablished = false;
  receivedByteIndex = 0;
  digitalWrite(STATUS_LED_PIN, LOW);
}

void setup() {
  pinMode(DATA_PIN, INPUT_PULLUP);
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  for (int i = 0; i < 6; ++i) {
    servos[i].attach(i + 2);  // Pins D2 to D7
    servos[i].write(90);      // Initial position
  }

}

void loop() {
  uint8_t incomingByte = readByte();

  if (!syncEstablished) {
    if (incomingByte == 181) {
      syncEstablished = true;
      receivedByteIndex = 0;
      digitalWrite(STATUS_LED_PIN, HIGH);
    }
  } else {
    if (incomingByte == 255) {
      if (receivedByteIndex == 5) {
        applyReceivedAngles();
      }
      resetSyncState();
    } else if (incomingByte <= 180) {
      if (receivedByteIndex < 5) {
        receivedAngles[receivedByteIndex++] = incomingByte;
      } else {
        resetSyncState();
      }
    } else {
      resetSyncState();
    }
  }
}
