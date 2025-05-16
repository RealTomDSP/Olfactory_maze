

#include <Wire.h>

const int beamPins[] = {2, 3, 4, 5, 7};
const int numBeams = sizeof(beamPins) / sizeof(beamPins[0]);

bool lastBeamStates[numBeams];

void setup() {
  Serial.begin(9600);

  // Set up beam sensor pins as input with pull-up resistors
  for (int i = 0; i < numBeams; i++) {
    pinMode(beamPins[i], INPUT_PULLUP);
    lastBeamStates[i] = digitalRead(beamPins[i]);  // Initialize with current state
  }
}

void loop() {
  for (int i = 0; i < numBeams; i++) {
    bool currentState = digitalRead(beamPins[i]);

    if (currentState != lastBeamStates[i]) {
      if (currentState == LOW) {
        Serial.print("Beam on pin ");
        Serial.print(beamPins[i]);
        Serial.println(" is BROKEN");
      } else {
        Serial.print("Beam on pin ");
        Serial.print(beamPins[i]);
        Serial.println(" is UNBROKEN");
      }

      lastBeamStates[i] = currentState;  // Update the stored state
    }
  }

  delay(50);  // Small debounce delay
}
