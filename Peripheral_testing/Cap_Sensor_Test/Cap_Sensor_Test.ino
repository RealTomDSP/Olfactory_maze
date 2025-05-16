#include <Wire.h>
#include "FDC1004.h"

FDC1004 sensor;

// Sensor channels
const uint8_t SENSOR1_CHANNEL = 0;  // CIN1 LEFT NOSE POKE
const uint8_t SENSOR2_CHANNEL = 3;  // CIN2 RIGHT NOSE POKE

const uint8_t CAPDAC = 0;

// Shield assignments (if using them physically)
const uint8_t SHIELD1 = 1;  // SHLD1
const uint8_t SHIELD2 = 2;  // SHLD2

// Adjust thresholds as needed
const float TOUCH_THRESHOLD_1 = 5.4;  
const float TOUCH_THRESHOLD_2 = 5.4;


// Track previous touch states
bool wasTouched1 = false;
bool wasTouched2 = false;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Configure each sensor channel with its respective shield
  // If you need shields explicitly, see library's `configureMeasurement` function
  sensor.configureMeasurementSingle(0, SENSOR1_CHANNEL, CAPDAC); // Meas ID=0 → CIN1
  sensor.configureMeasurementSingle(1, SENSOR2_CHANNEL, CAPDAC); // Meas ID=1 → CIN2




  Serial.println("Touch sensors ready:");
}

void loop() {
  // --- SENSOR 1 ---
  uint16_t rawValue1;
  float capacitance1;

  if (sensor.measureChannel(SENSOR1_CHANNEL, CAPDAC, &rawValue1) == 0) {
    // Convert raw data to pF
    capacitance1 = ((int16_t)rawValue1) / 5242.88;
    bool isTouched1 = capacitance1 > TOUCH_THRESHOLD_1;

    // Detect state change for Sensor 1
    if (isTouched1 && !wasTouched1) {
      Serial.print("Sensor 1: Touch detected! pF = ");
      Serial.println(capacitance1, 2); // prints 2 decimal places
    } 
    else if (!isTouched1 && wasTouched1) {
      Serial.print("Sensor 1: Touch released! pF = ");
      Serial.println(capacitance1, 2);
    }
  }

  // --- SENSOR 2 ---
  uint16_t rawValue2;
  float capacitance2;

  if (sensor.measureChannel(SENSOR2_CHANNEL, CAPDAC, &rawValue2) == 0) {
    // Convert raw data to pF
    capacitance2 = ((int16_t)rawValue2) / 5242.88;
    bool isTouched2 = capacitance2 > TOUCH_THRESHOLD_2;

    // Detect state change for Sensor 2
    if (isTouched2 && !wasTouched2) {
      Serial.print("Sensor 2: Touch detected! pF = ");
      Serial.println(capacitance2, 2);
    } 
    else if (!isTouched2 && wasTouched2) {
      Serial.print("Sensor 2: Touch released! pF = ");
      Serial.println(capacitance2, 2);
    }
  }

  delay(100);  // adjust for responsiveness as needed
}

