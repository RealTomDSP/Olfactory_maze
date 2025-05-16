// T Stowell
// Motor controller
// Works in tandem with arduino uno microcontroller and python script (in jupyter notebook)

#include <Wire.h>
#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMSLeft(0x60);
Adafruit_MotorShield AFMSRight(0x61);

Adafruit_DCMotor* motors[6] = {
  AFMSLeft.getMotor(1),  // M1: Scent 1 Right
  AFMSLeft.getMotor(2),  // M2: Scent 2 Right
  AFMSLeft.getMotor(3),  // M3: Reward Right
  AFMSRight.getMotor(1), // M4: Scent 1 Left
  AFMSRight.getMotor(2), // M5: Scent 2 Left
  AFMSRight.getMotor(3)  // M6: Reward Left
};

void setup() {
  Serial.begin(9600);
  AFMSLeft.begin();
  AFMSRight.begin();
  
  for(int i=0; i<6; i++) {
    motors[i]->setSpeed(150);
  }
}

void loop() {
  if(Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if(cmd.length() == 2) {
      char action = cmd[0];
      int motorNum = cmd[1] - '1'; // Convert to 0-5 index
      
      if(motorNum >=0 && motorNum <6) {
        if(action == 'S') {
          motors[motorNum]->run(FORWARD);
        }
        else if(action == 'T') {
          motors[motorNum]->run(RELEASE);
        }
      }
    }
  }
}
