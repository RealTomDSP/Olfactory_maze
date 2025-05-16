/* Tom Stowell

Entity: Alternation Maze with Scent & Hold Task
Description: Builds on alternation with capacitive sensor maze
- behaviour is unchanged alternation enforced, capacitive touch and hold
- refactored for non-blocking motor control
- added scent delivery: triggers a random scent motor (1 second) at nose-poke 
  (without blocking hold detection)
  since the motor delivery is random, the same scent should be loaded for both motors per side
Notes:
- right side delivers orange scent & left side delivering coconut scent
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "FDC1004.h"

//ir beam declaration (active low)
#define BEAM_BROKEN(pin) (!digitalRead(pin))   

Adafruit_MotorShield motorShieldLeft  = Adafruit_MotorShield(0x60);
Adafruit_MotorShield motorShieldRight = Adafruit_MotorShield(0x61);
Adafruit_DCMotor *leftRewardMotor     = motorShieldLeft.getMotor(3);  // M3: left reward 
Adafruit_DCMotor *rightRewardMotor    = motorShieldRight.getMotor(3); // M6: right reward
//scent delivery motors added on each side
Adafruit_DCMotor *leftScentMotor1     = motorShieldLeft.getMotor(1);  // M1:left scent 1
Adafruit_DCMotor *leftScentMotor2     = motorShieldLeft.getMotor(2);  // M2: left scent 2
Adafruit_DCMotor *rightScentMotor1    = motorShieldRight.getMotor(1); // M4: right scent 1
Adafruit_DCMotor *rightScentMotor2    = motorShieldRight.getMotor(2); // M5: right scent 2

const unsigned long motorRunDuration = 300;    // run reward motors for 300 ms
const unsigned long scentRunDuration = 1000;   // run scent motors for 1000 ms
unsigned long m3StartTime = 0;
unsigned long m6StartTime = 0;
// timestamps for scent motors
unsigned long m1StartTime = 0;
unsigned long m2StartTime = 0;
unsigned long m4StartTime = 0;
unsigned long m5StartTime = 0;
int m3Retries        = 0;
int m6Retries        = 0;
const int maxRetries = 3;
// flags for scent motors running state
bool m1Running       = false;
bool m2Running       = false;
bool m3Running       = false;
bool m4Running       = false;
bool m5Running       = false;
bool m6Running       = false;

// non blocking motor activation parameters
const unsigned long motorDelayAfterBeep = 50;  //delay after piezo beep before motor starts
const unsigned int  rewardBeepDuration  = 500;  //piezo beep duration for reward
unsigned long motorScheduledTime        = 0;
//motor flags
bool motorScheduled     = false;
bool motorScheduledLeft = false;

//pin declarations
const uint8_t LEFT_COMMIT_PIN  = 2;
const uint8_t LEFT_NOSE_PIN    = 3;
const uint8_t CENTER_PIN       = 4;
const uint8_t RIGHT_COMMIT_PIN = 5;
const uint8_t RIGHT_NOSE_PIN   = 7;
const uint8_t PIEZO_RIGHT_PIN  = 9;
const uint8_t PIEZO_LEFT_PIN   = 10;

// capacitive sensing 
FDC1004 sensor;
const uint8_t SENSOR_LEFT_CH  = 0;  // CIN1
const uint8_t SENSOR_RIGHT_CH = 3;  // CIN2
const uint8_t CAPDAC          = 0;
const float THRESH_LEFT       = 5.4; 
const float THRESH_RIGHT      = 5.4;
bool leftTouched              = false;
bool rightTouched             = false;

// Async FDC1004 sequencing
static uint8_t  fStage      = 0; // 0 trigger L, 1 read L & trigger R, 2 read R
static unsigned long fStamp = 0;
const uint16_t FDC_MS       = 16; // ≥16 ms between trigger & read

// state machine declarations
enum State { 
  WAIT_COMMIT,
  PENDING_REWARD_LEFT, PENDING_REWARD_RIGHT,
  LEFT_SUCCESS, RIGHT_SUCCESS, LEFT_FAILED, RIGHT_FAILED 
};
State state         = WAIT_COMMIT;   
uint8_t requireSide = 0;//either side on lap 1

// task timing variables 
unsigned long requiredHoldTime        = 100; // accum 12 ms per lap (up to HOLD_MAX)
const unsigned long HOLD_INC          = 12;
const unsigned long HOLD_MAX          = 700; //700ms max
const unsigned long DECISION_TIMEOUT  = 30000;

//lap handling variables
int lapCount        = 0;
int successCt       = 0; 
int failCt          = 0;
int nextSideAllowed = 0;   // 0=either; 1=must left; 2=must right

// edge-detection bookkeeping for IR beams
static bool prevLcommit = false;
static bool prevRcommit = false;
static bool prevCenter  = false;
static bool prevLnose   = false;
static bool prevRnose   = false;

//function declarations
void failLap(bool wasLeft);
void updateMotors();

void setup() {
  Serial.begin(9600);
  if (!motorShieldLeft.begin() || !motorShieldRight.begin()) {
    Serial.println(F("Motor-shield init FAIL – check I2C addresses!"));
    while(true); // halt if motor shields not found
  }
  randomSeed(analogRead(A0)); // NEW: seed random generator for scent motor selection
  // Configure pin modes for IR beam sensors and piezos
  pinMode(LEFT_COMMIT_PIN, INPUT_PULLUP);
  pinMode(RIGHT_COMMIT_PIN, INPUT_PULLUP);
  pinMode(LEFT_NOSE_PIN,   INPUT_PULLUP);
  pinMode(RIGHT_NOSE_PIN,  INPUT_PULLUP);
  pinMode(CENTER_PIN,      INPUT_PULLUP);
  pinMode(PIEZO_LEFT_PIN,  OUTPUT);
  pinMode(PIEZO_RIGHT_PIN, OUTPUT);

  Wire.begin();
  // initialize capacitive sensor measurements for left/right channels
  sensor.configureMeasurementSingle(0, SENSOR_LEFT_CH,  CAPDAC);
  sensor.configureMeasurementSingle(1, SENSOR_RIGHT_CH, CAPDAC);

  Serial.println(F("System Initialized - Let's Go!"));
}

void loop() {
  // Read sensors (capacitive touch and IR beams) asynchronously
  readSensors();
  updateMotors();  // update motor states (start/stop) without blocking

  // Read IR break-beam sensors for current loop iteration
  bool Lcommit = BEAM_BROKEN(LEFT_COMMIT_PIN);
  bool Rcommit = BEAM_BROKEN(RIGHT_COMMIT_PIN);
  bool Lnose   = BEAM_BROKEN(LEFT_NOSE_PIN);
  bool Rnose   = BEAM_BROKEN(RIGHT_NOSE_PIN);
  bool Center  = BEAM_BROKEN(CENTER_PIN);

  // edge detection - true only on the transition from unbroken to broken
  bool LcommitEdge = Lcommit && !prevLcommit;
  bool RcommitEdge = Rcommit && !prevRcommit;
  bool LnoseEdge   = Lnose   && !prevLnose;
  bool RnoseEdge   = Rnose   && !prevRnose;
  bool CenterEdge  = Center  && !prevCenter;
  prevLcommit = Lcommit;
  prevRcommit = Rcommit;
  prevCenter  = Center;
  prevLnose   = Lnose;
  prevRnose   = Rnose;

  static unsigned long stateStart = 0;  //timestamp when entering a new state

  // state machine start
  switch(state) {
  // handle center beam and commit point logic 
    case WAIT_COMMIT:
  //initial lap allows for commit on either left or right 
  if (requireSide == 0) {
        // if committed left
        if (LcommitEdge) {    
            lapCount = 1;
            Serial.println(F("Commit Point: Left "));
            nextSideAllowed = 2;   // next lap must go right
            requireSide     = 2;
            stateStart = millis();
            state = PENDING_REWARD_LEFT;
        }
        else if (RcommitEdge) {
            lapCount = 1;
            // chose Right
            Serial.println(F("Commit Point: Right "));
            nextSideAllowed = 1;
            requireSide     = 1;
            stateStart = millis();
            state = PENDING_REWARD_RIGHT;
        }
        break;
      }

  // enforce alternation after initial lap 
      if (requireSide == 1) {  // must go left this lap
        if (LcommitEdge) {
            Serial.println(F("Commit Point: Left"));
            nextSideAllowed = 2;
            requireSide     = 2;
            stateStart = millis();
            state = PENDING_REWARD_LEFT;
        } else if (RcommitEdge) {
            Serial.println(F("Did not alternate"));
            failLap(true);  // failed to commit left
        }
      } else  {   
        // expecting right commit trigger
        if (RcommitEdge) {
            Serial.println(F("Commit Point: Right"));
            nextSideAllowed = 1;
            requireSide     = 1;
            stateStart = millis();
            state = PENDING_REWARD_RIGHT;
        } else if (LcommitEdge) {
            Serial.println(F("Did not alternate"));
            failLap(false);     // failed to commit right
        }
      }
      break;

    case PENDING_REWARD_LEFT:
      // non blocking logic when reward motor triggered
      if (m3Running) break;
      if (CenterEdge) { 
        Serial.println(F("Skipped task")); 
        failLap(true);   // failed by leaving before nose-poke
      } 
      else if (LnoseEdge) {
        // trigger random scent delivery motor (left side) on nosepoke
        int motorChoice = random(1, 3);  // randomly select motor 1 or 2 for left side
        if (motorChoice == 1) {
          leftScentMotor1->setSpeed(150);
          leftScentMotor1->run(FORWARD);
          m1Running = true;
          m1StartTime = millis();
          Serial.println(F("Odor release: Left scent motor 1 activated (1000ms)"));
        } else {
          leftScentMotor2->setSpeed(150);
          leftScentMotor2->run(FORWARD);
          m2Running = true;
          m2StartTime = millis();
          Serial.println(F("Odor release: Left scent motor 2 activated (1000ms)"));
        }
        rewardRoutine(true); // process left nose-poke for reward (capacitive hold)
      } 
      else if (millis() - stateStart > DECISION_TIMEOUT) {
        Serial.println(F("Timeout"));
        failLap(true);  // no nose-poke in time
      }
      break;

    case PENDING_REWARD_RIGHT:
      if (m6Running) break;
      if (CenterEdge) { 
        Serial.println(F("Skipped task")); 
        failLap(false); 
      } 
      else if (RnoseEdge) {
        // trigger random scent delivery motor (right side) on nosepoke
        int motorChoice = random(4, 6);  // randomly select motor 4 or 5 for right side
        if (motorChoice == 4) {
          rightScentMotor1->setSpeed(150);
          rightScentMotor1->run(FORWARD);
          m4Running = true;
          m4StartTime = millis();
          Serial.println(F("Odor release: Right scent motor 4 activated (1000ms)"));
        } else {
          rightScentMotor2->setSpeed(150);
          rightScentMotor2->run(FORWARD);
          m5Running = true;
          m5StartTime = millis();
          Serial.println(F("Odor release: Right scent motor 5 activated (1000ms)"));
        }
        rewardRoutine(false);// process right nose-poke for reward (capacitive hold)
      } 
      else if (millis() - stateStart > DECISION_TIMEOUT) {
        Serial.println(F("Timeout"));
        failLap(false);
      }
      break;

    case LEFT_SUCCESS:
    case RIGHT_SUCCESS:
      if (CenterEdge) {
        startNextLapPass();
      }
      break;

    case LEFT_FAILED:
    case RIGHT_FAILED:
      if (CenterEdge) {
        startNextLapFail();
      }
      break;
  }
}

// readSensors function
// asyncrhonous reading from FDC1004 capacitive sensor
// triggers and reads capacitive sensor measurements in a non-blocking sequence
void readSensors() {
  switch (fStage) {
    case 0: // trigger left channel measurement
      sensor.triggerSingleMeasurement(0, FDC1004_100HZ);
      fStamp = millis();
      fStage = 1;
      break;
    case 1: // read left result (after >=16 ms) and trigger right channel
      if (millis() - fStamp < FDC_MS) return;
      { 
        uint16_t r[2];
        if (sensor.readMeasurement(0, r) == 0) {
          float c = ((int16_t)r[0]) / 5242.88;
          leftTouched = (c > THRESH_LEFT);
        }
      }
      sensor.triggerSingleMeasurement(1, FDC1004_100HZ);
      fStamp = millis();
      fStage = 2;
      break;
    case 2: // read right result (after >=16 ms)
      if (millis() - fStamp < FDC_MS) return;
      {
        uint16_t r[2];
        if (sensor.readMeasurement(1, r) == 0) {
          float c = ((int16_t)r[0]) / 5242.88;
          rightTouched = (c > THRESH_RIGHT);
        }
      }
      fStage = 0;
      break;
  }
}

// rewardroutine function
// note: uses blocking reads of FDC1004 for timing during hold detection)
void rewardRoutine(bool isLeft) {
  const uint8_t piezo = isLeft ? PIEZO_LEFT_PIN : PIEZO_RIGHT_PIN;
  Adafruit_DCMotor *pump = isLeft ? leftRewardMotor : rightRewardMotor;
  const uint8_t motorID = isLeft ? 3 : 6;
  const uint8_t ch = isLeft ? SENSOR_LEFT_CH : SENSOR_RIGHT_CH;
  const float thr = isLeft ? THRESH_LEFT : THRESH_RIGHT;

  unsigned long touchStart = 0;
  bool touching = false;
  unsigned long waitStart = millis();
  const unsigned long TOTAL_TIMEOUT = 15000;

  Serial.print(F("Nose-poke: "));
  Serial.println(isLeft ? F("Left") : F("Right"));
  Serial.print(F("Waiting hold ≥ "));
  Serial.print(requiredHoldTime);
  Serial.println(F(" ms"));

  while (true) {
    // capacitive sensing (blocking read for quick feedback)
    uint16_t raw[2];
    bool got = (sensor.measureChannel(ch, CAPDAC, raw) == 0);
    bool contact = false;
    if (got) {
      float cap = ((int16_t)raw[0]) / 5242.88;
      contact = (cap > thr);
    }

    // handle touch timing (check if contact made for requiredHoldTime)
    if (!touching) {
      if (contact) {
        touching = true;
        touchStart = millis();
        Serial.println(F("Contact : Time Start"));
      }
    } else {
      if (!contact) {
        Serial.println(F("Contact Lost"));
        touching = false;
      }
      else if (millis() - touchStart >= requiredHoldTime) {
        // only trigger reward after successful hold duration
        Serial.println(F("Hold completed : Pass "));

        // reward sequence
        // play tone and schedule motor activation
        tone(piezo, 4000, rewardBeepDuration);
        motorScheduled = true;
        motorScheduledLeft = isLeft;
        motorScheduledTime = millis();
        // motor will actually run after motorDelayAfterBeep via updateMotors

        // update state and exit
        successCt++;
        state = isLeft ? LEFT_SUCCESS : RIGHT_SUCCESS;
        return;
      }
    }

    // handle timeout for nose-poke hold
    if (millis() - waitStart > TOTAL_TIMEOUT) {
      Serial.println(F("Reward timeout"));
      failLap(isLeft);
      return;
    }

    delay(5); // small delay to avoid CPU hogging and sensor flooding
    // NEW: update motor states during hold (allow scent motor to run)
    updateMotors();
  }
}

//startnextlappass function
//lap counter for successful lap (add time to gavage contact)
void startNextLapPass() {
  lapCount++;
  Serial.print(F("Lap "));              
  Serial.println(lapCount);

  // increase requires contact time with gavage
  if (requiredHoldTime < HOLD_MAX) {
      requiredHoldTime += HOLD_INC;
      if (requiredHoldTime > HOLD_MAX) requiredHoldTime = HOLD_MAX;
  }

  //check commit side 0 = either, 1 = must-left, 2 = must-right
  requireSide = nextSideAllowed;
  state       = WAIT_COMMIT;// new lap goes into the single wait_commit state
}

//startnextlapfail function (does not increase gavage hold time)
void startNextLapFail() {
  lapCount++;
  Serial.print(F("Lap "));              
  Serial.println(lapCount);

//check which commit point is expected
requireSide = nextSideAllowed;
  state       = WAIT_COMMIT;            // straight into the single wait state
}

// updatemotors function
// handles scheduled motor activation and timing
void updateMotors() {
  unsigned long currentTime = millis();

  // waits to activate motors after auditory cue from piezo 
  if (motorScheduled && (currentTime - motorScheduledTime >= motorDelayAfterBeep)) {
    Adafruit_DCMotor *pumpToStart = motorScheduledLeft ? leftRewardMotor : rightRewardMotor;
    pumpToStart->setSpeed(150);
    pumpToStart->run(FORWARD);
    // mark motor as running and record start time
    if (motorScheduledLeft) {
      m3Running = true;
      m3StartTime = currentTime;
    } else {
      m6Running = true;
      m6StartTime = currentTime;
    }
    motorScheduled = false;
  }

  // left scent motors - stop after scent run duration
  if (m1Running && (currentTime - m1StartTime >= scentRunDuration)) {
    leftScentMotor1->run(RELEASE);
    m1Running = false;
  }
  if (m2Running && (currentTime - m2StartTime >= scentRunDuration)) {
    leftScentMotor2->run(RELEASE);
    m2Running = false;
  }
  // left reward motor - stop after motor run duration
  if (m3Running && (currentTime - m3StartTime >= motorRunDuration)) {
    leftRewardMotor->run(RELEASE);
    m3Running = false;
    //Serial.println(F("M3 stopped"));
  }

  // right scent motors - stop after scent run duration
  if (m4Running && (currentTime - m4StartTime >= scentRunDuration)) {
    rightScentMotor1->run(RELEASE);
    m4Running = false;
  }
  if (m5Running && (currentTime - m5StartTime >= scentRunDuration)) {
    rightScentMotor2->run(RELEASE);
    m5Running = false;
  }
  // right reward motor - stop after motor run duration
  if (m6Running && (currentTime - m6StartTime >= motorRunDuration)) {
    rightRewardMotor->run(RELEASE);
    m6Running = false;
    // Serial.println(F("M6 stopped"));
  }
}

// faillap logs lap as failure
void failLap(bool wasLeft) {
  failCt++;
  Serial.print(F("Lap "));
  Serial.print(lapCount);
  Serial.println(F(" - Fail"));
  state = wasLeft ? LEFT_FAILED : RIGHT_FAILED;
  // reset edge-detection flags for commit sensors - ready for next lap 
  prevLcommit = prevRcommit = false;
}
