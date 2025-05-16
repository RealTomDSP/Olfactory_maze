/* Tom Stowell
  
Entity: Alternation Maze with Multiple Tasks and Scents
Description: Combines "Hold" and "Withdraw" task logic in alternation maze 
- Two task types per side: HOLD or WITHDRAW, indicated by which odor pump is activated
   Left side: M1 = hold task (coconut), M2 = withdraw task (orange)
   Right side: M4 = hold task (orange), M5 = withdraw task (coconut)
   M3 and M6 are reward delivery pumps for left and right
- Odor delivery - On nose-poke a random odor motor on that side runs for 1s
   Each side's two odor motors will not run more than 3 times consecutively (to prevent overusing one odor/task)
- alternation enforced:
   First trial can start on either side, but thereafter the opposite side is required on the next trial. Upon committing to the wrong side, the trial fails
- HOLD task behavior:
   Animal must maintain contact with gavage for at least `requiredHoldTime` which increments in time per successful lap (while the nose-poke IR beam is broken)
- WITHDRAW task behavior: 
   Animal must withdraw from the nose-poke (clear the nose IR beam) within 2 seconds of nose entry, and avoid touching the gavage for at least 1 second after nose-poke entry
   Successful withdrawal triggers a short success tone, after which the animal must re-enter make contact for their reward
- Trial outcomes:
   Success: correct behavior performed for the selected task, triggers reward
   Failure: wrong behavior (e.g., touching gavage too early in a withdraw trial, or not holding long enough in a hold trial), failing to alternate sides, or skipping the task
   Failures require the center beam to be broken to reset for the next trial
  - Logging: 
    Each trial logs Side (Left/Right), selected motor (corresponds to associated task) , task type (hold or withdraw), and outcome (pass or fail)
  
  Hardware:
  - FDC1004 capacitive sensor for detecting gavage touch (left and right channels)
  - Adafruit Motor Shield v2 (two shields at I2C 0x60 and 0x61) controlling 6 DC motors:
      M1, M2 = odor pumps on left side (M1 for hold-task odor, M2 for withdraw-task odor)
      M3 = reward pump on left side
      M4, M5 = odor pumps on right side (M4 for hold-task odor, M5 for withdraw-task odor)
      M6 = reward pump on right side
  - Infrared break-beam sensors:
      LEFT_COMMIT_PIN, RIGHT_COMMIT_PIN at maze entry (commit point for each side)
      LEFT_NOSE_PIN, RIGHT_NOSE_PIN at nose-poke (near gavage)
      CENTER_PIN at the maze center (to detect when the animal returns to center to start the next trial)
  - Piezo buzzers (one on each side) for auditory feedback (reward cue)
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

//pin declarations (IRs are active low)
const uint8_t LEFT_COMMIT_PIN  = 2;
const uint8_t LEFT_NOSE_PIN    = 3;
const uint8_t CENTER_PIN       = 4;
const uint8_t RIGHT_COMMIT_PIN = 5;
const uint8_t RIGHT_NOSE_PIN   = 7;
const uint8_t PIEZO_RIGHT_PIN  = 9;
const uint8_t PIEZO_LEFT_PIN   = 10;

//capacitive sensing 
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

// State machine definitions for trial progression
enum State { 
  WAIT_COMMIT,
  PENDING_REWARD_LEFT, PENDING_REWARD_RIGHT,
  LEFT_SUCCESS, RIGHT_SUCCESS,
  LEFT_FAILED, RIGHT_FAILED
};
State state         = WAIT_COMMIT;
uint8_t requireSide = 0;    // required side this trial: 0 = either (no alternation yet), 1 = must-left, 2 = must-right

// hold task timing variables 
unsigned long requiredHoldTime        = 100; // accum 12 ms per lap (up to HOLD_MAX)
const unsigned long HOLD_INC          = 12;
const unsigned long HOLD_MAX          = 700; //700ms max
const unsigned long DECISION_TIMEOUT  = 30000;
// withdraw task update declarations
const unsigned long WITHDRAW_TIMEOUT = 4000; // time to exit nosepoke after beam break
const unsigned long EXCLUSION_TIME   = 2000;  // buffer for early gavage contact 

// lap handling variables
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

// Variables for odor motor selection and task tracking
uint8_t lastLeftTaskType  = 0;   // last task type used on left side (1 = Hold, 2 = Withdraw)
uint8_t lastRightTaskType = 0;  // last task type used on right side (1 = Hold, 2 = Withdraw)
uint8_t leftRepeatCount   = 0;    // consecutive left-side trials using the same task type
uint8_t rightRepeatCount  = 0;   // consecutive right-side trials using the same task type
uint8_t currentMotorID    = 0;   // motor ID (M1–M5) selected for odor in the current trial
bool currentSideIsLeft     = false;   // side of the current trial (set when commit beam breaks)
bool currentTaskIsWithdraw = false; // task type of current trial: true = Withdraw, false = Hold

// functions declarations
void updateMotors();
void failLap(bool wasLeft);
void startNextLapPass();
void startNextLapFail();
void readSensors();
void doHoldTask(bool isLeft);
void doWithdrawTask(bool isLeft);

void setup() {
  Serial.begin(9600);
  // initialize motor shields
  if (!motorShieldLeft.begin() || !motorShieldRight.begin()) {
    Serial.println(F("Motor-shield init FAIL – check I2C addresses!"));
    while (true); // halt if motor shields not found
  }
  randomSeed(analogRead(A0));  //seed random number generator for odor motor selection
  
  // pin modes for IR sensors and piezos
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
  Serial.println(F("System Initialized - Let's Go"));
}

void loop() {
  // read sensors asynchronously
  readSensors();
  updateMotors();
  
  // current state of IR break-beam sensors
  bool Lcommit = BEAM_BROKEN(LEFT_COMMIT_PIN);
  bool Rcommit = BEAM_BROKEN(RIGHT_COMMIT_PIN);
  bool Lnose   = BEAM_BROKEN(LEFT_NOSE_PIN);
  bool Rnose   = BEAM_BROKEN(RIGHT_NOSE_PIN);
  bool Center  = BEAM_BROKEN(CENTER_PIN);
  
  // edge detection: true only on the transition from unbroken to broken
  bool LcommitEdge = Lcommit && !prevLcommit;
  bool RcommitEdge = Rcommit && !prevRcommit;
  bool LnoseEdge   = Lnose   && !prevLnose;
  bool RnoseEdge   = Rnose   && !prevRnose;
  bool CenterEdge  = Center  && !prevCenter;
  
  // store current sensor states for next iteration edge detection
  prevLcommit = Lcommit;
  prevRcommit = Rcommit;
  prevCenter  = Center;
  prevLnose   = Lnose;
  prevRnose   = Rnose;
  
  static unsigned long stateStart = 0;  // timestamp when entering a new state (for timeouts)
  
  // state machine for trial logic
  switch (state) {
    case WAIT_COMMIT:
      // waiting for the animal to commit to either side 
      if (requireSide == 0) {
        // first trial: accept commitment from either side
        if (LcommitEdge) {
          lapCount = 1;
          currentSideIsLeft = true;
          currentMotorID = 0;           // no odor motor selected yet
          currentTaskIsWithdraw = false;
          Serial.println(F("Commit Point: Left"));
          nextSideAllowed = 2;         // next trial must go to rright
          requireSide     = 2;
          stateStart = millis();
          state = PENDING_REWARD_LEFT;
        } else if (RcommitEdge) {
          lapCount = 1;
          currentSideIsLeft = false;
          currentMotorID = 0;
          currentTaskIsWithdraw = false;
          Serial.println(F("Commit Point: Right"));
          nextSideAllowed = 1;         // next trial must go to left
          requireSide     = 1;
          stateStart = millis();
          state = PENDING_REWARD_RIGHT;
        }
        break;
      }
      // enforce alternation 
      if (requireSide == 1) {
        // trial must be on left
        if (LcommitEdge) {
          Serial.println(F("Commit Point: Left"));
          currentSideIsLeft = true;
          currentMotorID = 0;
          currentTaskIsWithdraw = false;
          nextSideAllowed = 2;        // after this trial next must be right
          requireSide     = 2;
          stateStart = millis();
          state = PENDING_REWARD_LEFT;
        } else if (RcommitEdge) {
          // wrong side (went r instead of l) fail lap 
          Serial.println(F("Did not alternate (went Right instead of Left)"));
          currentSideIsLeft = false;
          currentMotorID = 0;
          currentTaskIsWithdraw = false;
          failLap(true);  // treat as left failed (did not alternate)
        }
      } else {  // requireSide == 2
        // must be on right
        if (RcommitEdge) {
          Serial.println(F("Commit Point: Right"));
          currentSideIsLeft = false;
          currentMotorID = 0;
          currentTaskIsWithdraw = false;
          nextSideAllowed = 1;        // after this trial next must be left
          requireSide     = 1;
          stateStart = millis();
          state = PENDING_REWARD_RIGHT;
        } else if (LcommitEdge) {
          // wrong side (went l instead of r) fail lap
          Serial.println(F("Did not alternate (went Left instead of Right)"));
          currentSideIsLeft = true;
          currentMotorID = 0;
          currentTaskIsWithdraw = false;
          failLap(false);  // treat as right fail
        }
      }
      break;
      
    case PENDING_REWARD_LEFT:
      // committed to Left side - waiting for nose-poke and response
      if (m3Running) {
        //reward is currently being dispensed on left side - pause until done
        break;
      }
      if (CenterEdge) {
        // animal left back to center before nose-poke (skipped the task)
        Serial.println(F("Skipped task (did not enter nosepoke)"));
        failLap(true);  // left side trial failed due to skip
      } 
      else if (LnoseEdge) {
        // nose-poke detected on left - deliver odor and determine task type
        // randomly select one of the two odor pumps on left (M1 or M2)
        // (enforcing no >3 consecutive uses of the same motor)
        uint8_t taskTypeChoice;
        if (lastLeftTaskType != 0 && leftRepeatCount >= 3) {
          // if one task type was used 3 times in a row on left side, force switch to the other type
          taskTypeChoice = (lastLeftTaskType == 1 ? 2 : 1);
        } else {
          taskTypeChoice = random(1, 3);  // 1 or 2 (1 = hold task, 2 = withdraw task)
        }
        // update consecutive-use tracking for left side
        if (taskTypeChoice == lastLeftTaskType) {
          leftRepeatCount++;
        } else {
          lastLeftTaskType = taskTypeChoice;
          leftRepeatCount = 1;
        }
        // activate the selected odor pump on left
        if (taskTypeChoice == 1) {  // hold task odor
          leftScentMotor1->setSpeed(150);
          leftScentMotor1->run(FORWARD);
          m1Running = true;
          m1StartTime = millis();
          Serial.println(F("Odor dispensed: motor 1 (Hold Task)"));
          currentTaskIsWithdraw = false;   // hold task selected
          currentMotorID = 1;
        } else {  // withdraw task odor
          leftScentMotor2->setSpeed(150);
          leftScentMotor2->run(FORWARD);
          m2Running = true;
          m2StartTime = millis();
          Serial.println(F("Odor dispensed: motor 2 (Withdraw Task)"));
          currentTaskIsWithdraw = true;    // withdraw task selected
          currentMotorID = 2;
        }
        // evaluate behavior based on task type
        if (currentTaskIsWithdraw) {
          doWithdrawTask(true);
        } else {
          doHoldTask(true);
        }
      }
      else if (millis() - stateStart > DECISION_TIMEOUT) {
        // no nose-poke in time window after committing - timeout failure
        Serial.println(F("Timeout waiting for nose-poke on left side"));
        failLap(true);
      }
      break;
      
    case PENDING_REWARD_RIGHT:
      // committed to right side - waiting for nose-poke and response
      if (m6Running) {
        //reward currently being dispensed on right side - wait until done
        break;
      }
      if (CenterEdge) {
        // animal left back to center before nose-poking
        Serial.println(F("Skipped task (did not enter nosepoke)"));
        failLap(false);  // right side trial failed due to skip
      } 
      else if (RnoseEdge) {
        // nose-poke detected on right side - deliver odor and determine task type
        uint8_t taskTypeChoice;
        if (lastRightTaskType != 0 && rightRepeatCount >= 3) {
          // if one task type was used 3 times in a row on right side, force switch to the other type
          taskTypeChoice = (lastRightTaskType == 1 ? 2 : 1);
        } else {
          taskTypeChoice = random(1, 3);  // 1 or 2 (1 = hold task , 2 = withdraw task)
        }
        if (taskTypeChoice == lastRightTaskType) {
          rightRepeatCount++;
        } else {
          lastRightTaskType = taskTypeChoice;
          rightRepeatCount = 1;
        }
        if (taskTypeChoice == 1) {  // hold task odor
          rightScentMotor1->setSpeed(150);
          rightScentMotor1->run(FORWARD);
          m4Running = true;
          m4StartTime = millis();
          Serial.println(F("Odor dispensed: motor 4 (Hold Task)"));
          currentTaskIsWithdraw = false;
          currentMotorID = 4;
        } else {  // withdraw task odor
          rightScentMotor2->setSpeed(150);
          rightScentMotor2->run(FORWARD);
          m5Running = true;
          m5StartTime = millis();
          Serial.println(F("Odor dispensed: motor 5 (Withdraw Task)"));
          currentTaskIsWithdraw = true;
          currentMotorID = 5;
        }
        if (currentTaskIsWithdraw) {
          doWithdrawTask(false);
        } else {
          doHoldTask(false);
        }
      }
      else if (millis() - stateStart > DECISION_TIMEOUT) {
        Serial.println(F("Timeout waiting for nose-poke on right side"));
        failLap(false);
      }
      break;
      
    case LEFT_SUCCESS:
    case RIGHT_SUCCESS:
      // trial success - wait for the animal to return to center to start next lap
      if (CenterEdge) {
        startNextLapPass();
      }
      break;
      
    case LEFT_FAILED:
    case RIGHT_FAILED:
      // trial failed -  wait for the animal to return to center to reset
      if (CenterEdge) {
        startNextLapFail();
      }
      break;
  }
}

// readSensors(): initiate and read capacitive sensor measurements asynchronously to update touch status
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
        uint16_t data[2];
        if (sensor.readMeasurement(0, data) == 0) {
          float capVal = ((int16_t)data[0]) / 5242.88;
          leftTouched = (capVal > THRESH_LEFT);
        }
      }
      sensor.triggerSingleMeasurement(1, FDC1004_100HZ);
      fStamp = millis();
      fStage = 2;
      break;
    case 2: // read right result (after >=16 ms)
      if (millis() - fStamp < FDC_MS) return;
      {
        uint16_t data[2];
        if (sensor.readMeasurement(1, data) == 0) {
          float capVal = ((int16_t)data[0]) / 5242.88;
          rightTouched = (capVal > THRESH_RIGHT);
        }
      }
      fStage = 0;
      break;
  }
}

// doHoldTask(): handle behavior for a HOLD task (must maintain touch for requiredHoldTime to succeed)
void doHoldTask(bool isLeft) {
  // Determine which piezo buzzer to use for this side
  uint8_t piezoPin = isLeft ? PIEZO_LEFT_PIN : PIEZO_RIGHT_PIN;
  Serial.print(F("Nose-poke: "));
  Serial.println(isLeft ? F("Left (Hold task)") : F("Right (Hold task)"));
  Serial.print(F("Waiting for hold ≥ "));
  Serial.print(requiredHoldTime);
  Serial.println(F(" ms"));
  
  unsigned long touchStart = 0;
  bool touching = false;
  unsigned long waitStart = millis();
  const unsigned long TOTAL_TIMEOUT = 15000;  // overall timeout to complete hold
  
  // loop until the animal maintains touch for requiredHoldTime or a failure occurs
  while (true) {
    // immediate capacitive reading for touch status
    uint16_t raw[2];
    bool gotReading = (sensor.measureChannel(isLeft ? SENSOR_LEFT_CH : SENSOR_RIGHT_CH, CAPDAC, raw) == 0);
    bool contact = false;
    if (gotReading) {
      float cap = ((int16_t)raw[0]) / 5242.88;
      contact = (cap > (isLeft ? THRESH_LEFT : THRESH_RIGHT));
    }
    // check for contact start or continuity
    if (!touching) {
      if (contact) {
        touching = true;
        touchStart = millis();
        Serial.println(F("Contact detected: start timing hold"));
      }
    } else {
      if (!contact) {
        Serial.println(F("Contact lost before required hold time"));
        touching = false;
      } else if (millis() - touchStart >= requiredHoldTime) {
        // required hold duration achieved
        Serial.println(F("Hold completed: SUCCESS"));
        tone(piezoPin, 4000, rewardBeepDuration);  // reward cue tone
        // schedule reward pump activation after tone
        motorScheduled = true;
        motorScheduledLeft = isLeft;
        motorScheduledTime = millis();
        successCt++;
        // log trial outcome - success
        Serial.print(isLeft ? F("Left") : F("Right"));
        Serial.print(F(" - M"));
        Serial.print(currentMotorID);
        Serial.print(F(" - Hold - Success"));
        Serial.println();
        // set state to success (will wait for center beam to be broken)
        state = isLeft ? LEFT_SUCCESS : RIGHT_SUCCESS;
        return;
      }
    }
    // check for timeout
    if (millis() - waitStart > TOTAL_TIMEOUT) {
      Serial.println(F("Hold task failed: timed out waiting for sufficient contact"));
      failLap(isLeft);
      return;
    }
    // small delay - allow motor updates (odor pump stop)
    delay(5);
    updateMotors();
  }
}

// doWithdrawTask(): handle behavior for a withdraw task (must withdraw quickly, then hold for reward)
void doWithdrawTask(bool isLeft) {
  uint8_t piezoPin = isLeft ? PIEZO_LEFT_PIN : PIEZO_RIGHT_PIN;
  Serial.print(F("Nose-poke: "));
  Serial.println(isLeft ? F("Left (Withdraw task)") : F("Right (Withdraw task)"));
  Serial.println(F("Begin withdraw phase: must exit nosepoke within 1s and avoid touch for 900ms"));
  
  unsigned long withdrawStart = millis();
  bool exitedNose = false;
  bool earlyTouch = false;
  
  // phase 1: withdrawal check (wait window)
  while (millis() - withdrawStart < WITHDRAW_TIMEOUT) {
    // check capacitive contact during withdrawal phase
    uint16_t raw[2];
    bool gotReading = (sensor.measureChannel(isLeft ? SENSOR_LEFT_CH : SENSOR_RIGHT_CH, CAPDAC, raw) == 0);
    bool contact = false;
    if (gotReading) {
      float cap = ((int16_t)raw[0]) / 5242.88;
      contact = (cap > (isLeft ? THRESH_LEFT : THRESH_RIGHT));
    }
    // if the animal touches the gavage too soon (within EXCLUSION_TIME), mark failure
    if (contact && (millis() - withdrawStart < EXCLUSION_TIME)) {
      earlyTouch = true;
      Serial.println(F("Failed: Touched gavage too early during withdraw"));
      break;
    }
    // check if nose-poke beam is cleared (animal withdrew nose)
    if (!BEAM_BROKEN(isLeft ? LEFT_NOSE_PIN : RIGHT_NOSE_PIN)) {
      // nose beam is unbroken => nose has been removed from the nose-poke
      exitedNose = true;
      // do not break here; continue loop to enforce full exclusion window
    }
    updateMotors();  // update motor states (allows odor pump to stop after 1s)
    delay(1);
  }
  // evaluate withdrawal outcome
  if (!exitedNose) {
    // Nose was not removed within time window - failure
    Serial.println(F("Failed: Did not withdraw nose within time window"));
    failLap(isLeft);
    return;
  }
  if (earlyTouch) {
    // the animal touched the gavage too early during withdrawal -> failure
    failLap(isLeft);
    return;
  }
  // if here, withdrawal was successful
  Serial.println(F("Withdrawal success!"));

  // play a success tone to indicate correct withdraw behavior
  //tone(piezoPin, 4000, 1000); 
  
  // phase 2: hold for reward after successful withdrawal
  Serial.print(F("Now must hold ≥ "));
  Serial.print(requiredHoldTime);
  Serial.println(F(" ms for reward"));
  
  unsigned long touchStart           = 0;
  unsigned long holdPhaseStart      = millis();
  const unsigned long TOTAL_TIMEOUT = 15000;
  bool touching                     = false;

  
  // wait for the animal to make and hold contact for requiredHoldTime (as in hold task)
  while (true) {
    uint16_t raw[2];
    bool gotReading = (sensor.measureChannel(isLeft ? SENSOR_LEFT_CH : SENSOR_RIGHT_CH, CAPDAC, raw) == 0);
    bool contact = false;
    if (gotReading) {
      float cap = ((int16_t)raw[0]) / 5242.88;
      contact = (cap > (isLeft ? THRESH_LEFT : THRESH_RIGHT));
    }
    if (!touching) {
      if (contact) {
        touching   = true;
        touchStart = millis();
        Serial.println(F("Contact detected: start timing hold (post-withdraw)"));
      }
    } else {
      if (!contact) {
        Serial.println(F("Contact lost during hold phase"));
        touching = false;
      } else if (millis() - touchStart >= requiredHoldTime) {
        // Hold requirement met after withdrawal
        Serial.println(F("Hold completed after withdraw: SUCCESS"));
        tone(piezoPin, 4000, rewardBeepDuration);
        motorScheduled     = true;
        motorScheduledLeft = isLeft;
        motorScheduledTime = millis();
        successCt++;
        Serial.print(isLeft ? F("Left") : F("Right"));
        Serial.print(F(" - M"));
        Serial.print(currentMotorID);
        Serial.print(F(" - Withdraw - Success"));
        Serial.println();
        state = isLeft ? LEFT_SUCCESS : RIGHT_SUCCESS;
        return;
      }
    }
    if (millis() - holdPhaseStart > TOTAL_TIMEOUT) {
      Serial.println(F("Withdraw task failed: timed out waiting for hold after withdraw"));
      failLap(isLeft);
      return;
    }
    delay(5);
    updateMotors();
  }
}

// updateMotors(): Non-blocking motor management for reward and odor pumps (start/stop motors at the right times)
void updateMotors() {
  unsigned long currentTime = millis();
  // check if a reward pump should be started after the reward tone delay
  if (motorScheduled && (currentTime - motorScheduledTime >= motorDelayAfterBeep)) {
    Adafruit_DCMotor *pump = motorScheduledLeft ? leftRewardMotor : rightRewardMotor;
    pump->setSpeed(150);
    pump->run(FORWARD);
    if (motorScheduledLeft) {
      m3Running = true;
      m3StartTime = currentTime;
    } else {
      m6Running = true;
      m6StartTime = currentTime;
    }
    motorScheduled = false;
    // (reward pump will be stopped after motorRunDuration via checks below)
  }
  // stop odor pumps after scentRunDuration (automatically stop after ~1s)
  if (m1Running && (currentTime - m1StartTime >= scentRunDuration)) {
    leftScentMotor1->run(RELEASE);
    m1Running = false;
  }
  if (m2Running && (currentTime - m2StartTime >= scentRunDuration)) {
    leftScentMotor2->run(RELEASE);
    m2Running = false;
  }
  if (m4Running && (currentTime - m4StartTime >= scentRunDuration)) {
    rightScentMotor1->run(RELEASE);
    m4Running = false;
  }
  if (m5Running && (currentTime - m5StartTime >= scentRunDuration)) {
    rightScentMotor2->run(RELEASE);
    m5Running = false;
  }
  // stop reward pumps after motorRunDuration (stop dispensing reward)
  if (m3Running && (currentTime - m3StartTime >= motorRunDuration)) {
    leftRewardMotor->run(RELEASE);
    m3Running = false;
    // Serial.println(F("M3 stopped"));
  }
  if (m6Running && (currentTime - m6StartTime >= motorRunDuration)) {
    rightRewardMotor->run(RELEASE);
    m6Running = false;
    // Serial.println(F("M6 stopped"));
  }
}

// failLap(): handle a trial failure, log it, and transition to a failure state
void failLap(bool wasLeft) {
  failCt++;
  // log trial outcome (Failure). If no odor motor was triggered, log "N/A" for motor and task.
  Serial.print(currentSideIsLeft ? F("Left") : F("Right"));
  Serial.print(F(" - "));
  if (currentMotorID != 0) {
    Serial.print(F("M"));
    Serial.print(currentMotorID);
    Serial.print(currentTaskIsWithdraw ? F(" - Withdraw - Failure") : F(" - Hold - Failure"));
  } else {
    Serial.print(F("N/A - N/A - Failure"));
  }
  Serial.println();
  // set state to appropriate FAILED state
  state = wasLeft ? LEFT_FAILED : RIGHT_FAILED;
  // reset commit sensor edge flags (prevents carrying over a broken beam state to next trial)
  prevLcommit = prevRcommit = false;
}

// startNextLapPass(): Prepare for the next lap after a successful trial
void startNextLapPass() {
  lapCount++;
  Serial.print(F("Lap "));
  Serial.print(lapCount);
  Serial.println(F(" - Ready"));
  // increase required hold time for next trials (up to max)
  if (requiredHoldTime < HOLD_MAX) {
    requiredHoldTime += HOLD_INC;
    if (requiredHoldTime > HOLD_MAX) requiredHoldTime = HOLD_MAX;
  }
  // enforce alternation: set which side is required next (from this trial's nextSideAllowed)
  requireSide = nextSideAllowed;
  state = WAIT_COMMIT;
}

// startNextLapFail(): Prepare for the next lap after a failed trial
void startNextLapFail() {
  lapCount++;
  Serial.print(F("Lap "));
  Serial.print(lapCount);
  Serial.println(F(" - Ready"));
  // (requiredHoldTime remains unchanged on failure)
  requireSide = nextSideAllowed;
  state = WAIT_COMMIT;
}
