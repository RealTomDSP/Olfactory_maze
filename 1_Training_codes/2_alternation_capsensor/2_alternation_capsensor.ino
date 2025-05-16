/* Tom Stowell 

Entity: Alternation Maze with Capacitive Sensor & Hold Task

Description: Builds on the simple alternation maze: 
- animal is free to choose either side for first lap
- animal is rewarded for successful alternation and sustained contact with gavage
- Duration for contact starts at 100ms - increases by 12ms per successful trial 
- Maximum threshold of 700ms
Notes: 
- reward motor dispenses approximately one drop of reward per 100ms
- FDC1004 chips requires FDC1004 library (zip file to integrate with arduino)
   slight edits to the library have been made to mute some print statements 
*/ 

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "FDC1004.h" 

//ir beam declaration (active low)
#define BEAM_BROKEN(pin) (!digitalRead(pin))   

//motor setup
Adafruit_MotorShield motorShieldLeft  = Adafruit_MotorShield(0x60);
Adafruit_MotorShield motorShieldRight = Adafruit_MotorShield(0x61);
Adafruit_DCMotor *leftRewardMotor     = motorShieldLeft.getMotor(3);  
Adafruit_DCMotor *rightRewardMotor    = motorShieldRight.getMotor(3); 

const unsigned long motorRunDuration = 300; // run motor for 300 ms 
unsigned long m3StartTime = 0;
unsigned long m6StartTime = 0;
const int maxRetries      = 3;
int m3Retries             = 0;
int m6Retries             = 0;
bool m3Running = false;
bool m6Running = false;

// non-blocking motor activation parameters
const unsigned long motorDelayAfterBeep = 50; // 50ms as functional 'debounce' before motor activation
const unsigned int  rewardBeepDuration  = 500; // 50 ms piezo beep duration for reward
unsigned long motorScheduledTime        = 0;
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


// cap sensor declarations 
FDC1004 sensor;
const uint8_t SENSOR_LEFT_CH  = 0; // cin1 (channel 0 on FDC)
const uint8_t SENSOR_RIGHT_CH = 3; // cin2 (channel 3 on FDC)
const uint8_t CAPDAC          = 0;
const float THRESH_LEFT       = 5.4;
const float THRESH_RIGHT      = 5.4; //tune these for sensitivity (higher threshold is less sensitive)
bool leftTouched              = false;
bool rightTouched             = false;

// asynchronous FDC1004 sequencing
static uint8_t  fStage        = 0;  // 0 trigger L, 1 read L & trigger R, 2 read R
static unsigned long fStamp   = 0;
const uint16_t FDC_MS         = 16;  // ≥16 ms between trigger & read (play with this threshold if issues with contact occur)

// state machine declarations
enum State { 
  WAIT_COMMIT,
  PENDING_REWARD_LEFT, PENDING_REWARD_RIGHT,
  LEFT_SUCCESS, RIGHT_SUCCESS, LEFT_FAILED, RIGHT_FAILED 
};
//State state = WAIT_COMMIT_ANY;
State state         = WAIT_COMMIT;   
uint8_t requireSide = 0;       // allows either side success for inital lap 


// reward timing declarations 
unsigned long requiredHoldTime       = 100;// initial hold time 100ms 
const unsigned long HOLD_INC         = 12; //hold time scaling factor (12ms)
const unsigned long HOLD_MAX         = 700; // max hold threshold (met around 58 laps)
const unsigned long DECISION_TIMEOUT = 30000; // timeout 

//counters for lap count, successful trial, failed trial 
int lapCount        = 0;
int successCt       = 0;
int failCt          = 0; 
int nextSideAllowed = 0;   // 0=expects either, 1= expects left, 2=expects right

// bools for prev lap direction 
static bool prevLcommit  = false;
static bool  prevRcommit = false;
static bool  prevCenter  = false;
static bool prevLnose    = false;
static bool prevRnose    = false;

// function declarations
void failLap(bool wasLeft);
void updateMotors();

//main
void setup() {
  Serial.begin(9600);
  if (!motorShieldLeft.begin() || !motorShieldRight.begin()) {
    Serial.println(F("Motor-shield init FAIL – check I2C addresses!"));
    while(true); // kill if motor shields not found
  }
  // pin declarations 
  pinMode(LEFT_COMMIT_PIN, INPUT_PULLUP);
  pinMode(RIGHT_COMMIT_PIN, INPUT_PULLUP);
  pinMode(LEFT_NOSE_PIN,   INPUT_PULLUP);
  pinMode(RIGHT_NOSE_PIN,  INPUT_PULLUP);
  pinMode(CENTER_PIN,      INPUT_PULLUP);
  pinMode(PIEZO_LEFT_PIN,  OUTPUT);
  pinMode(PIEZO_RIGHT_PIN, OUTPUT);

  Wire.begin();
// initialize cap sensor measurement variables
  sensor.configureMeasurementSingle(0, SENSOR_LEFT_CH,  CAPDAC);
  sensor.configureMeasurementSingle(1, SENSOR_RIGHT_CH, CAPDAC);

  Serial.println(F("System Initialized - Let's Go!"));
}

void loop() {

  readSensors(); //read ir beam and cap sensors asynchronously
  updateMotors();  // update motor states (start/stop) without blocking

  //bool for break beam in main loop
  bool Lcommit = BEAM_BROKEN(LEFT_COMMIT_PIN);
  bool Rcommit = BEAM_BROKEN(RIGHT_COMMIT_PIN);
  bool Lnose   = BEAM_BROKEN(LEFT_NOSE_PIN);
  bool Rnose   = BEAM_BROKEN(RIGHT_NOSE_PIN);
  bool Center  = BEAM_BROKEN(CENTER_PIN);

  // declarations for ensuring alternation (record either pass / fail and next state) 
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

  static unsigned long stateStart = 0;  // declration for time stamp upon state update

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
          nextSideAllowed = 2; // next lap must go right 
          requireSide     = 2;
          stateStart = millis();
          state = PENDING_REWARD_LEFT;
      }
      // if commited right
      else if (RcommitEdge) {
          lapCount = 1;
          //chose right
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
          failLap(false);  // failed to commit right
      }
  }
  break;

    case PENDING_REWARD_LEFT:
      // non blocking logic when reward motor triggered
      if (m3Running) break;
      if (CenterEdge) { 
        Serial.println(F("Skipped task")); 
        failLap(true);     // failed by leaving before nose-poke
      } 
      else if (LnoseEdge) {
        rewardRoutine(true);// process left nose-poke for reward
      } 
      else if (millis() - stateStart > DECISION_TIMEOUT) {
        Serial.println(F("Timeout"));
        failLap(true);     // no nose-poke in time
      }
      break;

    case PENDING_REWARD_RIGHT:
      if (m6Running) break;
      if (CenterEdge) { 
        Serial.println(F("Skipped task")); 
        failLap(false); 
      } 
      else if (RnoseEdge) {
        rewardRoutine(false);
      } 
      else if (millis() - stateStart > DECISION_TIMEOUT) {
        Serial.println(F("Timeout"));
        failLap(false);
      }
      break;

    case LEFT_SUCCESS:
    case RIGHT_SUCCESS:
      if (CenterEdge) {
        //lapCount++;
        //state = WAIT_COMMIT;
        //Serial.print(F("Lap "));
        //Serial.print(lapCount);  // advance to next lap after outcome logged
        startNextLapPass();
      }
      break;

    case LEFT_FAILED:
    case RIGHT_FAILED:
      if (CenterEdge){
        startNextLapFail();
      }
      break;
  }
}

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

// rewardroutine
// waits for cap sensor to be held for required duration
// note: uses blocking reads of FDC1004 for timing during hold detection
void rewardRoutine(bool isLeft) {
  const uint8_t piezo     = isLeft ? PIEZO_LEFT_PIN : PIEZO_RIGHT_PIN;
  Adafruit_DCMotor *pump  = isLeft ? leftRewardMotor : rightRewardMotor;
  const uint8_t motorID   = isLeft ? 3 : 6;
  const uint8_t ch        = isLeft ? SENSOR_LEFT_CH : SENSOR_RIGHT_CH;
  const float thr         = isLeft ? THRESH_LEFT : THRESH_RIGHT;

  unsigned long touchStart          = 0;
  unsigned long waitStart           = millis();
  const unsigned long TOTAL_TIMEOUT = 15000;
  bool touching                     = false;

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
        //Serial.println(motorID);

        // reward sequence
        // play tone and schedule motor activation
        tone(piezo, 4000, rewardBeepDuration); //4000 hz - change this for higher / lower frequency tone
        motorScheduled = true;
        motorScheduledLeft = isLeft;
        motorScheduledTime = millis();
        //motor waits for motordelayafterbeep function in updatemotors function 

        // update state and exit
        successCt++;
        state = isLeft ? LEFT_SUCCESS : RIGHT_SUCCESS;

        //Serial.print(F("Lap "));
        //Serial.print(lapCount);
        //Serial.println(F(": Pass"));
        //Serial.println(F("Reward delivered: Pass"));

        return;
      }
    }

    // handle timeout for nose-poke hold
    if (millis() - waitStart > TOTAL_TIMEOUT) {
      Serial.println(F("Reward timeout"));
      failLap(isLeft);
      return;
    }
    delay(5);
  }
}

//startnextlappass function
void startNextLapPass() {

  lapCount++;
  Serial.print(F("Lap "));              Serial.println(lapCount);

  // increase requires contact time with gavage
  if (requiredHoldTime < HOLD_MAX) {
      requiredHoldTime += HOLD_INC;
      if (requiredHoldTime > HOLD_MAX) requiredHoldTime = HOLD_MAX;
  }
  //Serial.print(F("New hold‑time = "));
  //Serial.print(requiredHoldTime);
  //Serial.println(F(" ms"));

//check commit side
  requireSide = nextSideAllowed;
  state       = WAIT_COMMIT;// new lap goes into wait_commit state 
  //stateStart  = millis();         
}

//startnextlapfail function
void startNextLapFail() {

  lapCount++;
  Serial.print(F("Lap "));              Serial.println(lapCount);

//check which commit point is expected
  requireSide = nextSideAllowed;
  state       = WAIT_COMMIT;           
  //stateStart  = millis();              
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

  // left reward motor - stop after motor run duration
  if (m3Running && (currentTime - m3StartTime >= motorRunDuration)) {
    leftRewardMotor->run(RELEASE);
    m3Running = false;
    //Serial.println(F("M3 stopped"));
  }

  // right reward motor - stop after motor run duration elapsed
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
