/*  Tom Stowell  

Entity: Simple Alternation Maze with Reward
Description: This code uses and arduino with IR beam peripherals
to enforce the following:
- mandatory alternation
- lap counting / print on center beam
- lockout on incorrect commit
- commit beams locked after reward or error until center beam
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>

//motor setup 
Adafruit_MotorShield motorShieldLeft  = Adafruit_MotorShield(0x60);
Adafruit_MotorShield motorShieldRight = Adafruit_MotorShield(0x61);
Adafruit_DCMotor *leftRewardMotor     = motorShieldLeft .getMotor(3);
Adafruit_DCMotor *rightRewardMotor    = motorShieldRight.getMotor(3);

//pin declarations
const int leftCommitPin  = 2;
const int leftNosePin    = 3;
const int centerBeamPin  = 4;
const int rightCommitPin = 5;
const int rightNosePin   = 7;

//state machine declarations
enum State { IDLE, LEFT_ACTIVE, RIGHT_ACTIVE, LOCKOUT };
State currentState = IDLE;

//global vars
int  nextSideAllowed               = 0;  // 0 = first lap (either side OK)
const unsigned long rewardDuration = 200;  // ms
const unsigned long debounceDelay  = 50;    // ms

unsigned long lastCommitDebounceTime = 0;   // left/right commits
unsigned long lastCenterTriggerTime  = 0;   // centre beam debounce
bool          lastCenterBeam         = false;

//lap handler: 0 = none, 1 = correct, 2 = incorrect
uint8_t pendingType    = 0;
bool    centerRequired = false;   // true → must cross centre before next commit

int lapCount       = 0;   // total laps
int incorrectCount = 0;   // errors only

//setup
void setup() {
  Serial.begin(9600);
  motorShieldLeft.begin();
  motorShieldRight.begin();

  pinMode(leftCommitPin , INPUT_PULLUP);
  pinMode(leftNosePin   , INPUT_PULLUP);
  pinMode(centerBeamPin , INPUT_PULLUP);
  pinMode(rightCommitPin, INPUT_PULLUP);
  pinMode(rightNosePin  , INPUT_PULLUP);

  Serial.println("System Initialized - Lets Go!");
}

//main
void loop() {
  unsigned long now = millis();

  //ir beams (active low)
  bool leftCommit  = !digitalRead(leftCommitPin);
  bool rightCommit = !digitalRead(rightCommitPin);
  bool leftNose    = !digitalRead(leftNosePin);
  bool rightNose   = !digitalRead(rightNosePin);
  bool centerBeam  = !digitalRead(centerBeamPin);

  //state machine 
  switch (currentState) {
    case IDLE:
      handleCommits(leftCommit, rightCommit, now);
      break;

    case LEFT_ACTIVE:
      if (leftNose)  deliverReward(leftRewardMotor);
      break;

    case RIGHT_ACTIVE:
      if (rightNose) deliverReward(rightRewardMotor);
      break;

    case LOCKOUT:
      // ignore everything until center beam is triggered 
      break;
  }

  //center ir beam handler
  bool risingEdge = centerBeam && !lastCenterBeam;
  if (risingEdge && (now - lastCenterTriggerTime > debounceDelay)) {
    lastCenterTriggerTime = now;

    if (centerRequired) {    // only count when a lap is pending
      ++lapCount;

      if (pendingType == 1) {     // correct
        Serial.print("Lap ");
        Serial.print(lapCount);
        Serial.println(" complete – reset to IDLE");
      } else if (pendingType == 2) { // incorrect
        ++incorrectCount;
        Serial.print("Lap ");
        Serial.print(lapCount);
        Serial.print(" marked INCORRECT – total errors: ");
        Serial.println(incorrectCount);
      }

      //clears lap flags and unlock commits
      pendingType    = 0;
      centerRequired = false;
      currentState   = IDLE;            // also exits lockout state
    }
  }
  lastCenterBeam = centerBeam;
}

//functions
//handlecommits handles the commit point logic
void handleCommits(bool left, bool right, unsigned long t) {

  // ignore commits if centeer beam hasn’t been triggered after reward/error
  if (centerRequired) return;

  if (!left && !right) return;               // wait
  if (t - lastCommitDebounceTime < debounceDelay) return;
  lastCommitDebounceTime = t;

  //left commit beam
  if (left) {
    if (nextSideAllowed == 0 || nextSideAllowed == 1) {    // correct
      currentState    = LEFT_ACTIVE;
      nextSideAllowed = 2;                               // enfore alternation 
      Serial.println("Left commit accepted");
    } else {                                              // incorrect
      registerIncorrect("left");
    }
  }

  //right commit beam 
  else if (right) {
    if (nextSideAllowed == 0 || nextSideAllowed == 2) {     // correct
      currentState    = RIGHT_ACTIVE;
      nextSideAllowed = 1;                              // enforce alt
      Serial.println("Right commit accepted");
    } else {                                            // incorrect
      registerIncorrect("right");
    }
  }
}

void registerIncorrect(const char *side) {
  pendingType    = 2;            // lap will be counted by center beam trigger
  centerRequired = true;         // lock commits
  currentState   = LOCKOUT;      // ignore everything else

  Serial.print("Incorrect commit (");
  Serial.print(side);
  Serial.println(") – waiting for centre beam to reset");
}

void deliverReward(Adafruit_DCMotor* motor) {
  motor->setSpeed(255);
  motor->run(FORWARD);
  delay(rewardDuration);
  motor->run(RELEASE);
  Serial.println("Reward delivered: PASS");

  currentState   = IDLE;         // back to idle state after  reward
  pendingType    = 1;            // mark lap as correct
  centerRequired = true;         // lock commits until center beam triggered
}
