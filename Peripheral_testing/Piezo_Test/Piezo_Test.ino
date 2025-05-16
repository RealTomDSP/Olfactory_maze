/* Tom Stowell
   Entity: Buzzer-toggle demo
   Connect piezo buzzer (or an active buzzer module)
   – Signal  : D9   but change to whatever is available
   – GND     : GND
*/

const uint8_t BUZZ_PIN = 9;        // digital I/O pin for the buzzer
const uint16_t TONE_HZ = 4000;     // tone frequency if you use tone()
bool buzzerState = false;          // remembers current state

void setup() {
  pinMode(BUZZ_PIN, OUTPUT);
  digitalWrite(BUZZ_PIN, LOW);
  Serial.begin(9600);
  // Give the PC a moment to open the port:
  while (!Serial) { /* Leonardo/MKR ZERO need this to enumerate */ }
  Serial.println(F("Ready for commands: '1'=on  '0'=off"));
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == '1') {                 // turn ON
      buzzerState = true;
#ifdef __AVR__
      // AVR boards can use tone()
      tone(BUZZ_PIN, TONE_HZ);        // continuous tone
#else
      digitalWrite(BUZZ_PIN, HIGH);   // active buzzer module
#endif
      Serial.println(F("ON"));
    } else if (cmd == '0') {          // turn OFF
      buzzerState = false;
#ifdef __AVR__
      noTone(BUZZ_PIN);
#else
      digitalWrite(BUZZ_PIN, LOW);
#endif
      Serial.println(F("OFF"));
    }
  }
}
