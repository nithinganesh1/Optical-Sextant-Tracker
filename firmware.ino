// CNC Shield V3 dual-stepper firmware for Sun Tracker  (v2)
//
//   X axis = M1 (pins 2,5,9)  -> BASE / mirror, homed on limit switch, driven by Camera 1
//   Y axis = M2 (pins 3,6)    -> second stepper, no limit switch, driven by Camera 2
//
// Both axes are TARGET based and fully independent:
//   - a new target can be given while an axis is moving (no lost / cancelled steps)
//   - each axis accelerates / decelerates (faster cruise speed without stalling)
//   - X homing does not block Y
//
// Commands (one per line, 115200 baud):
//   HOME | HOME X     -> home X on the limit switch (position becomes 0)
//   STOP              -> stop both axes immediately
//   STATUS 1|0        -> status stream on/off (every 100 ms + immediately when a move ends)
//   G <abs>           -> X absolute target (steps)
//   GY <abs>          -> Y absolute target (steps)
//   MOVE X <delta>    -> X relative to current target
//   MOVE Y <delta>    -> Y relative to current target
//   SPD <xUs> <yUs>   -> cruise step period in microseconds (smaller = faster, 300..2000)
//   "<s1> <s2>"       -> legacy: relative X,Y move
//
// Status line:  S <xPos> <yPos> <limit> <state> <homed> <xBusy> <yBusy>
//   state: 2 = homing, 1 = moving, 0 = idle

#define X_STEP_PIN    2
#define X_DIR_PIN     5
#define X_LIMIT_PIN   9
#define Y_STEP_PIN    3
#define Y_DIR_PIN     6
#define ENABLE_PIN    8

const unsigned int START_US   = 2000;   // first step period of every move (slow start)
const unsigned int ACCEL_US   = 25;     // period shrinks by this much per step while accelerating
const unsigned int HOME_US    = 1000;   // homing step period
const long MAX_X_POS          = 3200;   // keep in sync with the Python app
const long Y_LIMIT            = 8000;   // |Y| safety limit (steps from power-on position)
const long HOME_MAX_STEPS     = 4000;

struct Axis {
  long pos;
  long target;
  int dir;
  unsigned long lastUs;
  unsigned int curUs;
  unsigned int cruiseUs;
  uint8_t stepPin;
  uint8_t dirPin;
};

Axis xA = {0, 0, -1, 0, START_US, 800,  X_STEP_PIN, X_DIR_PIN};
Axis yA = {0, 0,  1, 0, START_US, 1000, Y_STEP_PIN, Y_DIR_PIN};

bool xHoming = false;
bool xHomed = false;
bool statusOn = false;
bool statusDue = false;
long homeSteps = 0;
unsigned long lastHomeUs = 0;
unsigned long lastStatusMs = 0;

char lineBuf[64];
uint8_t lineLen = 0;

bool xLimitPressed() { return digitalRead(X_LIMIT_PIN) == LOW; }

void pulse(uint8_t pin) {
  digitalWrite(pin, HIGH);
  delayMicroseconds(4);
  digitalWrite(pin, LOW);
}

void stopAll() {
  xA.target = xA.pos;
  yA.target = yA.pos;
  xA.curUs = START_US;
  yA.curUs = START_US;
  xHoming = false;
  statusDue = true;
}

void startXHoming() {
  xA.target = xA.pos;
  digitalWrite(X_DIR_PIN, LOW);      // toward the home switch
  xA.dir = -1;
  xA.curUs = START_US;
  homeSteps = 0;
  xHomed = false;
  xHoming = true;
  statusDue = true;
}

void setTargetX(long t) {
  if (!xHomed) return;               // ignore until homed
  xA.target = constrain(t, 0L, MAX_X_POS);
}

void setTargetY(long t) {
  yA.target = constrain(t, -Y_LIMIT, Y_LIMIT);
}

void serviceAxis(Axis &a, bool isX) {
  if (a.pos == a.target) { a.curUs = START_US; return; }

  int want = (a.target > a.pos) ? 1 : -1;
  unsigned long now = micros();

  if (want != a.dir) {               // reversal: set direction, restart slowly
    a.dir = want;
    digitalWrite(a.dirPin, want > 0 ? HIGH : LOW);
    a.curUs = START_US;
    a.lastUs = now;
    return;
  }

  if (isX && want < 0 && xLimitPressed()) {   // hit the switch going down
    a.pos = 0;
    a.target = 0;
    a.curUs = START_US;
    statusDue = true;
    return;
  }

  if (now - a.lastUs < a.curUs) return;
  a.lastUs = now;
  pulse(a.stepPin);
  a.pos += a.dir;

  long remaining = labs(a.target - a.pos);
  if (remaining == 0) { a.curUs = START_US; statusDue = true; return; }

  unsigned int stopSteps = (START_US - a.curUs) / ACCEL_US;   // steps needed to slow to START_US
  if ((unsigned long)remaining <= stopSteps) {
    a.curUs = min((unsigned int)START_US, (unsigned int)(a.curUs + ACCEL_US));
  } else if (a.curUs > a.cruiseUs) {
    a.curUs = max(a.cruiseUs, (unsigned int)(a.curUs - ACCEL_US));
  } else if (a.curUs < a.cruiseUs) {
    a.curUs = min(a.cruiseUs, (unsigned int)(a.curUs + ACCEL_US));
  }
}

void serviceHoming() {
  if (xLimitPressed()) {
    xHoming = false;
    xHomed = true;
    xA.pos = 0;
    xA.target = 0;
    xA.curUs = START_US;
    Serial.println("READY");
    statusDue = true;
    return;
  }
  if (homeSteps >= HOME_MAX_STEPS) {
    xHoming = false;
    xA.target = xA.pos;
    Serial.println("HOME_FAIL");
    statusDue = true;
    return;
  }
  unsigned long now = micros();
  if (now - lastHomeUs < HOME_US) return;
  lastHomeUs = now;
  pulse(X_STEP_PIN);
  homeSteps++;
}

void sendStatus() {
  if (!statusOn) return;
  unsigned long ms = millis();
  if (!statusDue && ms - lastStatusMs < 100) return;
  statusDue = false;
  lastStatusMs = ms;

  bool xBusy = (xA.pos != xA.target);
  bool yBusy = (yA.pos != yA.target);
  Serial.print("S ");  Serial.print(xA.pos);
  Serial.print(' ');    Serial.print(yA.pos);
  Serial.print(' ');    Serial.print(xLimitPressed() ? 1 : 0);
  Serial.print(' ');    Serial.print(xHoming ? 2 : ((xBusy || yBusy) ? 1 : 0));
  Serial.print(' ');    Serial.print(xHomed ? 1 : 0);
  Serial.print(' ');    Serial.print(xBusy ? 1 : 0);
  Serial.print(' ');    Serial.println(yBusy ? 1 : 0);
}

void handleLine(char *s) {
  if (strncmp(s, "STATUS", 6) == 0) { statusOn = (atol(s + 6) != 0); statusDue = true; return; }

  if (strncmp(s, "HOME", 4) == 0) {
    char *axis = s + 4;
    while (*axis == ' ' || *axis == '\t') axis++;
    if (*axis == 'X' || *axis == '\0') startXHoming();
    return;                          // HOME Y ignored (no switch)
  }

  if (strncmp(s, "STOP", 4) == 0) { stopAll(); return; }

  if (strncmp(s, "SPD", 3) == 0) {
    char *p = s + 3;
    long xs = strtol(p, &p, 10);
    long ys = strtol(p, &p, 10);
    if (xs > 0) xA.cruiseUs = constrain(xs, 300L, (long)START_US);
    if (ys > 0) yA.cruiseUs = constrain(ys, 300L, (long)START_US);
    return;
  }

  if (strncmp(s, "GY", 2) == 0) { setTargetY(atol(s + 2)); return; }
  if (s[0] == 'G')              { setTargetX(atol(s + 1));  return; }

  if (strncmp(s, "MOVE", 4) == 0) {
    char *rest = s + 4;
    while (*rest == ' ' || *rest == '\t') rest++;
    char which = *rest;
    if (which == 'X' || which == 'Y') {
      rest++;
      long d = atol(rest);
      if (which == 'X') setTargetX(xA.target + d); else setTargetY(yA.target + d);
    }
    return;
  }

  // legacy "<s1> <s2>" relative pair
  if ((s[0] >= '0' && s[0] <= '9') || s[0] == '-' || s[0] == '+') {
    long s1 = 0, s2 = 0;
    char *token = strtok(s, " ,\t");
    if (token != NULL) {
      s1 = atol(token);
      token = strtok(NULL, " ,\t");
      if (token != NULL) s2 = atol(token);
    }
    if (s1 != 0) setTargetX(xA.target + s1);
    if (s2 != 0) setTargetY(yA.target + s2);
  }
}

void readSerialNonBlocking() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      if (lineLen > 0) handleLine(lineBuf);
      lineLen = 0;
      continue;
    }
    if (c == '\r') continue;
    if (lineLen < sizeof(lineBuf) - 1) lineBuf[lineLen++] = c;
    else lineLen = 0;
  }
}

void setup() {
  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(X_LIMIT_PIN, INPUT_PULLUP);
  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);
  digitalWrite(X_STEP_PIN, LOW);
  digitalWrite(Y_STEP_PIN, LOW);
  digitalWrite(Y_DIR_PIN, HIGH);

  Serial.begin(115200);
  Serial.setTimeout(0);

  startXHoming();                    // home X automatically at boot
}

void loop() {
  readSerialNonBlocking();
  if (xHoming) serviceHoming();
  else         serviceAxis(xA, true);
  serviceAxis(yA, false);            // Y is independent, also runs while X homes
  sendStatus();
}
