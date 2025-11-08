#define TEAM_NUMBER 14

#ifndef TEAM_NUMBER
#error "Define your team number with `#define TEAM_NUMBER 12345` at the top of the file."
#elif TEAM_NUMBER < 1 || 40 < TEAM_NUMBER
#error "Team number must be within 1 and 40"
#endif

#include <math.h>   // for fabs & lroundf

// ========================= Drive tuning =========================
const float DEADBAND   = 0.05f; // ignore small stick noise
const bool  SQUARED    = true;  // square inputs for finer control near center
const bool  INVERT_LEFT  = true;
const bool  INVERT_RIGHT = false;

// ========================= Arm / servo calibration =========================
static const int SHOULDER_SERVO_ID = 1; // RR_setServo1 -> shoulder
static const int ELBOW_SERVO_ID    = 2; // RR_setServo2 -> elbow
static const int BTN_SERVO_ID      = 3; // RR_setServo3 -> button servo (unique channel)

// Offsets and reversals for shoulder/elbow (button servo uses raw 0/180)
float SHOULDER_OFFSET_DEG = 0.0f;
bool  SHOULDER_REVERSED   = false;

float ELBOW_OFFSET_DEG = 0.0f;
bool  ELBOW_REVERSED   = false;

const int SHOULDER_MIN_DEG = 0;
const int SHOULDER_MAX_DEG = 179;
const int ELBOW_MIN_DEG    = 0;
const int ELBOW_MAX_DEG    = 179;

// ===== Slew parameters (non-blocking for arm) =====
int curShoulderCmd = -1, curElbowCmd = -1;  // last written (0..180)
int tgtShoulderCmd = -1, tgtElbowCmd = -1;  // targets (0..180)
const int SERVO_STEP = 3;                   // deg per step
const int SERVO_STEP_DELAY_MS = 8;          // ms between steps
unsigned long lastServoStepMs = 0;

// Convert elbow internal angle (between links) -> elbow joint bend
static inline float elbowInternalToJointDeg(float internal_deg) {
  return 180.0f - internal_deg;
}

// ========================= Helpers =========================
static inline float applyDeadband(float v, float db) {
  if (fabs(v) < db) return 0.0f;
  return (v > 0 ? (v - db) : (v + db)) / (1.0f - db);
}

static inline float shape(float v) {
  if (!SQUARED) return v;
  return (v >= 0 ? 1 : -1) * (v * v);
}

static inline int clampi(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline int applyOffsetReverseClamp(float joint_deg, float offset_deg, bool reversed,
                                          int lo = 0, int hi = 180) {
  float d = joint_deg + offset_deg;
  if (reversed) d = 180.0f - d;
  return clampi((int)lroundf(d), lo, hi);
}

static inline int stepToward(int cur, int target, int step) {
  if (cur < 0) return target;                 // first write jumps to target
  if (cur < target) return (cur + step > target) ? target : (cur + step);
  if (cur > target) return (cur - step < target) ? target : (cur - step);
  return cur;
}

// Low-level writes respecting channel mapping
static inline void writeShoulder(int cmd) {
  if (SHOULDER_SERVO_ID == 1) RR_setServo1(cmd);
  else                        RR_setServo2(cmd);
}
static inline void writeElbow(int cmd) {
  if (ELBOW_SERVO_ID == 2)    RR_setServo2(cmd);
  else                        RR_setServo1(cmd);
}
static inline void writeButtonServo(int cmd) {
  // Route to the selected button-servo channel
  if (BTN_SERVO_ID == 1) RR_setServo1(cmd);
  else if (BTN_SERVO_ID == 2) RR_setServo2(cmd);
  else /* BTN_SERVO_ID == 3 */ RR_setServo3(cmd);
}

// Non-blocking arm servo slewing: call once per loop
void serviceServosSlew() {
  unsigned long now = millis();
  if (now - lastServoStepMs < (unsigned long)SERVO_STEP_DELAY_MS) return;
  lastServoStepMs = now;

  if (tgtShoulderCmd < 0 && tgtElbowCmd < 0) return;

  int newS = curShoulderCmd;
  int newE = curElbowCmd;

  if (tgtShoulderCmd >= 0) newS = stepToward(curShoulderCmd, tgtShoulderCmd, SERVO_STEP);
  if (tgtElbowCmd    >= 0) newE = stepToward(curElbowCmd,    tgtElbowCmd,    SERVO_STEP);

  if (newS != curShoulderCmd) { writeShoulder(newS); curShoulderCmd = newS; }
  if (newE != curElbowCmd)    { writeElbow(newE);    curElbowCmd    = newE; }
}

// ========================= ARM STATES (FSM) =========================
struct Pose {
  int shoulder_deg;        // absolute joint angle (deg)
  int elbow_internal_deg;  // internal angle between links (deg, 180 = straight)
};

enum ArmState { SCORE = 0, INTAKE = 1, BUTTON = 2, IDLE = 3, STATE_COUNT = 4 };
void setState(ArmState s);  // explicit prototype

// >>> EDIT THESE ANGLES TO THE FIELD POSITIONS <<<
Pose poses[STATE_COUNT] = {
  /* SCORE  (A) */ { 180, 120 },
  /* INTAKE (B) */ { 120,  90 },
  /* BUTTON (X) */ {  60, 150 },
  /* IDLE   (Y) */ {  90, 180 },
};

ArmState currentState     = IDLE;           // start idle
ArmState lastAppliedState = (ArmState)(-1); // force initial apply

// Compute new targets for the arm slewer
void applyCurrentPose() {
  const Pose &p = poses[(int)currentState];

  int shoulder_cmd = applyOffsetReverseClamp(
      clampi(p.shoulder_deg, SHOULDER_MIN_DEG, SHOULDER_MAX_DEG),
      SHOULDER_OFFSET_DEG, SHOULDER_REVERSED, 0, 180);

  float elbow_joint_deg = elbowInternalToJointDeg((float)p.elbow_internal_deg);
  int elbow_cmd = applyOffsetReverseClamp(
      clampi((int)lroundf(elbow_joint_deg), ELBOW_MIN_DEG, ELBOW_MAX_DEG),
      ELBOW_OFFSET_DEG, ELBOW_REVERSED, 0, 180);

  tgtShoulderCmd = shoulder_cmd;
  tgtElbowCmd    = elbow_cmd;

  // On first apply, jump to target immediately
  if (curShoulderCmd < 0) { curShoulderCmd = tgtShoulderCmd; writeShoulder(curShoulderCmd); }
  if (curElbowCmd    < 0) { curElbowCmd    = tgtElbowCmd;    writeElbow(curElbowCmd); }
}

void setState(ArmState s) {
  currentState = s;
  if (currentState != lastAppliedState) {
    applyCurrentPose();
    lastAppliedState = currentState;
  }
}

// ========================= Intake/Outtake motor (Motor3) =========================
const bool  AUX_REVERSED   = false; // flip if directions are backwards
const float AUX_BASE_SPEED = 1.0f;

static inline float auxIntakeSpeed()  { return AUX_REVERSED ? -AUX_BASE_SPEED :  AUX_BASE_SPEED; }
static inline float auxOuttakeSpeed() { return AUX_REVERSED ?  AUX_BASE_SPEED : -AUX_BASE_SPEED; }

// ========================= Button Servo Toggle (LB) =========================
// LB toggles the button servo 0° <-> 180° (with edge detection)
bool btnServoHigh = false;   // false = 0°, true = 180°
int  btnServoCmd  = 0;       // last commanded to the button servo

// ========================= Setup / Loop =========================
void setup() {
  Serial.begin(115200);
  applyCurrentPose();           // set initial arm targets & write once

  // Initialize button servo to 0°
  btnServoHigh = false;
  btnServoCmd  = 0;
  writeButtonServo(btnServoCmd);
}

void loop() {
  // -------- Tank drive --------
  float leftY  = RR_axisLY();
  float rightY = RR_axisRY();

  float leftCmd  = applyDeadband(leftY,  DEADBAND);
  float rightCmd = applyDeadband(rightY, DEADBAND);

  leftCmd  = shape(leftCmd);
  rightCmd = shape(rightCmd);

  if (INVERT_LEFT)  leftCmd  = -leftCmd;
  if (INVERT_RIGHT) rightCmd = -rightCmd;

  RR_setMotor1(leftCmd);   // left side
  RR_setMotor2(rightCmd);  // right side

  // -------- Buttons --------
  bool btnA  = RR_buttonA();   // SCORE
  bool btnB  = RR_buttonB();   // INTAKE
  bool btnX  = RR_buttonX();   // BUTTON
  bool btnY  = RR_buttonY();   // IDLE
  bool btnRB = RR_buttonRB();  // OUTTAKE
  bool btnLB = RR_buttonLB();  // TOGGLE button servo

  // Arm state selection (targets change; slewer handles motion)
  if (btnA) setState(SCORE);
  if (btnB) { setState(INTAKE); RR_setMotor3(auxIntakeSpeed()); } // intake while B held
  if (btnX) setState(BUTTON);
  if (btnY) setState(IDLE);

  // Outtake (Motor3) on RB. Stop when neither RB nor B is held.
  if (btnRB) {
    RR_setMotor3(auxOuttakeSpeed());
  } else if (!btnB) {
    RR_setMotor3(0.0f);
  }

  // ---- LB: toggle the button servo between 0 and 180 on each press ----
  static bool lbPrev = false;
  if (btnLB && !lbPrev) {
    btnServoHigh = !btnServoHigh;                 // flip state
    btnServoCmd  = btnServoHigh ? 180 : 0;        // target angle
    writeButtonServo(btnServoCmd);                // immediate write (no slew)
  }
  lbPrev = btnLB;

  // -------- Slew the arm servos smoothly (non-blocking) --------
  serviceServosSlew();

  // -------- Telemetry --------
  Serial.print("Ultrasonic=");
  Serial.print(RR_getUltrasonic());
  Serial.print(" ;; Line=");

  int sensors[6];
  RR_getLineSensors(sensors);
  for (int i = 0; i < 6; ++i) {
    Serial.print(sensors[i]);
    Serial.print(" ");
  }
  Serial.print(" | State=");
  Serial.print((int)currentState);
  Serial.print(" | CmdS/E=");
  Serial.print(curShoulderCmd);
  Serial.print("/");
  Serial.print(curElbowCmd);
  Serial.print(" | BtnServo=");
  Serial.println(btnServoCmd);

  delay(5);
}
