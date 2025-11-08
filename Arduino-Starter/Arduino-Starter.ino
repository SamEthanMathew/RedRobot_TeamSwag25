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
// RR_setServo1 -> shoulder
// RR_setServo2 -> elbow
// RR_setServo3 -> button servo


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


static inline void writeShoulder(int cmd) { RR_setServo1(cmd); }
static inline void writeElbow(int cmd)    { RR_setServo2(cmd); }
static inline void writeButtonServo(int cmd) { RR_setServo3(cmd); }


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


enum ArmState { SCORE = 0, INTAKE = 1, STATE_COUNT = 2 };
void setState(ArmState s);  // explicit prototype


// >>> EDIT THESE ANGLES TO THE FIELD POSITIONS <<<
Pose poses[STATE_COUNT] = {
  /* SCORE  (A) */ { 180, 120 },
  /* INTAKE (B) */ { 120,  90 },
};


ArmState currentState     = INTAKE;           // start in intake state
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
bool btnServoHigh = false;   // false = 0°, true = 180°
int  btnServoCmd  = 0;
bool autoButtonPressed = false;  // track if button was pressed in auto mode


// ========================= Mode handling =========================
enum RunMode { MODE_STATIONARY, MODE_AUTO, MODE_TELEOP };
RunMode mode = MODE_STATIONARY;
unsigned long autoStartMs = 0;
int autoStep = 0;


// --- autonomous step timing (edit to taste) ---
const unsigned long T_INTAKE_MS   = 1500;  // intake window
const unsigned long T_DRIVE_MS    = 2000;  // drive forward
const unsigned long T_SCORE_MS    = 1000;  // settle in score pose


// Start teleop when RT is pressed
// Automatically uses RR_buttonRT() from Library.ino
static inline bool teleopStartRequested() {
  return RR_buttonRT();
}


// Stop drivetrain + intake safely
static inline void stopAllMotion() {
  RR_setMotor1(0.0f);
  RR_setMotor2(0.0f);
  RR_setMotor3(0.0f);
}


// ---------- AUTO line-follow params ----------
const float  AUTO_BASE_SPEED   = 0.50f;  // forward speed (0..1)
const float  AUTO_TURN_KP      = 0.35f;  // steering gain (tweak on carpet)
const int    STOP_DIST_CM      = 20;     // stop when obstacle closer than this
const int    LINE_N            = 6;      // RR_getLineSensors() returns 6 values in this project

// Line sensor ranges: white = 200-600, black = 4000-7000
const int    LINE_SENSOR_WHITE_MIN = 200;   // minimum white value
const int    LINE_SENSOR_WHITE_MAX = 600;   // maximum white value
const int    LINE_SENSOR_BLACK_MIN = 4000;  // minimum black value
const int    LINE_SENSOR_BLACK_MAX = 7000;  // maximum black value
// Readings between 600-4000 are ambiguous and ignored

// Map sensor indices to positions (left -> negative, right -> positive).
// Spread wider for a stronger steering signal.
const int8_t LINE_POS[LINE_N] = { -5, -3, -1, 1, 3, 5 };


// Compute line error in ~[-1, +1]. Positive = line is to the RIGHT.
float computeLineError(bool &haveLine) {
  int sensors[LINE_N];
  RR_getLineSensors(sensors);

  // Process sensors: white range (200-600) = no line, black range (4000-7000) = on line
  // Ignore ambiguous readings between 600-4000
  long wsum = 0;       // weighted sum (position * normalized value)
  float totalWeight = 0.0f;  // sum of normalized values for averaging
  
  for (int i = 0; i < LINE_N; ++i) {
    int sensorVal = sensors[i];
    float weight = 0.0f;
    
    // Check if sensor is in white range (200-600)
    if (sensorVal >= LINE_SENSOR_WHITE_MIN && sensorVal <= LINE_SENSOR_WHITE_MAX) {
      // White = no line, weight = 0 (don't contribute to line position)
      weight = 0.0f;
    }
    // Check if sensor is in black range (4000-7000)
    else if (sensorVal >= LINE_SENSOR_BLACK_MIN && sensorVal <= LINE_SENSOR_BLACK_MAX) {
      // Black = on line, normalize within black range [4000, 7000] -> [0.0, 1.0]
      // Closer to 7000 = stronger black detection
      float normalized = (float)(sensorVal - LINE_SENSOR_BLACK_MIN) / 
                          (float)(LINE_SENSOR_BLACK_MAX - LINE_SENSOR_BLACK_MIN);
      weight = normalized;  // use normalized value as weight
      wsum += (long)(LINE_POS[i] * weight * 1000.0f);  // scale for precision
      totalWeight += weight;
    }
    // Readings between 600-4000 are ambiguous/transitional - ignore them
    // (weight remains 0.0f)
  }

  haveLine = (totalWeight > 0.1f);  // have line if total weight is significant

  static float lastErr = 0.0f;
  if (!haveLine) {
    // If we lost the line, keep steering the last known way a bit.
    return lastErr;
  }

  // Calculate weighted average position
  float avgPos = (float)wsum / (totalWeight * 1000.0f);  // roughly in [-5..+5]
  float err = avgPos / 5.0f;                // normalize to about [-1..+1]
  lastErr = err;
  return err;
}


// One step of autonomous line following. Returns true when we decided to stop.
bool autoLineFollowStep() {
  // 1) Stop if obstacle is close
  int dist = RR_getUltrasonic();     // assumed centimeters
  if (dist > 0 && dist <= STOP_DIST_CM) {
    RR_setMotor1(0.0f);
    RR_setMotor2(0.0f);
    RR_setMotor3(0.0f);
    return true;
  }


  // 2) Compute line error and steer
  bool haveLine = false;
  float err = computeLineError(haveLine);


  // Base forward speed; steer by differential
  float left  = AUTO_BASE_SPEED + (AUTO_TURN_KP * err);  // err>0 (right) -> steer right (left faster)
  float right = AUTO_BASE_SPEED - (AUTO_TURN_KP * err);


  // Safety clamp
  if (left  >  1.0f) left  =  1.0f;
  if (right >  1.0f) right =  1.0f;
  if (left  < -1.0f) left  = -1.0f;
  if (right < -1.0f) right = -1.0f;


  // Apply motor inversion settings (same as teleop)
  if (INVERT_LEFT)  left  = -left;
  if (INVERT_RIGHT) right = -right;


  RR_setMotor1(left);
  RR_setMotor2(right);


  // Keep the arm in intake pose during auto
  setState(INTAKE);


  return false; // keep going
}


// ========================= Setup / Loop =========================
void setup() {
  Serial.begin(115200);
 
  // Initialize mode to STATIONARY - robot waits for LT or RT
  mode = MODE_STATIONARY;
 
  // Initialize arm pose (intake)
  setState(INTAKE);
  applyCurrentPose(); // set initial targets & write once


  // Initialize button servo to 0°
  btnServoHigh = false;
  btnServoCmd  = 0;
  writeButtonServo(btnServoCmd);
  autoButtonPressed = false;  // Reset button press flag


  // Stop all motion - ensure robot is stationary
  stopAllMotion();


  autoStartMs = millis();
  autoStep = 0;
}


void loop() {
  // --------- GLOBAL: smooth the arm servos ---------
  serviceServosSlew();


  // --------- Check for mode switching ---------
  if (mode == MODE_STATIONARY) {
    // Robot is stationary - wait for LT or RT to start a mode
    stopAllMotion();  // Ensure all motion is stopped
   
    if (RR_buttonLT()) {
      // LT pressed: start autonomous mode
      mode = MODE_AUTO;
      stopAllMotion();
      autoButtonPressed = false;  // Reset button press flag
      Serial.println("LT pressed - Starting AUTONOMOUS mode");
    } else if (teleopStartRequested()) {
      // RT pressed: start teleop mode
      mode = MODE_TELEOP;
      stopAllMotion();
      autoButtonPressed = false;  // Reset button press flag
      Serial.println("RT pressed - Starting TELEOP mode");
    }
  } else if (mode == MODE_AUTO && teleopStartRequested()) {
    // RT pressed during autonomous: switch to teleop
    mode = MODE_TELEOP;
    stopAllMotion();
    autoButtonPressed = false;  // Reset button press flag
    Serial.println("RT pressed - Switching from AUTO to TELEOP");
  }


  // --------- MODE LOGIC ---------
  if (mode == MODE_STATIONARY) {
    // Stationary mode: stop all motion, wait for LT or RT
    stopAllMotion();
  } else if (mode == MODE_AUTO) {
    // Follow the tape until ultrasonic says we're close enough, then stop & press button.
    bool shouldStop = autoLineFollowStep();
    if (shouldStop) {
      // Press the button when we arrive (only once)
      if (!autoButtonPressed) {
        // Just press the button servo, arm stays in current pose
        btnServoHigh = true;        // Press button (180°)
        btnServoCmd  = 180;
        writeButtonServo(btnServoCmd);
        autoButtonPressed = true;   // Mark as pressed
      }
      RR_setMotor3(0.0f);        // stop intake/outtake
      // stay in AUTO; driver can press RT to enter teleop at any time
    }
  } else {
    // =========== TELEOP ===========
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
    bool btnRB = RR_buttonRB();  // OUTTAKE
    bool btnLB = RR_buttonLB();  // TOGGLE button servo


    // Arm state selection (targets change; slewer handles motion)
    if (btnA) setState(SCORE);
    if (btnB) { setState(INTAKE); RR_setMotor3(auxIntakeSpeed()); } // intake while B held


    // Outtake (Motor3) on RB. Stop when neither RB nor B is held.
    if (btnRB) {
      RR_setMotor3(auxOuttakeSpeed());
    } else if (!btnB) {
      RR_setMotor3(0.0f);
    }


    // LB: toggle the button servo 0 <-> 180 on each press
    static bool lbPrev = false;
    if (btnLB && !lbPrev) {
      btnServoHigh = !btnServoHigh;
      btnServoCmd  = btnServoHigh ? 180 : 0;
      writeButtonServo(btnServoCmd);
    }
    lbPrev = btnLB;
  }


  // -------- Telemetry --------
  Serial.print("Mode=");
  if (mode == MODE_STATIONARY) {
    Serial.print("STATIONARY");
  } else if (mode == MODE_AUTO) {
    Serial.print("AUTO");
  } else {
    Serial.print("TELEOP");
  }
  Serial.print(" | State=");
  Serial.print((int)currentState);
  Serial.print(" | CmdS/E=");
  Serial.print(curShoulderCmd);
  Serial.print("/");
  Serial.print(curElbowCmd);
  Serial.print(" | BtnServo=");
  Serial.println(btnServoCmd);


  delay(5);  // keep loop snappy
}
