// Replace 12345 with the correct team number and then uncomment the line below.
#define TEAM_NUMBER 14

#ifndef TEAM_NUMBER
#error "Define your team number with `#define TEAM_NUMBER 12345` at the top of the file."
#elif TEAM_NUMBER < 1 || 40 < TEAM_NUMBER
#error "Team number must be within 1 and 40"
#endif

// --- Tuning options ---
const float DEADBAND = 0.05f;   // ignore small stick noise
const bool  SQUARED  = true;    // square inputs for finer control near center
const bool  INVERT_LEFT  = true; // flip if your left side runs backwards
const bool  INVERT_RIGHT = false; // flip if your right side runs backwards

static inline float applyDeadband(float v, float db) {
  if (fabs(v) < db) return 0.0f;
  // rescale so full throw still reaches 1.0 after deadband
  return (v > 0 ? (v - db) : (v + db)) / (1.0f - db);
}

static inline float shape(float v) {
  if (!SQUARED) return v;
  // keep sign, square magnitude (gentler near zero, full power at 1)
  return (v >= 0 ? 1 : -1) * (v * v);
}

void setup() {
  Serial.begin(115200);
}

int temp = 0;

void loop() {
  // Read the four joystick axes in [-1.0, 1.0]
  float rightX = RR_axisRX();
  float rightY = RR_axisRY();
  float leftX  = RR_axisLX();
  float leftY  = RR_axisLY();

  // ---- TANK DRIVE ----
  // Left stick Y drives left side; Right stick Y drives right side
  float leftCmd  = applyDeadband(leftY,  DEADBAND);
  float rightCmd = applyDeadband(rightY, DEADBAND);

  leftCmd  = shape(leftCmd);
  rightCmd = shape(rightCmd);

  if (INVERT_LEFT)  leftCmd  = -leftCmd;
  if (INVERT_RIGHT) rightCmd = -rightCmd;

  // Motor1 = left side, Motor2 = right side (same mapping you had before)
  RR_setMotor1(leftCmd);
  RR_setMotor2(rightCmd);

  // --- Buttons ---
  bool btnA  = RR_buttonA();
  bool btnB  = RR_buttonB();
  bool btnX  = RR_buttonX();
  bool btnY  = RR_buttonY();
  bool btnRB = RR_buttonRB();
  bool btnLB = RR_buttonLB();

  // Motor3 via A/B
  if (btnA) {
    RR_setMotor3(1.0);
  } else if (btnB) {
    RR_setMotor3(-1.0);
  } else {
    RR_setMotor3(0.0);
  }

  // Motor4 via X/Y
  if (btnX) {
    RR_setMotor4(1.0);
  } else if (btnY) {
    RR_setMotor4(-1.0);
  } else {
    RR_setMotor4(0.0);
  }

  // Servo 1 via dpad
  if (RR_dpad() == 6) { // left
    if (temp > 0) temp -= 10;
  } else if (RR_dpad() == 2) { // right
    if (temp < 180) temp += 10;
  }
  RR_setServo1(temp);

  // Servo 2 via shoulders
  if (btnRB) {
    RR_setServo2(180);
  } else if (btnLB) {
    RR_setServo2(0);
  }

  // --- Sensors / Telemetry ---
  Serial.print("Ultrasonic=");
  Serial.print(RR_getUltrasonic());
  Serial.print(" ;; ");

  int sensors[6];
  Serial.print("Line sensors=");
  RR_getLineSensors(sensors);
  for (int i = 0; i < 6; ++i) {
    Serial.print(sensors[i]);
    Serial.print(" ");
  }
  Serial.print(btnA ? 1 : 0);
  Serial.print(btnB ? 1 : 0);
  Serial.print(btnX ? 1 : 0);
  Serial.print(btnY ? 1 : 0);
  Serial.println();

 
  delay(1);
}

// vim: tabstop=2 shiftwidth=2 expandtab
