/*
    FINALIZED Robotic Arm Controller for Arduino Mega
    =================================================

    Hardware (your finalized setup):
    - 3 × BTS7960 H-bridge modules → 3 linear actuators (DC motors)
    - 1 × DM860H stepper driver     → 1 × NEMA 23 stepper (high torque joint, e.g. base/shoulder)
    - 2 × TB6600 stepper drivers    → 2 × NEMA 17 steppers (smaller joints)
    - 1 × 35kg metal gear servo     → gripper

    Pin mapping:
    BTS1: RPWM=5,  LPWM=6,  REN=22, LEN=23   → Linear actuator 1
    BTS2: RPWM=2,  LPWM=3,  REN=24, LEN=25   → Linear actuator 2
    BTS3: RPWM=7,  LPWM=8,  REN=9,  LEN=10   → Linear actuator 3
    Stepper1 (DM860H NEMA 23): PUL=12, DIR=13, ENA=11
    Stepper2 (TB6600 NEMA 17): PUL=47, DIR=48, ENA=46
    Stepper3 (TB6600 NEMA 17): PUL=49, DIR=50, ENA=51   ← New assignment; change if needed
    Gripper servo: pin 9 (PWM capable on Mega)

    SUPPORTED SERIAL COMMANDS:
    ----------------------------------------------------------------
    DC1 <speed>     → Linear actuator 1     (-255 reverse ... 0 stop ... +255 forward)
    DC2 <speed>     → Linear actuator 2
    DC3 <speed>     → Linear actuator 3

    S1 <steps>      → NEMA 23 stepper (DM860H) relative steps
    S2 <steps>      → NEMA 17 stepper 2 (TB6600) relative steps
    S3 <steps>      → NEMA 17 stepper 3 (TB6600) relative steps

    POS1 <position> → NEMA 23 stepper to absolute position (software tracked)

    GRIP <angle>    → Gripper servo (0–180°)

    STOP or ESTOP   → Immediately stop ALL motors

    Note: Send commands with newline (\n). Use small values first!
*/

#include <AccelStepper.h>
#include <Servo.h>

// ────────────────────────────────────────────────
// BTS7960 arrays: {RPWM, LPWM, REN, LEN}
const int BTS1[] = {5, 6, 22, 23};   // Linear actuator 1
const int BTS2[] = {2, 3, 24, 25};   // Linear actuator 2
const int BTS3[] = {7, 8,  9, 10};   // Linear actuator 3

// ────────────────────────────────────────────────
// Steppers (DRIVER mode = STEP + DIR signals)
AccelStepper stepper1(AccelStepper::DRIVER, 12, 13);   // NEMA 23 via DM860H (big joint)
AccelStepper stepper2(AccelStepper::DRIVER, 47, 48);   // NEMA 17 via TB6600 #1
AccelStepper stepper3(AccelStepper::DRIVER, 49, 50);   // NEMA 17 via TB6600 #2  ← adjust pins if conflict

// Gripper servo
Servo gripper;
const int gripperPin = 9;

// Enable polarity (common for both DM860H & TB6600 clones: LOW = enabled)
#define ENA_ACTIVE LOW              // Change to HIGH if motors don't energize/hold

// ────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("Robotic Arm FINAL Controller Started");
  Serial.println("Hardware: 3*DC linear | 1*NEMA23 DM860H | 2*NEMA17 TB6600 | Servo gripper");
  Serial.println("Commands: DC1/2/3 <spd>, S1/2/3 <steps>, POS1 <pos>, GRIP <ang>, STOP");

  // Prepare all BTS7960 pins → start safe (all LOW)
  int allBtsPins[] = {2,3,5,6,7,8,9,10,22,23,24,25};
  for (int p : allBtsPins) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }

  // Enable BTS7960 bridges (both EN HIGH = active)
  digitalWrite(BTS1[2], HIGH); digitalWrite(BTS1[3], HIGH);
  digitalWrite(BTS2[2], HIGH); digitalWrite(BTS2[3], HIGH);
  digitalWrite(BTS3[2], HIGH); digitalWrite(BTS3[3], HIGH);

  // Stepper enable pins – test polarity!
  pinMode(11, OUTPUT);  digitalWrite(11, ENA_ACTIVE);   // DM860H NEMA 23
  pinMode(46, OUTPUT);  digitalWrite(46, ENA_ACTIVE);   // TB6600 #1 NEMA 17
  pinMode(51, OUTPUT);  digitalWrite(51, ENA_ACTIVE);   // TB6600 #2 NEMA 17

  // Stepper tuning – start conservative (adjust later based on load)
  stepper1.setMaxSpeed(800);    stepper1.setAcceleration(400);   // NEMA 23 – slower/heavier
  stepper2.setMaxSpeed(1500);   stepper2.setAcceleration(800);   // NEMA 17 – faster/lighter
  stepper3.setMaxSpeed(1500);   stepper3.setAcceleration(800);

  // Gripper safe start
  gripper.attach(gripperPin);
  gripper.write(90);            // Neutral/middle position
}

// ────────────────────────────────────────────────
void loop() {
  // Run non-blocking stepper movements
  stepper1.run();
  stepper2.run();
  stepper3.run();

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 0) return;

    // ── Linear actuators (DC via BTS7960) ────────
    if (cmd.startsWith("DC")) {
      int joint = cmd.substring(2,3).toInt();
      int speed = constrain(cmd.substring(4).toInt(), -255, 255);

      if      (joint == 1) setBTS(BTS1, speed);
      else if (joint == 2) setBTS(BTS2, speed);
      else if (joint == 3) setBTS(BTS3, speed);
      else { Serial.println("Invalid DC joint (1-3)"); return; }

      Serial.print("DC"); Serial.print(joint); Serial.print(" speed "); Serial.println(speed);
    }

    // ── Relative moves for steppers ──────────────
    else if (cmd.startsWith("S1 ")) {
      long steps = cmd.substring(3).toInt();
      stepper1.move(steps);
      Serial.print("NEMA23 (S1) moving "); Serial.println(steps);
    }
    else if (cmd.startsWith("S2 ")) {
      long steps = cmd.substring(3).toInt();
      stepper2.move(steps);
      Serial.print("NEMA17-1 (S2) moving "); Serial.println(steps);
    }
    else if (cmd.startsWith("S3 ")) {
      long steps = cmd.substring(3).toInt();
      stepper3.move(steps);
      Serial.print("NEMA17-2 (S3) moving "); Serial.println(steps);
    }

    // ── Absolute position (only for main NEMA 23 for now) ──
    else if (cmd.startsWith("POS1 ")) {
      long pos = cmd.substring(5).toInt();
      stepper1.moveTo(pos);
      Serial.print("NEMA23 to absolute pos "); Serial.println(pos);
    }

    // ── Gripper ──────────────────────────────────
    else if (cmd.startsWith("GRIP ")) {
      int angle = constrain(cmd.substring(5).toInt(), 0, 180);
      gripper.write(angle);
      Serial.print("Gripper → "); Serial.print(angle); Serial.println("°");
    }

    // ── Emergency stop ───────────────────────────
    else if (cmd == "STOP" || cmd == "ESTOP") {
      setBTS(BTS1, 0); setBTS(BTS2, 0); setBTS(BTS3, 0);
      stepper1.stop(); stepper2.stop(); stepper3.stop();
      Serial.println("ALL MOTORS STOPPED");
    }

    else {
      Serial.println("Unknown cmd. Use: DC1-3, S1-3, POS1, GRIP, STOP");
    }
  }
}

// ────────────────────────────────────────────────
// Set speed on BTS7960 (positive = forward, negative = reverse, 0 = coast)
void setBTS(const int bts[], int speed) {
  int r_pwm = bts[0];
  int l_pwm = bts[1];

  if (speed > 0) {
    analogWrite(r_pwm, speed);
    analogWrite(l_pwm, 0);
  } else if (speed < 0) {
    analogWrite(r_pwm, 0);
    analogWrite(l_pwm, -speed);
  } else {
    analogWrite(r_pwm, 0);
    analogWrite(l_pwm, 0);          // Coast stop
    // For brake mode: analogWrite(r_pwm, 255); analogWrite(l_pwm, 255);
  }
}