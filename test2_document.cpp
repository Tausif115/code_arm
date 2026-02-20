/*
    Robotic Arm Controller for Arduino Mega
    =======================================
    
    Hardware:
    - 3 × BTS7960 H-bridge modules → control 3 linear actuators (DC motors)
    - 2 × DM860H stepper drivers → control 2 NEMA 23 stepper motors
    - 1 × 35kg metal gear servo → gripper
    
    Pin mapping (based on your notes):
    BTS1: RPWM=5, LPWM=6, REN=22, LEN=23
    BTS2: RPWM=2, LPWM=3, REN=24, LEN=25
    BTS3: RPWM=7, LPWM=8, REN=9,  LEN=10
    Stepper1 (DM860H): PUL=12, DIR=13, ENA=11
    Stepper2 (DM860H): PUL=47, DIR=48, ENA=46
    Gripper servo: pin 9 (PWM capable)
    
    SUPPORTED SERIAL COMMANDS (send via Serial Monitor or Python):
    ----------------------------------------------------------------
    DC1 <speed>     → Move linear actuator 1     (speed: -255 to +255)
    DC2 <speed>     → Move linear actuator 2
    DC3 <speed>     → Move linear actuator 3
                    Examples:  DC1 120    DC2 -80    DC1 0 (stop)
    
    S1 <steps>      → Move stepper 1 relative steps (positive/negative)
    S2 <steps>      → Move stepper 2 relative steps
                    Examples:  S1 400    S2 -1200
    
    POS1 <position> → Move stepper 1 to absolute position (software tracked)
                    Example:   POS1 0    POS1 5000
    
    GRIP <angle>    → Set gripper servo angle (0–180°)
                    Examples:  GRIP 0    GRIP 90    GRIP 180
    
    STOP            → Immediately stop all motors (DC + steppers)
    ESTOP           → Same as STOP (emergency stop)
    
    Note: Commands must end with newline (\n)
*/

#include <AccelStepper.h>
#include <Servo.h>

// ────────────────────────────────────────────────
// BTS7960 H-bridge arrays: {RPWM, LPWM, REN, LEN}
const int BTS1[] = {5, 6, 22, 23};   // Joint 1 linear actuator
const int BTS2[] = {2, 3, 24, 25};   // Joint 2 linear actuator
const int BTS3[] = {7, 8, 9, 10};    // Joint 3 linear actuator

// ────────────────────────────────────────────────
// Stepper motor drivers (DM860H style - STEP/PULSE + DIR)
AccelStepper stepper1(AccelStepper::DRIVER, 12, 13);   // Stepper 1: base/shoulder?
AccelStepper stepper2(AccelStepper::DRIVER, 47, 48);   // Stepper 2: elbow/another joint?

// Gripper servo (35kg metal gear)
Servo gripper;
const int gripperPin = 9;           // PWM-capable pin on Mega

// Enable polarity for DM860H (most common is LOW = enabled)
#define ENA_ACTIVE LOW              // ← Change to HIGH if your driver requires it

// ────────────────────────────────────────────────
void setup() {
  // Start serial communication – faster rate = more responsive
  Serial.begin(115200);
  Serial.println("Robotic Arm Controller Started");
  Serial.println("Supported commands: DC1/DC2/DC3 <speed>, S1/S2 <steps>, POS1 <pos>, GRIP <angle>, STOP");

  // ── Prepare all BTS7960 control pins ───────────────
  int allBtsPins[] = {2,3,5,6,7,8,9,10,22,23,24,25};
  for (int p : allBtsPins) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);           // Start in safe state (motors off)
  }

  // Enable all three BTS7960 bridges (both enable pins HIGH = active)
  digitalWrite(BTS1[2], HIGH); digitalWrite(BTS1[3], HIGH);
  digitalWrite(BTS2[2], HIGH); digitalWrite(BTS2[3], HIGH);
  digitalWrite(BTS3[2], HIGH); digitalWrite(BTS3[3], HIGH);

  // ── Stepper driver enable pins ─────────────────────
  pinMode(11, OUTPUT); digitalWrite(11, ENA_ACTIVE);   // Stepper 1 enable
  pinMode(46, OUTPUT); digitalWrite(46, ENA_ACTIVE);   // Stepper 2 enable

  // Configure stepper motion parameters (tune these later!)
  stepper1.setMaxSpeed(1200);       // max steps/second
  stepper1.setAcceleration(600);    // steps/second²
  stepper2.setMaxSpeed(1200);
  stepper2.setAcceleration(600);

  // Initialize gripper in safe middle position
  gripper.attach(gripperPin);
  gripper.write(90);                // 90° = usually open/neutral
}

// ────────────────────────────────────────────────
void loop() {
  // Non-blocking movement for both steppers
  stepper1.run();
  stepper2.run();

  // Check if any command arrived over USB serial
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();                       // Remove whitespace

    if (cmd.length() == 0) return;    // Ignore empty lines

    // ── DC motor / linear actuator control ──────────
    if (cmd.startsWith("DC")) {
      int joint = cmd.substring(2,3).toInt();           // get joint number 1,2,3
      int speed = constrain(cmd.substring(4).toInt(), -255, 255);  // limit -255..255

      if (joint == 1)      setBTS(BTS1, speed);
      else if (joint == 2) setBTS(BTS2, speed);
      else if (joint == 3) setBTS(BTS3, speed);
      else {
        Serial.println("Invalid DC joint (use 1,2,3)");
        return;
      }

      Serial.print("DC"); Serial.print(joint);
      Serial.print(" set to speed "); Serial.println(speed);
    }

    // ── Relative movement for stepper 1 ─────────────
    else if (cmd.startsWith("S1 ")) {
      long steps = cmd.substring(3).toInt();
      stepper1.move(steps);               // positive = one direction, negative = opposite
      Serial.print("Stepper 1 moving "); Serial.println(steps);
    }

    // ── Relative movement for stepper 2 ─────────────
    else if (cmd.startsWith("S2 ")) {
      long steps = cmd.substring(3).toInt();
      stepper2.move(steps);
      Serial.print("Stepper 2 moving "); Serial.println(steps);
    }

    // ── Absolute position for stepper 1 (software) ──
    else if (cmd.startsWith("POS1 ")) {
      long pos = cmd.substring(5).toInt();
      stepper1.moveTo(pos);
      Serial.print("Stepper 1 moving to absolute position "); Serial.println(pos);
    }

    // ── Gripper servo control ───────────────────────
    else if (cmd.startsWith("GRIP ")) {
      int angle = constrain(cmd.substring(5).toInt(), 0, 180);
      gripper.write(angle);
      Serial.print("Gripper set to "); Serial.print(angle); Serial.println(" degrees");
    }

    // ── Emergency / normal stop ─────────────────────
    else if (cmd == "STOP" || cmd == "ESTOP") {
      setBTS(BTS1, 0);
      setBTS(BTS2, 0);
      setBTS(BTS3, 0);
      stepper1.stop();
      stepper2.stop();
      Serial.println("All motors stopped (ESTOP)");
    }

    // Unknown command
    else {
      Serial.println("Unknown command. Supported: DC1/2/3, S1, S2, POS1, GRIP, STOP");
    }
  }
}

// ────────────────────────────────────────────────
// Helper function: Set speed on one BTS7960 module
// speed:  positive → forward
//         negative → reverse
//         0        → coast stop (free wheel)
void setBTS(const int bts[], int speed) {
  int r_pwm = bts[0];   // Right/Forward PWM
  int l_pwm = bts[1];   // Left/Reverse PWM

  if (speed > 0) {
    analogWrite(r_pwm, speed);
    analogWrite(l_pwm, 0);
  }
  else if (speed < 0) {
    analogWrite(r_pwm, 0);
    analogWrite(l_pwm, -speed);     // make positive value
  }
  else {
    analogWrite(r_pwm, 0);
    analogWrite(l_pwm, 0);          // Coast (most natural stop)
    // If you want strong brake instead, uncomment next line:
    // analogWrite(r_pwm, 255); analogWrite(l_pwm, 255);
  }
}