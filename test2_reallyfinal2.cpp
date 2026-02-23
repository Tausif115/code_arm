#include <AccelStepper.h>
#include <Servo.h>

// --- ROVER DRIVE PINS (BTS7960) ---
const int Motor_1_Left[]  = {10, 11, 12, 13}; //
const int Motor_2_Right[] = {3, 2, 50, 51};   //
const int Motor_3_Left[]  = {6, 7, 8, 9};     //
const int Motor_4_Right[] = {4, 5, 34, 35};   //

// --- ARM STEPPER PINS ---
AccelStepper stepperBase(AccelStepper::DRIVER, 12, 13); // NEMA 23
AccelStepper stepperJoint2(AccelStepper::DRIVER, 47, 48); // NEMA 17
AccelStepper stepperJoint3(AccelStepper::DRIVER, 43, 44); // NEMA 17

// --- GRIPPER SERVO ---
Servo gripper;
const int gripperPin = 45; //

void setup() {
  Serial.begin(115200);
  
  // Initialize Rover Drive Motors
  initBTS(Motor_1_Left); initBTS(Motor_2_Right);
  initBTS(Motor_3_Left); initBTS(Motor_4_Right);

  // Initialize Arm Steppers
  stepperBase.setMaxSpeed(1000); stepperBase.setAcceleration(500);
  stepperJoint2.setMaxSpeed(800); stepperJoint2.setAcceleration(400);
  stepperJoint3.setMaxSpeed(800); stepperJoint3.setAcceleration(400);

  gripper.attach(gripperPin);
}

void loop() {
  // Non-blocking stepper updates
  stepperBase.run();
  stepperJoint2.run();
  stepperJoint3.run();

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    // --- ROVER MOVEMENT ---
    if (cmd.startsWith("DRIVE")) { // Format: DRIVE <leftSpeed> <rightSpeed>
      int spaceIdx = cmd.indexOf(' ', 6);
      int L = cmd.substring(6, spaceIdx).toInt();
      int R = cmd.substring(spaceIdx + 1).toInt();
      driveBTS(Motor_1_Left, L); driveBTS(Motor_3_Left, L);
      driveBTS(Motor_2_Right, R); driveBTS(Motor_4_Right, R);
    }
    // --- ARM MOVEMENT ---
    else if (cmd.startsWith("BASE")) stepperBase.move(cmd.substring(5).toInt());
    // --- GRIPPER (360 LIMIT) ---
    else if (cmd.startsWith("GRIP")) {
      int angle = constrain(cmd.substring(5).toInt(), 0, 180);
      gripper.write(angle);
    }
    else if (cmd == "STOP") {
      stopAll();
    }
  }
}

void initBTS(const int p[]) {
  for(int i=0; i<4; i++) pinMode(p[i], OUTPUT);
  digitalWrite(p[2], HIGH); digitalWrite(p[3], HIGH);
}

void driveBTS(const int p[], int spd) {
  spd = constrain(spd, -255, 255);
  if (spd > 0) { analogWrite(p[0], spd); analogWrite(p[1], 0); }
  else { analogWrite(p[0], 0); analogWrite(p[1], abs(spd)); }
}

void stopAll() {
  driveBTS(Motor_1_Left, 0); driveBTS(Motor_2_Right, 0);
  driveBTS(Motor_3_Left, 0); driveBTS(Motor_4_Right, 0);
  stepperBase.stop(); stepperJoint2.stop(); stepperJoint3.stop();
}