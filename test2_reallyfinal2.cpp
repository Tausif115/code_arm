#include <AccelStepper.h>
#include <Servo.h>

// BTS7960 Linear Actuators: {RPWM, LPWM, REN, LEN} 
const int BTS1[] = {5, 6, 22, 23};   
const int BTS2[] = {2, 3, 24, 25};   
const int BTS3[] = {7, 8,  9, 10};   

// --- Steppers (PUL, DIR) ---
AccelStepper stepper1(AccelStepper::DRIVER, 12, 13); // NEMA 23 (DM860H)
AccelStepper stepper2(AccelStepper::DRIVER, 47, 48); // NEMA 17 (TB6600)
AccelStepper stepper3(AccelStepper::DRIVER, 49, 50); // NEMA 17 (TB6600)

// Gripper 
Servo gripper;
const int gripperPin = 9; 
#define ENA_ACTIVE LOW 

void setup() {
  Serial.begin(115200);

  // Initialize BTS7960 Pins
  int allBtsPins[] = {2,3,5,6,7,8,9,10,22,23,24,25};
  for (int p : allBtsPins) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }

  // Enable BTS Bridges
  digitalWrite(BTS1[2], HIGH); digitalWrite(BTS1[3], HIGH);
  digitalWrite(BTS2[2], HIGH); digitalWrite(BTS2[3], HIGH);
  digitalWrite(BTS3[2], HIGH); digitalWrite(BTS3[3], HIGH);

  // Stepper Enable Pins
  pinMode(11, OUTPUT); digitalWrite(11, ENA_ACTIVE); // NEMA 23
  pinMode(46, OUTPUT); digitalWrite(46, ENA_ACTIVE); // NEMA 17-1
  pinMode(51, OUTPUT); digitalWrite(51, ENA_ACTIVE); // NEMA 17-2

  // Stepper Tuning
  stepper1.setMaxSpeed(800);  stepper1.setAcceleration(400);
  stepper2.setMaxSpeed(1200); stepper2.setAcceleration(600);
  stepper3.setMaxSpeed(1200); stepper3.setAcceleration(600);
}

void loop() {
  stepper1.run();
  stepper2.run();
  stepper3.run();

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 0) return;

    // Linear Actuators
    if (cmd.startsWith("DC")) {
      int joint = cmd.substring(2,3).toInt();
      int speed = constrain(cmd.substring(4).toInt(), -255, 255);
      if (joint == 1) setBTS(BTS1, speed);
      else if (joint == 2) setBTS(BTS2, speed);
      else if (joint == 3) setBTS(BTS3, speed);
    }
    // Steppers (Relative Move)
    else if (cmd.startsWith("S1 ")) stepper1.move(cmd.substring(3).toInt());
    else if (cmd.startsWith("S2 ")) stepper2.move(cmd.substring(3).toInt());
    else if (cmd.startsWith("S3 ")) stepper3.move(cmd.substring(3).toInt());
    
    // Gripper Control (with Auto-Attach)
    else if (cmd.startsWith("GRIP ")) {
      if(!gripper.attached()) gripper.attach(gripperPin);
      gripper.write(cmd.substring(5).toInt());
    }
    else if (cmd == "GRIP_OFF") {
      gripper.detach(); // Stops Jittering
    }
    else if (cmd == "STOP") {
      setBTS(BTS1, 0); setBTS(BTS2, 0); setBTS(BTS3, 0);
      stepper1.stop(); stepper2.stop(); stepper3.stop();
      gripper.detach();
    }
  }
}

void setBTS(const int bts[], int speed) {
  if (speed > 0) { analogWrite(bts[0], speed); analogWrite(bts[1], 0); }
  else if (speed < 0) { analogWrite(bts[0], 0); analogWrite(bts[1], -speed); }
  else { analogWrite(bts[0], 0); analogWrite(bts[1], 0); }
}