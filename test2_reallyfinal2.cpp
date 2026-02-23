#include <AccelStepper.h>
#include <Servo.h>

//BTS7960 Linear Actuators
const int BTS1[] = {5, 6, 22, 23};   
const int BTS2[] = {2, 3, 24, 25};   
const int BTS3[] = {7, 8,  9, 10};   

// (PUL, DIR)
// Stepper 1: Main Base (DM860H)
AccelStepper stepperBase(AccelStepper::DRIVER, 12, 13); 
// Stepper 2: Gripper Tilt (TB6600) - New NEMA 23
AccelStepper stepperTilt(AccelStepper::DRIVER, 40, 41); 

//Gripper Servo
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
  pinMode(11, OUTPUT); digitalWrite(11, ENA_ACTIVE); // DM860H Enable
  pinMode(42, OUTPUT); digitalWrite(42, ENA_ACTIVE); // TB6600 Enable

  // Stepper Tuning (Conservative for NEMA 23 torque)
  stepperBase.setMaxSpeed(800);  stepperBase.setAcceleration(400);
  stepperTilt.setMaxSpeed(600);  stepperTilt.setAcceleration(300);
}

void loop() {
  stepperBase.run();
  stepperTilt.run();

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 0) return;

    //DC Linear Actuators
    if (cmd.startsWith("DC")) {
      int joint = cmd.substring(2,3).toInt();
      int speed = constrain(cmd.substring(4).toInt(), -255, 255);
      if      (joint == 1) setBTS(BTS1, speed);
      else if (joint == 2) setBTS(BTS2, speed);
      else if (joint == 3) setBTS(BTS3, speed);
    }
    //Steppers
    else if (cmd.startsWith("S1 ")) stepperBase.move(cmd.substring(3).toInt()); // Base
    else if (cmd.startsWith("S2 ")) stepperTilt.move(cmd.substring(3).toInt()); // Tilt
    
    //Gripper Control
    else if (cmd.startsWith("GRIP ")) {
      if(!gripper.attached()) gripper.attach(gripperPin);
      gripper.write(cmd.substring(5).toInt());
    }
    else if (cmd == "GRIP_OFF") {
      gripper.detach(); 
    }
    else if (cmd == "STOP") {
      setBTS(BTS1, 0); setBTS(BTS2, 0); setBTS(BTS3, 0);
      stepperBase.stop(); stepperTilt.stop();
      gripper.detach();
    }
  }
}

void setBTS(const int bts[], int speed) {
  if (speed > 0) { analogWrite(bts[0], speed); analogWrite(bts[1], 0); }
  else if (speed < 0) { analogWrite(bts[0], 0); analogWrite(bts[1], -speed); }
  else { analogWrite(bts[0], 0); analogWrite(bts[1], 0); }
}