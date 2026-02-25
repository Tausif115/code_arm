#include <AccelStepper.h>
#include <Servo.h>

// BTS7960 Linear Actuators (DC Motors) 
const int BTS1[] = {5, 6, 22, 23};   
const int BTS2[] = {2, 3, 24, 25};   
const int BTS3[] = {7, 8,  9, 10};   

//  Steppers (PUL, DIR)
// Stepper 1: Main Base (DM860H) - Pins 12, 13
AccelStepper stepperBase(AccelStepper::DRIVER, 12, 13); 
// Stepper 2: Wrist/Tilt (TB6600) - Pins 40, 41
AccelStepper stepperTilt(AccelStepper::DRIVER, 40, 41); 

// Gripper Servo 
Servo gripper;
const int gripperPin = 30; 
#define ENA_ACTIVE LOW 

// 360 Degree Calibration: Set to 1600 if your TB6600 is on 1/8 microstepping
long fullRotationSteps = 1600; 

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

  // Stepper Tuning
  stepperBase.setMaxSpeed(800);  stepperBase.setAcceleration(400);
  
  // High acceleration for the 360 wrist to prevent sluggish response
  stepperTilt.setMaxSpeed(600);  
  stepperTilt.setAcceleration(1200); 
}

void loop() {
  // Constant execution for smooth movement
  stepperBase.run();
  stepperTilt.run();

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 0) return;

    // DC Linear Actuators 
    if (cmd.startsWith("DC")) {
      int joint = cmd.substring(2,3).toInt();
      int speed = constrain(cmd.substring(4).toInt(), -255, 255);
      if      (joint == 1) setBTS(BTS1, speed);
      else if (joint == 2) setBTS(BTS2, speed);
      else if (joint == 3) setBTS(BTS3, speed);
    }
    
    // Base Stepper (Relative move) 
    else if (cmd.startsWith("S1 ")) {
      stepperBase.move(cmd.substring(3).toInt()); 
    }
    
    // Wrist Stepper (360 Rotation Logic) 
    else if (cmd == "WRIST_CW")  stepperTilt.moveTo(fullRotationSteps);
    else if (cmd == "WRIST_CCW") stepperTilt.moveTo(-fullRotationSteps);
    else if (cmd == "WRIST_STOP") stepperTilt.stop();
    
    // Gripper Control (Pin 30) 
    else if (cmd.startsWith("GRIP ")) {
      int angle = cmd.substring(5).toInt();
      if(!gripper.attached()) gripper.attach(gripperPin);
      gripper.write(angle);
    }
    else if (cmd == "GRIP_OFF") {
      gripper.detach(); 
    }
    
    // --- Emergency Stop ---
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