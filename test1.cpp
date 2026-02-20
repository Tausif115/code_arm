#include <AccelStepper.h>
#include <Servo.h>

//BTS7960 DC / Linear Actuator Pins(4 modules)
const int bts1_r_pwm = 5;
const int bts1_l_pwm = 6;
const int bts1_r_en  = 22;
const int bts1_l_en  = 23;

const int bts2_r_pwm = 2;
const int bts2_l_pwm = 3;
const int bts2_r_en  = 24;
const int bts2_l_en  = 25;

const int bts3_r_pwm = 7; // assuming 7 from your "37z" likely typo
const int bts3_l_pwm = 8;
const int bts3_r_en  = 9;
const int bts3_l_en  = 10;

//bts-4 not needed

//Stepper (DM860H) - one NEMA 23 
#define PUL_PIN 12
#define DIR_PIN 13
#define ENA_PIN 11
AccelStepper stepper(AccelStepper::DRIVER, PUL_PIN, DIR_PIN);  // STEP=PUL, DIR=DIR

//Gripper Servo
Servo gripper;
const int gripperPin = 26; // change to your actual pin (PWM capable, e.g. 9,12, etc.)

void setup() {
  Serial.begin(9600);
  Serial.println("Robotic Arm Ready (BTS7960 + DM860H + Servo)");

  // BTS7960 pins
  pinMode(bts1_r_pwm, OUTPUT); pinMode(bts1_l_pwm, OUTPUT);
  pinMode(bts1_r_en,  OUTPUT); pinMode(bts1_l_en,  OUTPUT);
  pinMode(bts2_r_pwm, OUTPUT); pinMode(bts2_l_pwm, OUTPUT);
  pinMode(bts2_r_en,  OUTPUT); pinMode(bts2_l_en,  OUTPUT);
  pinMode(bts3_r_pwm, OUTPUT); pinMode(bts3_l_pwm, OUTPUT);
  pinMode(bts3_r_en,  OUTPUT); pinMode(bts3_l_en,  OUTPUT);

  // Enable all BTS7960 bridges (set both EN HIGH)
  digitalWrite(bts1_r_en, HIGH); digitalWrite(bts1_l_en, HIGH);
  digitalWrite(bts2_r_en, HIGH); digitalWrite(bts2_l_en, HIGH);
  digitalWrite(bts3_r_en, HIGH); digitalWrite(bts3_l_en, HIGH);

  // Stepper config
  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(ENA_PIN, LOW);  // LOW = enabled on most DM860H (check your driver!)
  stepper.setMaxSpeed(800);  //have to adjust
  stepper.setAcceleration(400);
  stepper.setCurrentPosition(0);

  // Gripper
  gripper.attach(gripperPin);
  gripper.write(90);  // neutral / open — adjust
}

void loop() {
  stepper.run();   // non-blocking for the stepper

  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("DC")) { // DC joint control: DC<joint> <speed>   (-255..255)
      // Example: DC1 150   → forward medium, DC1 -200 → reverse faster
      int joint = cmd.substring(2,3).toInt();
      int speed = cmd.substring(4).toInt();

      speed = constrain(speed, -255, 255);

      if (joint == 1) setBTS(bts1_r_pwm, bts1_l_pwm, speed);
      else if (joint == 2) setBTS(bts2_r_pwm, bts2_l_pwm, speed);
      else if (joint == 3) setBTS(bts3_r_pwm, bts3_l_pwm, speed);
      // add BTS-4 if present

      Serial.print("DC Joint "); Serial.print(joint); Serial.print(" speed "); Serial.println(speed);
    }
    else if (cmd.startsWith("STEP")) {   // Relative steps: STEP <steps>
      long steps = cmd.substring(5).toInt();
      stepper.move(steps);
      Serial.print("Stepper moving "); Serial.println(steps);
    }
    else if (cmd.startsWith("POS")) {    // Absolute: POS <position>
      long pos = cmd.substring(4).toInt();
      stepper.moveTo(pos);
      Serial.print("Stepper to pos "); Serial.println(pos);
    }
    else if (cmd.startsWith("GRIP")) {
      int angle = cmd.substring(4).toInt();
      angle = constrain(angle, 0, 180);
      gripper.write(angle);
      Serial.print("Gripper to "); Serial.println(angle);
    }
    else if (cmd == "STOP") {
      setBTS(bts1_r_pwm, bts1_l_pwm, 0);
      setBTS(bts2_r_pwm, bts2_l_pwm, 0);
      setBTS(bts3_r_pwm, bts3_l_pwm, 0);
      stepper.stop();
      Serial.println("All stopped");
    }
  }
}

// Helper: Set speed on one BTS7960 (-255 reverse, +255 forward, 0 stop/brake)
void setBTS(int r_pwm_pin, int l_pwm_pin, int speed) {
  if (speed > 0) {
    analogWrite(r_pwm_pin, speed);
    analogWrite(l_pwm_pin, 0);
  } else if (speed < 0) {
    analogWrite(r_pwm_pin, 0);
    analogWrite(l_pwm_pin, -speed);   // positive PWM value
  } else {
    analogWrite(r_pwm_pin, 0);
    analogWrite(l_pwm_pin, 0);  // both low = fast decay / coast (or brake if you tie differently)
  }
}