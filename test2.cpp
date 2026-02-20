#include <AccelStepper.h>
#include <Servo.h>

// BTS7960 groups: {RPWM, LPWM, REN, LEN}
const int BTS1[] = {5, 6, 22, 23};
const int BTS2[] = {2, 3, 24, 25};
const int BTS3[] = {7, 8,  9, 10};

// Stepper 1 (DM860H): PUL 12, DIR 13, ENA 11
AccelStepper stepper1(AccelStepper::DRIVER, 12, 13);

// Stepper 2: PUL 47, DIR 48, ENA 46  ← confirm these pins exist & are free! //important
AccelStepper stepper2(AccelStepper::DRIVER, 47, 48);

Servo gripper;
const int gripperPin = 9;           // Better: real PWM pin (Mega supports 9)

#define ENA_ACTIVE LOW              // Change to HIGH if your DM860H needs it

void setup() {
  Serial.begin(115200);             // Faster baud rate is better
  Serial.println("Robotic Arm Controller Started");

  // All BTS PWM & EN pins
  int allBtsPins[] = {2,3,5,6,7,8,9,10,22,23,24,25};
  for (int p : allBtsPins) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);           // Start safe
  }

  // Enable all BTS bridges (both EN HIGH)
  digitalWrite(BTS1[2], HIGH); digitalWrite(BTS1[3], HIGH);
  digitalWrite(BTS2[2], HIGH); digitalWrite(BTS2[3], HIGH);
  digitalWrite(BTS3[2], HIGH); digitalWrite(BTS3[3], HIGH);

  // Stepper enables – test this polarity!
  pinMode(11, OUTPUT); digitalWrite(11, ENA_ACTIVE);
  pinMode(46, OUTPUT); digitalWrite(46, ENA_ACTIVE);

  stepper1.setMaxSpeed(1200);    // Tune these
  stepper1.setAcceleration(600);
  stepper2.setMaxSpeed(1200);
  stepper2.setAcceleration(600);

  gripper.attach(gripperPin);
  gripper.write(90);  // Safe middle position
}

void loop() {
  stepper1.run();
  stepper2.run();

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 0) return;

    if (cmd.startsWith("DC")) {           // DC<joint> <speed>   e.g. DC1 180   DC2 -120
      int joint = cmd.substring(2,3).toInt();
      int speed = constrain(cmd.substring(4).toInt(), -255, 255);

      if      (joint == 1) setBTS(BTS1, speed);
      else if (joint == 2) setBTS(BTS2, speed);
      else if (joint == 3) setBTS(BTS3, speed);
      else                 Serial.println("Invalid DC joint");

      Serial.print("DC"); Serial.print(joint); Serial.print(" = "); Serial.println(speed);
    }
    else if (cmd.startsWith("S1 ")) {     // S1 <relative steps>
      long steps = cmd.substring(3).toInt();
      stepper1.move(steps);
    }
    else if (cmd.startsWith("S2 ")) {
      long steps = cmd.substring(3).toInt();
      stepper2.move(steps);
    }
    else if (cmd.startsWith("POS1 ")) {   // Optional: absolute
      long pos = cmd.substring(5).toInt();
      stepper1.moveTo(pos);
    }
    else if (cmd.startsWith("GRIP ")) {
      int angle = constrain(cmd.substring(5).toInt(), 0, 180);
      gripper.write(angle);
    }
    else if (cmd == "STOP" || cmd == "ESTOP") {
      setBTS(BTS1, 0); setBTS(BTS2, 0); setBTS(BTS3, 0);
      stepper1.stop(); stepper2.stop();
      Serial.println("Emergency STOP");
    }
    else {
      Serial.println("Unknown command");
    }
  }
}

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
    analogWrite(l_pwm, 0);          // Coast stop (safe default)
    // analogWrite(r_pwm, 255); analogWrite(l_pwm, 255); // Brake if desired
  }
}