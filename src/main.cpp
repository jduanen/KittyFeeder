#include <Arduino.h>
#include <Stepper.h>


const int stepsPerRevolution = 2048; // change this to fit the number of steps per revolution

// ULN2003 Motor Driver Pins
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 10

// initialize the stepper library
Stepper myStepper(stepsPerRevolution, IN1, IN3, IN2, IN4);

bool cw = true;


void setup() {
  Serial.begin(115200);
  while (!Serial) { ; };
  delay(1000);
  Serial.println("BEGIN");

  myStepper.setSpeed(15);
  Serial.println("START");
}

void loop() {
  int steps;

  if (cw) {
    steps = -stepsPerRevolution;
    Serial.println("CW");
  } else {
    steps = stepsPerRevolution;
    Serial.println("CCW");
  }

  for (int revs = 0; (revs < 3); revs++) {
    myStepper.step(steps);
    Serial.print("  Rev: ");Serial.println(revs + 1);
    delay(100);
  }
  cw = !cw;
}
