#include <Arduino.h>
#include <Stepper.h>


#define TAG   "FEEDER_DOOR"

// ULN2003 Motor Driver Pins
#define IN1  7   // GPIO7 D5
#define IN2  8   // GPIO8 D8
#define IN3  9   // GPIO9 D9
#define IN4  10  // GPIO10 D10

#define OPEN_SENSOR_PIN    5  // GPIO5 D3
#define CLOSED_SENSOR_PIN  4  // GPIO4 D2

#define STEPS_PER_REV   2048  // change this to fit the number of steps per revolution

#define MAX_MOVE_STEPS  (STEPS_PER_REV * 3)  // tune this value
#define MOVE_STEPS      128  // tune this value

#define CMD_PIN   3 // GPIO3 D1


typedef enum {
  CLOSING_DOOR,
  OPENING_DOOR,
  STOP_DOOR,
  ERROR_DOOR
} doorStates;


doorStates state = STOP_DOOR;


// initialize the stepper library
Stepper myStepper(STEPS_PER_REV, IN1, IN3, IN2, IN4);


bool isClosed() {
  return !digitalRead(CLOSED_SENSOR_PIN);
}

bool isOpen() {
  return !digitalRead(OPEN_SENSOR_PIN);
}

bool closeDoor() {
  int numSteps = 0;

  while (!isClosed()) {
    myStepper.step(-MOVE_STEPS);
    numSteps += MOVE_STEPS;
    if (numSteps > MAX_MOVE_STEPS) {
      return 1;
    }
  }
  return 0;
}

bool openDoor() {
  int numSteps = 0;

  while (!isOpen()) {
    myStepper.step(MOVE_STEPS);
    numSteps += MOVE_STEPS;
    if (numSteps > MAX_MOVE_STEPS) {
      return 1;
    }
  }
  return 0;
}

void stopDoor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; };
  Serial.println("BEGIN");

  pinMode(CMD_PIN, INPUT_PULLUP); //// TMP TMP TMP

  pinMode(OPEN_SENSOR_PIN, INPUT_PULLUP);
  pinMode(CLOSED_SENSOR_PIN, INPUT_PULLUP);
  delay(1000);
  Serial.print("Open: ");Serial.println(digitalRead(OPEN_SENSOR_PIN));
  Serial.print("Closed: ");Serial.println(digitalRead(OPEN_SENSOR_PIN));

  myStepper.setSpeed(4);  // N.B. 15 is about the fastest possible
  stopDoor();
  Serial.println("START");
}

void loop() {
  //// TMP TMP TMP
  if (!digitalRead(CMD_PIN)) {
    if (isOpen()) {
      state = CLOSING_DOOR;
      Serial.println("CLOSE");
    } else
    if (isClosed()) {
      state = OPENING_DOOR;
      Serial.println("OPEN");
    } else {
      ////Serial.println("NOP");
    }
  }

  switch (state) {
    case CLOSING_DOOR:
      if (closeDoor()) {
        ESP_LOGE(TAG, "Failed to sense door closed");
        state = ERROR_DOOR;
      } else {
        state = STOP_DOOR;
        stopDoor();
      }
      break;
    case OPENING_DOOR:
      if (openDoor()) {
        ESP_LOGE(TAG, "Failed to sense door open");
        state = ERROR_DOOR;
      } else {
        state = STOP_DOOR;
        stopDoor();
      }
      break;
    default:
      break;
  }
}
