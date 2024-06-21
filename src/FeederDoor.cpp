/*
* Class that encapsulates a stepper-driven door for the feeder
*
* This is designed for an ESP8266 controller, a ULN2003 driver board, a
*  28BYJ-48 stepper motor, and a pair of Hall-Effect sensors (that
*
* RPM: 6-24 rpm
*  - 6 rpm: might overheat
*  - 10 rpm: safe, high torque
*  - 22 rpm: fast, low torque
*  - 24 rpm: might skip
*  - N.B. 15 rpm is about the fastest possible in practice
*
* Arduino stepper libraries:
*  - #include <Stepper.h>
*  - #include <TinyStepper.h>
*  - #include <TinyStepper_28BYJ_48.h>
*
* In one library, 4076 (4075.7728) mini-steps/rev are used if motor is geared 63.68395:1,
*  and 4096 is used if geared 64:1 (need to measure to find out which yours is)
* All assume 5V power supply for rpm calculations.
* One library's default speed is ~16.25 rpm
*
* N.B. Actually using WeMos D1 NodeMCU Mini ESP8266
*/

#include "Arduino.h"
#include "FeederDoor.h"


FeederDoor::FeederDoor() {
	setup(STEPS_PER_REV, IN_1, IN_3, IN_2, IN_4, OPEN_SENSOR_PIN, CLOSED_SENSOR_PIN);
}

FeederDoor::FeederDoor(int stepsPerRev, int motorPin1, int motorPin2, int motorPin3, int motorPin4, int openSensorPin, int closedSensorPin) {
	setup(stepsPerRev, motorPin1, motorPin2, motorPin3, motorPin4, openSensorPin, closedSensorPin);
}

void FeederDoor::setup(int stepsPerRev, int motorPin1, int motorPin2, int motorPin3, int motorPin4, int openSensorPin, int closedSensorPin) {
	_doorStepper = new Stepper(stepsPerRev, motorPin1, motorPin2, motorPin3, motorPin4);
  _stopDoor();

	pinMode(openSensorPin, INPUT_PULLUP);
  pinMode(closedSensorPin, INPUT_PULLUP);

  ESP_LOGI(TAG, isOpen() ? "Open: True" : "Open: False");
  ESP_LOGI(TAG, isClosed() ? "Closed: True" : "Closed: False");

  _doorStepper->setSpeed(STEPPER_SPEED);  // low speed, high torque
}

void FeederDoor::loop() {
	switch (_state) {
	case CLOSING:
		if (isClosed()) {
			_state = STOP;
			_numSteps = 0;
		} else {
			if (_numSteps > MAX_MOVE_STEPS) {
				_state = ERROR;
			} else {
				_numSteps += MOVE_STEPS;
				_doorStepper->step(-MOVE_STEPS);
			}
		}
		break;
	case OPENING:
		if (isOpen()) {
			_state = STOP;
			_numSteps = 0;
		} else {
			if (_numSteps > MAX_MOVE_STEPS) {
				_state = ERROR;
			} else {
				_numSteps += MOVE_STEPS;
				_doorStepper->step(MOVE_STEPS);
			}
		}
		break;
	case STOP:
		break;
	case ERROR:
		ESP_LOGE(TAG, "Error state");
		_state = STOP;
		//// TODO reset?
		break;
	default:
		ESP_LOGE(TAG, "Invalid state");
		//// TODO reset?
		break;
	}
}

bool FeederDoor::isClosed() {
	return !digitalRead(CLOSED_SENSOR_PIN);
}

bool FeederDoor::isOpen() {
	return !digitalRead(OPEN_SENSOR_PIN);
}

void FeederDoor::close() {
	if (_state == CLOSING) {
		return;
	}
	_state = CLOSING;
}

void FeederDoor::open() {
	_state = OPENING;
}

void FeederDoor::_stopDoor() {
  digitalWrite(IN_1, LOW);
  digitalWrite(IN_2, LOW);
  digitalWrite(IN_3, LOW);
  digitalWrite(IN_4, LOW);
}
