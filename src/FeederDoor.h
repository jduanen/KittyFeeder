/*
* Class that encapsulates a stepper-driven door for the feeder
*
* This is designed for an ESP8266 controller, a ULN2003 driver board, a
*  28BYJ-48 stepper motor, and a pair of Hall-Effect sensors (that
*/

#pragma once

#include <Arduino.h>
#include <Stepper.h>


#ifdef ESP8266
#define ESP_LOGE(tag, msg)  Serial.print("ERROR: ");Serial.print(tag);Serial.println(msg);
#define ESP_LOGI(tag, msg)  Serial.print("INFO: ");Serial.print(tag);Serial.println(msg);
#endif // ESP8266


class FeederDoor {
  public:
    static constexpr const char *TAG = "feederDoor";

    static const unsigned char IN_1 = 15;  // GPIO15 D8
    static const unsigned char IN_2 = 13;  // GPIO13 D7
    static const unsigned char IN_3 = 12;  // GPIO12 D6
    static const unsigned char IN_4 = 14;  // GPIO14 D5

    static const unsigned char OPEN_SENSOR_PIN = 5;    // GPIO5 D1
    static const unsigned char CLOSED_SENSOR_PIN = 4;  // GPIO4 D2

    static const unsigned short STEPS_PER_REV = 2048;  // TODO check if this is correct
    static const unsigned short MAX_MOVE_STEPS = (2048 * 3);  // TODO fix this value
    static const unsigned short MOVE_STEPS = 128;   // TODO tune this value
    static const unsigned short STEPPER_SPEED = 4;  // TODO tune this value

    FeederDoor();
    FeederDoor(int stepsPerRev, int motorPin1, int motorPin2, int motorPin3, int motorPin4, int openSensor, int closedSensor);
    void setup(int stepsPerRev, int motorPin1, int motorPin2, int motorPin3, int motorPin4, int openSensor, int closedSensor);
    void loop();
    bool isClosed();
    bool isOpen();
    void close();
    void open();
  protected:
    typedef enum {
      CLOSING,
      OPENING,
      STOP,
      ERROR
    } doorStates;

    unsigned short _numSteps;
    doorStates _state = STOP;

    Stepper *_doorStepper;

    void _stopDoor();
};
