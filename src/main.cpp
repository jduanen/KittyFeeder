/*
*
* RPM: 6-24 rpm
*  - 6 rpm: might overheat
*  - 10 rpm: safe, high torque
*  - 22 rpm: fast, low torque
*  - 24 rpm: might skip
*  - N.B. 15 rpm is about the fastest possible in practice with the door
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


//#include <Arduino.h>
#include <FeederDoor.h>


#ifdef ESP32
#define USE_ESP_IDF_LOG   1

#define LOG_LOCAL_LEVEL   ESP_LOG_DEBUG  // this overrides CONFIG_LOG_MAXIMUM_LEVEL setting in menuconfig
                                         // and must be defined before including esp_log.h
#define MY_GLOBAL_DEBUG_LEVEL  ESP_LOG_DEBUG
#include "esp_log.h"
#endif // ESP32

#define CMD_PIN   16  // GPIO16 D0  //// TMP TMP TMP


FeederDoor *door;

int cnt = 0;  //// TMP TMP TMP


void setup() {
  Serial.begin(115200);
  while (!Serial) { ; };
  Serial.println("BEGIN");

  pinMode(CMD_PIN, INPUT_PULLUP); //// TMP TMP TMP

  #ifdef ESP32
  esp_log_level_set("*", MY_GLOBAL_DEBUG_LEVEL)
  #endif  // ESP32

  door = new FeederDoor();

  Serial.println("START");
}

void loop() {
  door->loop();

  if (cnt == 250000) {
    Serial.println("OPEN");
    door->open();
  }

  if (cnt == 500000) {
    Serial.println("CLOSE");
  }

  if (cnt == 750000) {
    Serial.println("OPEN2");
  }
  cnt++;
}
