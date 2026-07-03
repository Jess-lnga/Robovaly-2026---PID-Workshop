/*
Kit Robovaly - Prototype PID - 2026
Author: Jérôme ESSOLA ELANGA - Microtechnique EPFL
mail  : jerome.essolaelanga@epfl.ch
github: jess-lnga
*/

#include <Arduino.h>
#include "tof.h"
#include "ball_position.h"
#include "servo_cmd.h"

static const uint32_t SERIAL_BAUDRATE = 115200;
static const uint32_t PRINT_PERIOD_MS = 100;
static const uint32_t POS_UPDATE_DELAY_MS = 50;
static const uint32_t SERVO_CTRL_DELAY_MS = 20;
static const uint32_t ABS_INCR = 3;

static const int MIN_DEG = 0;
static const int MAX_DEG = 180;

static int angle = 0;
static int increment = ABS_INCR;



void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  delay(200);

  init_tof(false);
  init_servo_cmd();
}

void loop() {
  set_servo_angle(angle);
  angle += increment;
  
  if(angle > MAX_DEG - abs(increment)){
    increment = -ABS_INCR;
  }
  else if(angle < MIN_DEG + abs(increment)){
    increment = ABS_INCR;
  }

  vTaskDelay(pdMS_TO_TICKS(SERVO_CTRL_DELAY_MS));
}
