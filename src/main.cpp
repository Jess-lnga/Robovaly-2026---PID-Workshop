/*
Kit Robovaly - Prototype PID - 2026
Author: Jérôme ESSOLA ELANGA - Microtechnique EPFL
mail  : jerome.essolaelanga@epfl.ch
github: jess-lnga
*/

#include <Arduino.h>
#include "tof.h"
#include "ball_position.h"

static const uint32_t SERIAL_BAUDRATE = 115200;
static const uint32_t PRINT_PERIOD_MS = 100;
static const uint32_t POS_UPDATE_DELAY_MS = 50;


void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  delay(200);

  init_tof(false);
}

void loop() {
  display_distances();
  vTaskDelay(pdMS_TO_TICKS(POS_UPDATE_DELAY_MS));
}
