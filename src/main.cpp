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
static const uint32_t POS_UPDATE_DELAY_MS = 150;

static size_t previous_line_length = 0;
static int16_t previous_tof1 = INT16_MIN;
static int16_t previous_tof2 = INT16_MIN;

void display_distances(){
  update_tof_distances();

  int16_t tof1 = get_d1();
  int16_t tof2 = get_d2();

  if (tof1 == previous_tof1 && tof2 == previous_tof2) {
    return;
  }

  char line[48];
  snprintf(line, sizeof(line), "Distance 1: %4d mm    Distance 2: %4d mm", tof1, tof2);

  for (size_t i = 0; i < previous_line_length; i++) {
    Serial.print('\b');
  }

  Serial.print(line);
  previous_line_length = strlen(line);
  previous_tof1 = tof1;
  previous_tof2 = tof2;

}


void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  delay(200);

  init_tof(false);
}

void loop() {
  display_distances();
  vTaskDelay(pdMS_TO_TICKS(PRINT_PERIOD_MS));
}
