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

static size_t previous_line_length = 0;
static int16_t previous_tof1 = INT16_MIN;
static int16_t previous_tof2 = INT16_MIN;
static int16_t previous_ball_position = INT16_MIN;
static int16_t previous_ball_speed = INT16_MIN;
static bool previous_ball_speed_valid = false;

void display_distances(){
  update_tof_distances();
  compute_ball_position();

  int16_t tof1 = get_d1();
  int16_t tof2 = get_d2();
  int16_t ball_position = get_ball_position();
  int16_t ball_speed = get_ball_speed();
  bool ball_speed_valid = is_ball_speed_valid();

  if (tof1 == previous_tof1 && tof2 == previous_tof2 &&
      ball_position == previous_ball_position &&
      ball_speed == previous_ball_speed &&
      ball_speed_valid == previous_ball_speed_valid) {
    return;
  }

  char line[110];

  if (ball_speed_valid) {
    snprintf(line, sizeof(line),
             "Distance 1: %4d mm    Distance 2: %4d mm    X: %4d mm    V: %5d mm/s",
             tof1, tof2, ball_position, ball_speed);
  }
  else {
    snprintf(line, sizeof(line),
             "Distance 1: %4d mm    Distance 2: %4d mm    X: %4d mm    V:    -- mm/s",
             tof1, tof2, ball_position);
  }

  for (size_t i = 0; i < previous_line_length; i++) {
    Serial.print('\b');
  }

  Serial.print(line);
  previous_line_length = strlen(line);
  previous_tof1 = tof1;
  previous_tof2 = tof2;
  previous_ball_position = ball_position;
  previous_ball_speed = ball_speed;
  previous_ball_speed_valid = ball_speed_valid;

}


void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  delay(200);

  init_tof(false);
}

void loop() {
  display_distances();
  vTaskDelay(pdMS_TO_TICKS(POS_UPDATE_DELAY_MS));
}
