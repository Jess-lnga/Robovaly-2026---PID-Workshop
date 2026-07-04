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
#include "controller.h"

static const uint32_t SERIAL_BAUDRATE = 115200;
static const uint32_t CONTROLLER_PERIOD_MS = 50;

static size_t previous_line_length = 0;
static int previous_tof1 = INT_MIN;
static int previous_tof2 = INT_MIN;
static int previous_ball_position = INT_MIN;
static int previous_ball_speed = INT_MIN;
static int previous_servo_angle = INT_MIN;
static bool previous_ball_speed_valid = false;
static bool previous_controller_valid = false;

static void display_status(void) {
  int tof1 = get_d1();
  int tof2 = get_d2();
  int ball_position = get_ball_position();
  int ball_speed = get_ball_speed();
  int servo_angle = get_controller_last_angle_deg();
  bool ball_speed_valid = is_ball_speed_valid();
  bool controller_valid = controller_last_update_was_valid();

  if (tof1 == previous_tof1 &&
      tof2 == previous_tof2 &&
      ball_position == previous_ball_position &&
      ball_speed == previous_ball_speed &&
      servo_angle == previous_servo_angle &&
      ball_speed_valid == previous_ball_speed_valid &&
      controller_valid == previous_controller_valid) {
    return;
  }

  char line[150];

  if (ball_speed_valid) {
    snprintf(line, sizeof(line),
             "D1: %4d mm    D2: %4d mm    X: %4d mm    V: %5d mm/s    Servo: %3d deg    PID: %s",
             tof1, tof2, ball_position, ball_speed, servo_angle,
             controller_valid ? "OK" : "KO");
  }
  else {
    snprintf(line, sizeof(line),
             "D1: %4d mm    D2: %4d mm    X: %4d mm    V:    -- mm/s    Servo: %3d deg    PID: %s",
             tof1, tof2, ball_position, servo_angle,
             controller_valid ? "OK" : "KO");
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
  previous_servo_angle = servo_angle;
  previous_ball_speed_valid = ball_speed_valid;
  previous_controller_valid = controller_valid;
}


void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  delay(200);

  init_tof(false);
  init_controller();
}

void loop() {
  update_controller();
  display_status();
  vTaskDelay(pdMS_TO_TICKS(CONTROLLER_PERIOD_MS));
}
