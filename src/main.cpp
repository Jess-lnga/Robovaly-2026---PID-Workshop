/*
Kit Robovaly - Prototype PID - 2026
Author: Jérôme ESSOLA ELANGA - Microtechnique EPFL
mail  : jerome.essolaelanga@epfl.ch
github: jess-lnga
*/

#include <Arduino.h>
#include "tof.h"
#include "controller.h"
#include "interface_web.h"
#include "interface_usb.h"

static const uint32_t SERIAL_BAUDRATE = 115200;
static const BaseType_t CONTROLLER_TASK_CORE = 1;

static TaskHandle_t controller_task_handle = nullptr;

static void controller_task(void *pv) {
  (void)pv;

  TickType_t last_wake = xTaskGetTickCount();

  for (;;) {
    update_controller();
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(get_controller_period_ms()));
  }
}

void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  delay(200);

  init_tof(false);
  load_startup_persistent_settings();
  init_controller();
  launch_interface_usb();
  launch_interface_web();

  xTaskCreatePinnedToCore(
      controller_task,
      "TaskController",
      4096,
      nullptr,
      2,
      &controller_task_handle,
      CONTROLLER_TASK_CORE);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
