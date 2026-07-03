/*
Kit Robovaly - Prototype PID - 2026
Author: Jérôme ESSOLA ELANGA - Microtechnique EPFL
mail  : jerome.essolaelanga@epfl.ch
github: jess-lnga
*/

#include <Arduino.h>
#include "tof.h"

static const uint32_t SERIAL_BAUDRATE = 115200;
static const uint32_t PRINT_PERIOD_MS = 100;
static size_t previous_line_length = 0;

static void refresh_tof_display(void) {
  int16_t tof1 = get_mes_tof_1();
  int16_t tof2 = get_mes_tof_2();

  char line[48];
  snprintf(line, sizeof(line), "TOF 1: %4d mm    TOF 2: %4d mm", tof1, tof2);

  for (size_t i = 0; i < previous_line_length; i++) {
    Serial.print('\b');
  }

  Serial.print(line);
  previous_line_length = strlen(line);
}

void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  delay(200);

  Serial.println();
  Serial.println("Demarrage du test TOF...");

  if (launch_tof_mes()) {
    Serial.println("Tache de mesure TOF lancee.");
  } else {
    Serial.println("Erreur: aucun TOF initialise correctement.");
  }

  Serial.println();
  refresh_tof_display();
}

void loop() {
  refresh_tof_display();
  vTaskDelay(pdMS_TO_TICKS(PRINT_PERIOD_MS));
}
