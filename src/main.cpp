/*
Kit Robovaly - Prototype PID - 2026
Author: Jérôme ESSOLA ELANGA - Microtechnique EPFL
mail  : jerome.essolaelanga@epfl.ch
github: jess-lnga
*/

#include <Arduino.h>

#include "tof.h"

static const uint32_t SERIAL_BAUDRATE = 115200;
static const uint32_t PRINT_PERIOD_MS = 200;

static void print_distance(int16_t value_mm) {
  if (value_mm >= 0) {
    char value_buffer[8];
    snprintf(value_buffer, sizeof(value_buffer), "%4d", value_mm);
    Serial.print(value_buffer);
  } else {
    Serial.print("----");
  }
}

static void refresh_tof_display(void) {
  Serial.print("\r\033[2K");
  Serial.print("TOF 1 : ");
  print_distance(get_mes_tof_1());

  Serial.print(" mm");

  Serial.print("\n\033[2K");
  Serial.print("TOF 2 : ");
  print_distance(get_mes_tof_2());
  Serial.print(" mm");

  Serial.print("\033[1A");
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
