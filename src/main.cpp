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
}

void loop() {
  const int16_t mes_tof_1 = get_mes_tof_1();
  const int16_t mes_tof_2 = get_mes_tof_2();

  Serial.print("TOF 1");
  Serial.print(tof_1_is_initialized() ? " OK" : " KO");
  Serial.print(" | mesure: ");
  Serial.print(mes_tof_1);
  Serial.print(" mm | derniere maj: ");
  Serial.print(get_tof_1_last_update_ms());
  Serial.print(" ms");

  Serial.print("  ||  ");

  Serial.print("TOF 2");
  Serial.print(tof_2_is_initialized() ? " OK" : " KO");
  Serial.print(" | mesure: ");
  Serial.print(mes_tof_2);
  Serial.print(" mm | derniere maj: ");
  Serial.print(get_tof_2_last_update_ms());
  Serial.println(" ms");

  vTaskDelay(pdMS_TO_TICKS(PRINT_PERIOD_MS));
}
