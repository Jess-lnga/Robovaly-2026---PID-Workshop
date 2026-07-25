#include "tof.h"

#include <Adafruit_VL53L0X.h>
#include <Arduino.h>
#include <Wire.h>

static const int SDA_PIN = 21;
static const int SCL_PIN = 22;

static const int XSHUT_TOF_1 = 16;
static const int XSHUT_TOF_2 = 17;

static const uint8_t DEFAULT_TOF_ADDR = 0x29;
static const uint8_t TOF_1_ADDR = 0x30;
static const uint8_t TOF_2_ADDR = 0x31;

static const uint32_t I2C_CLOCK_HZ = 50000;
static const uint16_t I2C_TIMEOUT_MS = 500;

static const uint32_t TOF_TIMING_BUDGET_US = 20000;
static const uint32_t TOF_SLOT_MS = 25;
static const uint32_t TOF_POLL_PERIOD_MS = 1;
static const BaseType_t TOF_TASK_CORE = 1;
static const uint32_t TOF_BOOT_DELAY_MS = 150;

static Adafruit_VL53L0X tof_1;
static Adafruit_VL53L0X tof_2;

static portMUX_TYPE tof_mux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t tof_task_handle = nullptr;

static volatile int16_t mes_tof_1 = -1;
static volatile int16_t mes_tof_2 = -1;
static volatile bool tof_1_ok = false;
static volatile bool tof_2_ok = false;
static volatile uint32_t tof_1_last_update_ms = 0;
static volatile uint32_t tof_2_last_update_ms = 0;

static void set_tof_1_measurement(int16_t value_mm) {
  portENTER_CRITICAL(&tof_mux);
  mes_tof_1 = value_mm;
  tof_1_last_update_ms = millis();
  portEXIT_CRITICAL(&tof_mux);
}

static void set_tof_2_measurement(int16_t value_mm) {
  portENTER_CRITICAL(&tof_mux);
  mes_tof_2 = value_mm;
  tof_2_last_update_ms = millis();
  portEXIT_CRITICAL(&tof_mux);
}

static void set_init_status(bool tof_1_status, bool tof_2_status) {
  portENTER_CRITICAL(&tof_mux);
  tof_1_ok = tof_1_status;
  tof_2_ok = tof_2_status;
  portEXIT_CRITICAL(&tof_mux);
}

static void tofs_all_off(void) {
  digitalWrite(XSHUT_TOF_1, LOW);
  digitalWrite(XSHUT_TOF_2, LOW);
  delay(TOF_BOOT_DELAY_MS);
}

enum TofTaskState {
  START_TOF_1,
  WAIT_TOF_1,
  READ_TOF_1,
  START_TOF_2,
  WAIT_TOF_2,
  READ_TOF_2,
};

static bool start_tof(Adafruit_VL53L0X &sensor, int xshut_pin, uint8_t new_addr) {
  digitalWrite(xshut_pin, HIGH);
  delay(TOF_BOOT_DELAY_MS);

  if (!sensor.begin(DEFAULT_TOF_ADDR, false, &Wire)) {
    return false;
  }
  if (!sensor.setAddress(new_addr)) {
    return false;
  }

  if (!sensor.setMeasurementTimingBudgetMicroSeconds(TOF_TIMING_BUDGET_US)) {
    return false;
  }

  return true;
}

static bool init_tofs(void) {
  pinMode(XSHUT_TOF_1, OUTPUT);
  pinMode(XSHUT_TOF_2, OUTPUT);

  Wire.begin(SDA_PIN, SCL_PIN, I2C_CLOCK_HZ);
  Wire.setTimeOut(I2C_TIMEOUT_MS);

  tofs_all_off();

  bool first_ok = start_tof(tof_1, XSHUT_TOF_1, TOF_1_ADDR);
  if (!first_ok) {
    digitalWrite(XSHUT_TOF_1, LOW);
    delay(TOF_BOOT_DELAY_MS);
  }

  bool second_ok = start_tof(tof_2, XSHUT_TOF_2, TOF_2_ADDR);
  if (!second_ok) {
    digitalWrite(XSHUT_TOF_2, LOW);
    delay(TOF_BOOT_DELAY_MS);
  }

  if (!first_ok) {
    Serial.println("Erreur init TOF 1");
  }
  if (!second_ok) {
    Serial.println("Erreur init TOF 2");
  }

  set_init_status(first_ok, second_ok);
  return first_ok || second_ok;
}

static void read_one_tof(Adafruit_VL53L0X &sensor,
                         void (*store_measurement)(int16_t)) {
  uint16_t range_mm = sensor.readRangeResult();
  if (sensor.readRangeStatus() != 0 || range_mm == 0 || range_mm > INT16_MAX) {
    store_measurement(-1);
    return;
  }

  store_measurement((int16_t)range_mm);
}

static void tof_task(void *pv) {
  (void)pv;

  TickType_t last_wake = xTaskGetTickCount();
  TofTaskState state = START_TOF_1;
  uint32_t measure_start_ms = 0;
  bool measure_ready = false;

  for (;;) {
    bool first_ok;
    bool second_ok;

    portENTER_CRITICAL(&tof_mux);
    first_ok = tof_1_ok;
    second_ok = tof_2_ok;
    portEXIT_CRITICAL(&tof_mux);

    uint32_t now = millis();

    switch (state) {
      case START_TOF_1:
        if (!first_ok) {
          state = START_TOF_2;
          break;
        }
        measure_start_ms = now;
        measure_ready = false;
        if (tof_1.startRange()) {
          state = WAIT_TOF_1;
        } else {
          set_tof_1_measurement(-1);
          state = START_TOF_2;
        }
        break;

      case WAIT_TOF_1:
        measure_ready = tof_1.isRangeComplete();
        if (measure_ready || (now - measure_start_ms >= TOF_SLOT_MS)) {
          state = READ_TOF_1;
        }
        break;

      case READ_TOF_1:
        if (measure_ready) {
          read_one_tof(tof_1, set_tof_1_measurement);
        } else {
          set_tof_1_measurement(-1);
          tof_1.stopMeasurement();
        }
        state = START_TOF_2;
        break;

      case START_TOF_2:
        if (!second_ok) {
          state = START_TOF_1;
          break;
        }
        measure_start_ms = now;
        measure_ready = false;
        if (tof_2.startRange()) {
          state = WAIT_TOF_2;
        } else {
          set_tof_2_measurement(-1);
          state = START_TOF_1;
        }
        break;

      case WAIT_TOF_2:
        measure_ready = tof_2.isRangeComplete();
        if (measure_ready || (now - measure_start_ms >= TOF_SLOT_MS)) {
          state = READ_TOF_2;
        }
        break;

      case READ_TOF_2:
        if (measure_ready) {
          read_one_tof(tof_2, set_tof_2_measurement);
        } else {
          set_tof_2_measurement(-1);
          tof_2.stopMeasurement();
        }
        state = START_TOF_1;
        break;
    }

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TOF_POLL_PERIOD_MS));
  }
}

bool launch_tof_mes(void) {
  if (tof_task_handle != nullptr) {
    return true;
  }

  bool any_tof_ok = init_tofs();
  BaseType_t created = xTaskCreatePinnedToCore(
      tof_task, "TaskTOF", 4096, nullptr, 2, &tof_task_handle, TOF_TASK_CORE);

  if (created != pdPASS) {
    tof_task_handle = nullptr;
    return false;
  }

  return any_tof_ok;
}

int16_t get_mes_tof_1(void) {
  int16_t value;
  portENTER_CRITICAL(&tof_mux);
  value = mes_tof_1;
  portEXIT_CRITICAL(&tof_mux);
  return value;
}

int16_t get_mes_tof_2(void) {
  int16_t value;
  portENTER_CRITICAL(&tof_mux);
  value = mes_tof_2;
  portEXIT_CRITICAL(&tof_mux);
  return value;
}

bool tof_1_is_initialized(void) {
  bool value;
  portENTER_CRITICAL(&tof_mux);
  value = tof_1_ok;
  portEXIT_CRITICAL(&tof_mux);
  return value;
}

bool tof_2_is_initialized(void) {
  bool value;
  portENTER_CRITICAL(&tof_mux);
  value = tof_2_ok;
  portEXIT_CRITICAL(&tof_mux);
  return value;
}

uint32_t get_tof_1_last_update_ms(void) {
  uint32_t value;
  portENTER_CRITICAL(&tof_mux);
  value = tof_1_last_update_ms;
  portEXIT_CRITICAL(&tof_mux);
  return value;
}

uint32_t get_tof_2_last_update_ms(void) {
  uint32_t value;
  portENTER_CRITICAL(&tof_mux);
  value = tof_2_last_update_ms;
  portEXIT_CRITICAL(&tof_mux);
  return value;
}

static size_t previous_line_length = 0;
static int16_t previous_tof1 = INT16_MIN;
static int16_t previous_tof2 = INT16_MIN;

void tof_display(void) {
  int16_t tof1 = get_mes_tof_1();
  int16_t tof2 = get_mes_tof_2();

  if (tof1 == previous_tof1 && tof2 == previous_tof2) {
    return;
  }

  char line[48];
  snprintf(line, sizeof(line), "TOF 1: %4d mm    TOF 2: %4d mm", tof1, tof2);

  for (size_t i = 0; i < previous_line_length; i++) {
    Serial.print('\b');
  }

  Serial.print(line);
  previous_line_length = strlen(line);
  previous_tof1 = tof1;
  previous_tof2 = tof2;
}


void init_tof(bool debug){
  
  if(debug){
    Serial.println();
    Serial.println("Demarrage du test TOF...");
  }
  

  if (launch_tof_mes()) {
    if(debug){Serial.println("Tache de mesure TOF lancee.");}
  } else {
    if(debug){Serial.println("Erreur: aucun TOF initialise correctement.");}
  }

  if(debug){
    Serial.println();
    tof_display();
  }

}
