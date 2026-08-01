/*
Kit Robovaly - Prototype PID - 2026
Author: Jerome ESSOLA ELANGA - Microtechnique EPFL
mail  : jerome.essolaelanga@epfl.ch
github: jess-lnga
*/

#ifndef SERVO_CMD_H
#define SERVO_CMD_H

#include <stdbool.h>
#include <stdint.h>

#define SERVO_CMD_DEFAULT_PIN 18
#define SERVO_CMD_NEUTRAL_DEG 90
#define SERVO_CMD_MIN_DEG 0
#define SERVO_CMD_MAX_DEG 180
#define SERVO_CMD_MIN_PULSE_US 1000
#define SERVO_CMD_NEUTRAL_PULSE_US 1500
#define SERVO_CMD_MAX_PULSE_US 2000
#define SERVO_CMD_DEFAULT_PWM_STEP_US 10

#ifdef __cplusplus
extern "C" {
#endif

bool init_servo_cmd(void);
bool init_servo_cmd_on_pin(uint8_t servo_pin);

bool set_servo_angle(int angle_deg);
bool set_servo_angle_limits(int min_angle_deg, int max_angle_deg);
bool set_servo_theoretical_angle_range(int min_angle_deg, int max_angle_deg);
bool set_servo_angle_range(int min_angle_deg, int max_angle_deg, int neutral_angle_deg);
bool set_servo_neutral_angle_deg(int neutral_angle_deg);
void set_servo_pulse_limits_us(uint16_t min_pulse_us, uint16_t max_pulse_us);
bool set_servo_neutral_offset_us(int offset_us);
int get_servo_neutral_offset_us(void);
bool set_servo_calibration_pulse_us(uint16_t pulse_us);
uint16_t get_servo_current_pulse_us(void);

bool servo_cmd_is_initialized(void);
int get_servo_angle(void);
int get_servo_min_angle_deg(void);
int get_servo_max_angle_deg(void);
int get_servo_neutral_angle_deg(void);
int get_servo_theoretical_min_angle_deg(void);
int get_servo_theoretical_max_angle_deg(void);
void reset_servo_advanced_parameters(void);

#ifdef __cplusplus
}
#endif

#endif // SERVO_CMD_H
