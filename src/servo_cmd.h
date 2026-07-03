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

#ifdef __cplusplus
extern "C" {
#endif

bool init_servo_cmd(uint8_t servo_pin);

bool set_servo_angle(int angle_deg);
void set_servo_angle_limits(int min_angle_deg, int max_angle_deg);
void set_servo_pulse_limits_us(uint16_t min_pulse_us, uint16_t max_pulse_us);

bool servo_cmd_is_initialized(void);
int get_servo_angle(void);

#ifdef __cplusplus
}
#endif

#endif // SERVO_CMD_H
