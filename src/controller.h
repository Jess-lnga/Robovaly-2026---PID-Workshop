/*
Kit Robovaly - Prototype PID - 2026
Author: Jerome ESSOLA ELANGA - Microtechnique EPFL
mail  : jerome.essolaelanga@epfl.ch
github: jess-lnga
*/

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool init_controller(void);
bool update_controller(void);

void reset_controller(void);
void set_controller_enabled(bool enabled);
bool controller_is_enabled(void);
bool set_controller_manual_angle(int angle_deg);

void set_controller_reference_mm(int reference_mm);
int get_controller_reference_mm(void);

void set_controller_gains(float kp, float ki, float kd);
void get_controller_gains(float *kp, float *ki, float *kd);

int get_controller_last_angle_deg(void);
bool controller_last_update_was_valid(void);

#ifdef __cplusplus
}
#endif

#endif // CONTROLLER_H
