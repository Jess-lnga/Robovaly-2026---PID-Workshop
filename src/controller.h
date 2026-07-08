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

#define CONTROLLER_DEFAULT_MAX_STEP_DEG 40
#define CONTROLLER_DEFAULT_POSITION_DEADBAND_MM 5
#define CONTROLLER_DEFAULT_SPEED_DEADBAND_MM_S 35
#define CONTROLLER_DEFAULT_LOST_BALL_DELAY_MS 3000
#define CONTROLLER_DEFAULT_LOST_BALL_ITER 3

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

bool set_controller_max_step_deg(int max_step_deg);
int get_controller_max_step_deg(void);

bool set_controller_stabilization_position_deadband_mm(int deadband_mm);
int get_controller_stabilization_position_deadband_mm(void);

bool set_controller_stabilization_speed_deadband_mm_s(int deadband_mm_s);
int get_controller_stabilization_speed_deadband_mm_s(void);

bool set_controller_lost_ball_delay_ms(uint32_t delay_ms);
uint32_t get_controller_lost_ball_delay_ms(void);

bool set_controller_lost_ball_iter(int iterations);
int get_controller_lost_ball_iter(void);

void reset_controller_advanced_parameters(void);

#ifdef __cplusplus
}
#endif

#endif // CONTROLLER_H
