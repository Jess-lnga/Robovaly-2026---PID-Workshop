#include <Arduino.h>
#include "controller.h"
#include "ball_position.h"
#include "servo_cmd.h"

static const int DEFAULT_REFERENCE_MM = 150;
static const uint32_t MIN_CONTROLLER_DT_MS = 5;
static const float INTEGRAL_LIMIT_MM_S = 3000.0f;
static const int SERVO_DEADBAND_DEG = 0;
static const float SERVO_FILTER_ALPHA = 1.0f;
static const int LOST_BALL_RETURN_STEP_DEG = 2;

static bool controller_initialized = false;
static bool controller_enabled = false;
static bool last_update_valid = false;

static int reference_mm = DEFAULT_REFERENCE_MM;
static int last_angle_deg = SERVO_CMD_NEUTRAL_DEG;
static int servo_max_step_deg = CONTROLLER_DEFAULT_MAX_STEP_DEG;
static int position_deadband_mm = CONTROLLER_DEFAULT_POSITION_DEADBAND_MM;
static int speed_deadband_mm_s = CONTROLLER_DEFAULT_SPEED_DEADBAND_MM_S;
static uint32_t lost_ball_delay_ms = CONTROLLER_DEFAULT_LOST_BALL_DELAY_MS;
static int lost_ball_iter = CONTROLLER_DEFAULT_LOST_BALL_ITER;
static int max_control_speed_mm_s = CONTROLLER_DEFAULT_MAX_CONTROL_SPEED_MM_S;
static uint32_t controller_period_ms = CONTROLLER_DEFAULT_PERIOD_MS;

static float kp = 0.15f;
static float ki = 0.1f;
static float kd = 0.13f;

static float integral_error_mm_s = 0.0f;
static float filtered_angle_deg = SERVO_CMD_NEUTRAL_DEG;
static uint32_t last_update_ms = 0;
static uint32_t lost_ball_start_ms = 0;
static int lost_ball_count = 0;

static int clamp_int(int value, int min_value, int max_value) {
    if(value < min_value) return min_value;
    if(value > max_value) return max_value;
    return value;
}

static float clamp_float(float value, float min_value, float max_value) {
    if(value < min_value) return min_value;
    if(value > max_value) return max_value;
    return value;
}

static float soften_deadband_signal(float value, float deadband) {
    if(deadband <= 0.0f) {
        return value;
    }

    float magnitude = fabsf(value);
    if(magnitude >= deadband) {
        return value;
    }

    float attenuation = magnitude / deadband;
    return value * attenuation;
}

static int command_to_servo_angle(float pid_command_deg) {
    float angle = (float)get_servo_neutral_angle_deg() - pid_command_deg;
    return clamp_int((int)lroundf(angle), get_servo_min_angle_deg(), get_servo_max_angle_deg());
}

static int smooth_servo_angle(int raw_angle_deg, int max_step_deg) {
    filtered_angle_deg += SERVO_FILTER_ALPHA * ((float)raw_angle_deg - filtered_angle_deg);

    int filtered_angle_int = clamp_int((int)lroundf(filtered_angle_deg),
                                       get_servo_min_angle_deg(),
                                       get_servo_max_angle_deg());

    if(abs(filtered_angle_int - last_angle_deg) < SERVO_DEADBAND_DEG) {
        return last_angle_deg;
    }

    return clamp_int(filtered_angle_int,
                     last_angle_deg - max_step_deg,
                     last_angle_deg + max_step_deg);
}

static void reset_pid_terms(void) {
    integral_error_mm_s = 0.0f;
    last_update_ms = millis();
    filtered_angle_deg = (float)last_angle_deg;
}

static bool command_smoothed_angle(int raw_angle_deg, int max_step_deg) {
    int angle_deg = smooth_servo_angle(raw_angle_deg, max_step_deg);

    if(angle_deg == last_angle_deg) {
        last_update_valid = true;
        return true;
    }

    if(set_servo_angle(angle_deg)) {
        last_angle_deg = get_servo_angle();
        last_update_valid = true;
        return true;
    }

    last_update_valid = false;
    return false;
}

static bool command_smoothed_angle(int raw_angle_deg) {
    return command_smoothed_angle(raw_angle_deg, servo_max_step_deg);
}

static bool handle_lost_ball(uint32_t now) {
    if(lost_ball_start_ms == 0) {
        lost_ball_start_ms = now;
    }

    lost_ball_count++;

    if(lost_ball_count >= lost_ball_iter) {
        reset_pid_terms();
    }

    if(now - lost_ball_start_ms >= lost_ball_delay_ms) {
        command_smoothed_angle(get_servo_neutral_angle_deg(), LOST_BALL_RETURN_STEP_DEG);
        last_update_valid = false;
    }
    else {
        last_update_valid = false;
    }

    return false;
}

bool init_controller(void) {
    if(!servo_cmd_is_initialized()) {
        if(!init_servo_cmd()) {
            controller_initialized = false;
            return false;
        }
    }

    set_servo_angle(get_servo_neutral_angle_deg());
    reset_controller();
    controller_initialized = true;
    controller_enabled = false;
    return set_servo_angle(last_angle_deg);
}

void reset_controller(void) {
    integral_error_mm_s = 0.0f;
    last_update_ms = millis();
    last_update_valid = false;
    last_angle_deg = servo_cmd_is_initialized() ? get_servo_angle() : get_servo_neutral_angle_deg();
    filtered_angle_deg = (float)last_angle_deg;
    lost_ball_start_ms = 0;
    lost_ball_count = 0;
}

bool update_controller(void) {
    if(!controller_initialized) {
        return false;
    }

    update_tof_distances();
    bool position_valid = compute_ball_position();
    bool speed_valid = is_ball_speed_valid();
    uint32_t now = millis();

    if(!controller_enabled) {
        last_update_valid = position_valid;
        return position_valid;
    }

    if(!position_valid) {
        return handle_lost_ball(now);
    }

    lost_ball_start_ms = 0;
    lost_ball_count = 0;

    uint32_t dt_ms = now - last_update_ms;

    if(dt_ms < MIN_CONTROLLER_DT_MS) {
        last_update_valid = false;
        return false;
    }

    float dt_s = dt_ms * 0.001f;
    last_update_ms = now;

    int position_mm = get_ball_position();
    int speed_mm_s = speed_valid ? get_ball_speed() : 0;
    speed_mm_s = clamp_int(speed_mm_s, -max_control_speed_mm_s, max_control_speed_mm_s);

    float error_mm = (float)reference_mm - (float)position_mm;
    float effective_error_mm = soften_deadband_signal(error_mm, (float)position_deadband_mm);
    float effective_speed_mm_s = speed_valid
                                 ? soften_deadband_signal((float)speed_mm_s,
                                                          (float)speed_deadband_mm_s)
                                 : 0.0f;

    integral_error_mm_s += effective_error_mm * dt_s;
    integral_error_mm_s = clamp_float(integral_error_mm_s,
                                      -INTEGRAL_LIMIT_MM_S,
                                      INTEGRAL_LIMIT_MM_S);

    if(fabsf(effective_error_mm) < 0.001f) {
        integral_error_mm_s = 0.0f;
    }

    float pid_command_deg = kp * effective_error_mm +
                            ki * integral_error_mm_s -
                            kd * effective_speed_mm_s;

    int raw_angle_deg = command_to_servo_angle(pid_command_deg);
    return command_smoothed_angle(raw_angle_deg);
}

void set_controller_enabled(bool enabled) {
    controller_enabled = enabled;
    reset_controller();
}

bool controller_is_enabled(void) {
    return controller_enabled;
}

bool set_controller_manual_angle(int angle_deg) {
    if(!controller_initialized || controller_enabled) {
        return false;
    }

    if(set_servo_angle(angle_deg)) {
        last_angle_deg = get_servo_angle();
        filtered_angle_deg = (float)last_angle_deg;
        return true;
    }

    return false;
}

void set_controller_reference_mm(int new_reference_mm) {
    reference_mm = new_reference_mm;
    reset_controller();
}

int get_controller_reference_mm(void) {
    return reference_mm;
}

void set_controller_gains(float new_kp, float new_ki, float new_kd) {
    kp = new_kp;
    ki = new_ki;
    kd = new_kd;
    reset_controller();
}

void get_controller_gains(float *out_kp, float *out_ki, float *out_kd) {
    if(out_kp != nullptr) *out_kp = kp;
    if(out_ki != nullptr) *out_ki = ki;
    if(out_kd != nullptr) *out_kd = kd;
}

int get_controller_last_angle_deg(void) {
    return last_angle_deg;
}

bool controller_last_update_was_valid(void) {
    return last_update_valid;
}

bool set_controller_max_step_deg(int max_step_deg) {
    if(max_step_deg < 0 || max_step_deg > 180) {
        return false;
    }

    servo_max_step_deg = max_step_deg;
    return true;
}

int get_controller_max_step_deg(void) {
    return servo_max_step_deg;
}

bool set_controller_stabilization_position_deadband_mm(int deadband_mm) {
    if(deadband_mm < 0 || deadband_mm > 50) {
        return false;
    }

    position_deadband_mm = deadband_mm;
    return true;
}

int get_controller_stabilization_position_deadband_mm(void) {
    return position_deadband_mm;
}

bool set_controller_stabilization_speed_deadband_mm_s(int deadband_mm_s) {
    if(deadband_mm_s < 0 || deadband_mm_s > 300) {
        return false;
    }

    speed_deadband_mm_s = deadband_mm_s;
    return true;
}

int get_controller_stabilization_speed_deadband_mm_s(void) {
    return speed_deadband_mm_s;
}

bool set_controller_lost_ball_delay_ms(uint32_t delay_ms) {
    if(delay_ms > 10000) {
        return false;
    }

    lost_ball_delay_ms = delay_ms;
    return true;
}

uint32_t get_controller_lost_ball_delay_ms(void) {
    return lost_ball_delay_ms;
}

bool set_controller_lost_ball_iter(int iterations) {
    if(iterations < 1 || iterations > 20) {
        return false;
    }

    lost_ball_iter = iterations;
    return true;
}

int get_controller_lost_ball_iter(void) {
    return lost_ball_iter;
}

bool set_controller_max_control_speed_mm_s(int max_speed_mm_s) {
    if(max_speed_mm_s < 0 || max_speed_mm_s > 2000) {
        return false;
    }

    max_control_speed_mm_s = max_speed_mm_s;
    return true;
}

int get_controller_max_control_speed_mm_s(void) {
    return max_control_speed_mm_s;
}

bool set_controller_period_ms(uint32_t period_ms) {
    if(period_ms < 10 || period_ms > 100) {
        return false;
    }

    controller_period_ms = period_ms;
    reset_controller();
    return true;
}

uint32_t get_controller_period_ms(void) {
    return controller_period_ms;
}

void reset_controller_advanced_parameters(void) {
    servo_max_step_deg = CONTROLLER_DEFAULT_MAX_STEP_DEG;
    position_deadband_mm = CONTROLLER_DEFAULT_POSITION_DEADBAND_MM;
    speed_deadband_mm_s = CONTROLLER_DEFAULT_SPEED_DEADBAND_MM_S;
    lost_ball_delay_ms = CONTROLLER_DEFAULT_LOST_BALL_DELAY_MS;
    lost_ball_iter = CONTROLLER_DEFAULT_LOST_BALL_ITER;
    max_control_speed_mm_s = CONTROLLER_DEFAULT_MAX_CONTROL_SPEED_MM_S;
    controller_period_ms = CONTROLLER_DEFAULT_PERIOD_MS;
    reset_controller();
}
