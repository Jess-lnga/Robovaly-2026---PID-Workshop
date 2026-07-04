#include <Arduino.h>
#include "controller.h"
#include "ball_position.h"
#include "servo_cmd.h"

static const int DEFAULT_REFERENCE_MM = 150;
static const uint32_t MIN_CONTROLLER_DT_MS = 5;
static const float INTEGRAL_LIMIT_MM_S = 3000.0f;
static const int SERVO_DEADBAND_DEG = 0;
static const int SERVO_MAX_STEP_DEG = 50;
static const float SERVO_FILTER_ALPHA = 1.0f;

static bool controller_initialized = false;
static bool controller_enabled = true;
static bool last_update_valid = false;

static int reference_mm = DEFAULT_REFERENCE_MM;
static int last_angle_deg = SERVO_CMD_NEUTRAL_DEG;

static float kp = 0.15f;
static float ki = 0.0f;
static float kd = 0.14f;

static float integral_error_mm_s = 0.0f;
static float filtered_angle_deg = SERVO_CMD_NEUTRAL_DEG;
static uint32_t last_update_ms = 0;

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

static int command_to_servo_angle(float pid_command_deg) {
    float angle = (float)SERVO_CMD_NEUTRAL_DEG - pid_command_deg;
    return clamp_int((int)lroundf(angle), SERVO_CMD_MIN_DEG, SERVO_CMD_MAX_DEG);
}

static int smooth_servo_angle(int raw_angle_deg) {
    filtered_angle_deg += SERVO_FILTER_ALPHA * ((float)raw_angle_deg - filtered_angle_deg);

    int filtered_angle_int = clamp_int((int)lroundf(filtered_angle_deg),
                                       SERVO_CMD_MIN_DEG,
                                       SERVO_CMD_MAX_DEG);

    if(abs(filtered_angle_int - last_angle_deg) < SERVO_DEADBAND_DEG) {
        return last_angle_deg;
    }

    return clamp_int(filtered_angle_int,
                     last_angle_deg - SERVO_MAX_STEP_DEG,
                     last_angle_deg + SERVO_MAX_STEP_DEG);
}

bool init_controller(void) {
    if(!servo_cmd_is_initialized()) {
        if(!init_servo_cmd()) {
            controller_initialized = false;
            return false;
        }
    }

    reset_controller();
    controller_initialized = true;
    controller_enabled = true;
    return set_servo_angle(last_angle_deg);
}

void reset_controller(void) {
    integral_error_mm_s = 0.0f;
    last_update_ms = millis();
    last_update_valid = false;
    last_angle_deg = SERVO_CMD_NEUTRAL_DEG;
    filtered_angle_deg = SERVO_CMD_NEUTRAL_DEG;
}

bool update_controller(void) {
    if(!controller_initialized) {
        return false;
    }

    update_tof_distances();
    bool position_valid = compute_ball_position();
    bool speed_valid = is_ball_speed_valid();

    if(!controller_enabled) {
        last_update_valid = position_valid;
        return position_valid;
    }

    uint32_t now = millis();
    uint32_t dt_ms = now - last_update_ms;

    if(!position_valid || dt_ms < MIN_CONTROLLER_DT_MS) {
        last_update_valid = false;
        return false;
    }

    float dt_s = dt_ms * 0.001f;
    last_update_ms = now;

    int position_mm = get_ball_position();
    int speed_mm_s = speed_valid ? get_ball_speed() : 0;

    float error_mm = (float)reference_mm - (float)position_mm;
    integral_error_mm_s += error_mm * dt_s;
    integral_error_mm_s = clamp_float(integral_error_mm_s,
                                      -INTEGRAL_LIMIT_MM_S,
                                      INTEGRAL_LIMIT_MM_S);

    float pid_command_deg = kp * error_mm +
                            ki * integral_error_mm_s -
                            kd * (float)speed_mm_s;

    int raw_angle_deg = command_to_servo_angle(pid_command_deg);
    int angle_deg = smooth_servo_angle(raw_angle_deg);

    if(angle_deg == last_angle_deg) {
        last_update_valid = true;
        return true;
    }

    if(set_servo_angle(angle_deg)) {
        last_angle_deg = angle_deg;
        last_update_valid = true;
        return true;
    }

    last_update_valid = false;
    return false;
}

void set_controller_enabled(bool enabled) {
    controller_enabled = enabled;
    reset_controller();

    if(controller_initialized) {
        set_servo_angle(SERVO_CMD_NEUTRAL_DEG);
    }
}

bool controller_is_enabled(void) {
    return controller_enabled;
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
