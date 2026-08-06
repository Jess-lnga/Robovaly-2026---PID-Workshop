#include <Arduino.h>
#include "servo_cmd.h"

static const uint8_t SERVO_PWM_CHANNEL = 0;
static const uint32_t SERVO_PWM_FREQ_HZ = 50;
static const uint8_t SERVO_PWM_RESOLUTION_BITS = 16;
static const uint32_t SERVO_PWM_PERIOD_US = 1000000UL / SERVO_PWM_FREQ_HZ;

static uint8_t servo_pin_used = 255;
static bool servo_initialized = false;

static int servo_limit_min_angle_deg = SERVO_CMD_DEFAULT_THEORETICAL_MIN_DEG;
static int servo_limit_max_angle_deg = SERVO_CMD_DEFAULT_THEORETICAL_MAX_DEG;
static int servo_theoretical_min_angle_deg = SERVO_CMD_DEFAULT_THEORETICAL_MIN_DEG;
static int servo_theoretical_max_angle_deg = SERVO_CMD_DEFAULT_THEORETICAL_MAX_DEG;
static int servo_neutral_offset_us = 0;
static uint16_t servo_min_pulse_us = SERVO_CMD_MIN_PULSE_US;
static uint16_t servo_max_pulse_us = SERVO_CMD_MAX_PULSE_US;

static int servo_angle_deg = SERVO_CMD_NEUTRAL_DEG;
static uint16_t servo_current_pulse_us = SERVO_CMD_NEUTRAL_PULSE_US;

static int clamp_int(int value, int min_value, int max_value) {
    if(value < min_value) return min_value;
    if(value > max_value) return max_value;
    return value;
}

static uint32_t pulse_us_to_duty(uint16_t pulse_us) {
    const uint32_t max_duty = (1UL << SERVO_PWM_RESOLUTION_BITS) - 1UL;
    return ((uint32_t)pulse_us * max_duty) / SERVO_PWM_PERIOD_US;
}

static uint16_t angle_to_pulse_us(int angle_deg) {
    if(servo_theoretical_max_angle_deg == servo_theoretical_min_angle_deg) {
        return SERVO_CMD_NEUTRAL_PULSE_US;
    }

    angle_deg = clamp_int(angle_deg, servo_limit_min_angle_deg, servo_limit_max_angle_deg);

    int angle_span = servo_theoretical_max_angle_deg - servo_theoretical_min_angle_deg;
    uint16_t pulse_span = servo_max_pulse_us - servo_min_pulse_us;

    int ideal_pulse_us = servo_min_pulse_us +
                         ((angle_deg - servo_theoretical_min_angle_deg) * pulse_span) / angle_span;
    int corrected_pulse_us = ideal_pulse_us + servo_neutral_offset_us;

    return (uint16_t)clamp_int(corrected_pulse_us, servo_min_pulse_us, servo_max_pulse_us);
}

bool init_servo_cmd(void) {
    return init_servo_cmd_on_pin(SERVO_CMD_DEFAULT_PIN);
}

bool init_servo_cmd_on_pin(uint8_t servo_pin) {
    servo_pin_used = servo_pin;

    ledcSetup(SERVO_PWM_CHANNEL, SERVO_PWM_FREQ_HZ, SERVO_PWM_RESOLUTION_BITS);
    ledcAttachPin(servo_pin_used, SERVO_PWM_CHANNEL);

    servo_initialized = true;
    servo_angle_deg = get_servo_neutral_angle_deg();

    return set_servo_angle(servo_angle_deg);
}

bool set_servo_angle(int angle_deg) {
    if(!servo_initialized) {
        return false;
    }

    servo_angle_deg = clamp_int(angle_deg, servo_limit_min_angle_deg, servo_limit_max_angle_deg);
    servo_current_pulse_us = angle_to_pulse_us(servo_angle_deg);
    ledcWrite(SERVO_PWM_CHANNEL, pulse_us_to_duty(servo_current_pulse_us));

    return true;
}

bool set_servo_angle_limits(int min_angle_deg, int max_angle_deg) {
    int neutral_angle_deg = get_servo_neutral_angle_deg();

    if(min_angle_deg < servo_theoretical_min_angle_deg ||
       max_angle_deg > servo_theoretical_max_angle_deg ||
       min_angle_deg >= max_angle_deg ||
       neutral_angle_deg < min_angle_deg ||
       neutral_angle_deg > max_angle_deg) {
        return false;
    }

    servo_limit_min_angle_deg = min_angle_deg;
    servo_limit_max_angle_deg = max_angle_deg;

    if(servo_initialized) {
        set_servo_angle(servo_angle_deg);
    }

    return true;
}

bool set_servo_theoretical_angle_range(int min_angle_deg, int max_angle_deg) {
    if(min_angle_deg < SERVO_CMD_MIN_DEG || max_angle_deg > SERVO_CMD_MAX_DEG) {
        return false;
    }

    if(min_angle_deg >= max_angle_deg) {
        return false;
    }

    servo_theoretical_min_angle_deg = min_angle_deg;
    servo_theoretical_max_angle_deg = max_angle_deg;
    servo_limit_min_angle_deg = max(servo_limit_min_angle_deg, servo_theoretical_min_angle_deg);
    servo_limit_max_angle_deg = min(servo_limit_max_angle_deg, servo_theoretical_max_angle_deg);
    if(servo_limit_min_angle_deg >= servo_limit_max_angle_deg ||
       get_servo_neutral_angle_deg() < servo_limit_min_angle_deg ||
       get_servo_neutral_angle_deg() > servo_limit_max_angle_deg) {
        servo_limit_min_angle_deg = servo_theoretical_min_angle_deg;
        servo_limit_max_angle_deg = servo_theoretical_max_angle_deg;
    }

    if(servo_initialized) {
        set_servo_angle(servo_angle_deg);
    }

    return true;
}

bool set_servo_angle_range(int min_angle_deg, int max_angle_deg, int neutral_angle_deg) {
    (void)neutral_angle_deg;
    return set_servo_angle_limits(min_angle_deg, max_angle_deg);
}

bool set_servo_neutral_angle_deg(int neutral_angle_deg) {
    if(neutral_angle_deg != get_servo_neutral_angle_deg()) {
        return false;
    }

    return true;
}

void set_servo_pulse_limits_us(uint16_t min_pulse_us, uint16_t max_pulse_us) {
    if(min_pulse_us >= max_pulse_us) {
        return;
    }

    if(max_pulse_us > SERVO_PWM_PERIOD_US) {
        return;
    }

    servo_min_pulse_us = min_pulse_us;
    servo_max_pulse_us = max_pulse_us;

    if(servo_initialized) {
        set_servo_angle(servo_angle_deg);
    }
}

bool set_servo_neutral_offset_us(int offset_us) {
    if(offset_us < -500 || offset_us > 500) {
        return false;
    }

    servo_neutral_offset_us = offset_us;

    if(servo_initialized) {
        set_servo_angle(servo_angle_deg);
    }

    return true;
}

int get_servo_neutral_offset_us(void) {
    return servo_neutral_offset_us;
}

bool set_servo_calibration_pulse_us(uint16_t pulse_us) {
    if(!servo_initialized ||
       pulse_us < servo_min_pulse_us ||
       pulse_us > servo_max_pulse_us) {
        return false;
    }

    servo_current_pulse_us = pulse_us;
    ledcWrite(SERVO_PWM_CHANNEL, pulse_us_to_duty(servo_current_pulse_us));
    return true;
}

uint16_t get_servo_current_pulse_us(void) {
    return servo_current_pulse_us;
}

bool servo_cmd_is_initialized(void) {
    return servo_initialized;
}

int get_servo_angle(void) {
    return servo_angle_deg;
}

int get_servo_min_angle_deg(void) {
    return servo_limit_min_angle_deg;
}

int get_servo_max_angle_deg(void) {
    return servo_limit_max_angle_deg;
}

int get_servo_neutral_angle_deg(void) {
    return (servo_theoretical_min_angle_deg + servo_theoretical_max_angle_deg + 1) / 2;
}

int get_servo_theoretical_min_angle_deg(void) {
    return servo_theoretical_min_angle_deg;
}

int get_servo_theoretical_max_angle_deg(void) {
    return servo_theoretical_max_angle_deg;
}

void reset_servo_advanced_parameters(void) {
    servo_limit_min_angle_deg = servo_theoretical_min_angle_deg;
    servo_limit_max_angle_deg = servo_theoretical_max_angle_deg;

    if(servo_initialized) {
        set_servo_angle(servo_angle_deg);
    }
}
