/*
Kit Robovaly - Prototype PID - 2026
Author: Jérôme ESSOLA ELANGA - Microtechnique EPFL
mail  : jerome.essolaelanga@epfl.ch
github: jess-lnga
*/

#ifndef INTERFACE_WEB_H
#define INTERFACE_WEB_H

#include <stdbool.h>

#ifdef __cplusplus
#include <Arduino.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

bool launch_interface_web(void);
bool load_startup_persistent_settings(void);
bool wifi_interface_client_connected(void);
bool wifi_interface_has_priority(void);
bool set_wifi_ap_ssid(const char *ssid);
const char *get_wifi_ap_ssid(void);
bool save_persistent_controller_settings(void);
bool load_persistent_controller_settings(void);
bool save_persistent_advanced_settings(void);
bool load_persistent_advanced_settings(void);
void reset_persistent_advanced_settings(void);
int get_manual_angle_step_deg(void);
bool set_manual_angle_step_deg(int step_deg);
int get_plot_max_seconds_value(void);
bool set_plot_max_seconds_value(int seconds);
int get_servo_pwm_step_us(void);
bool set_servo_pwm_step_us(int step_us);

#ifdef __cplusplus
}

String usb_calibration_state_json(void);
String usb_calibration_start_json(const char *mode, int target);
String usb_calibration_action_json(const char *action, int value, bool has_value);
String usb_servo_calibration_state_json(void);
String usb_servo_calibration_start_json(bool initial_mode);
String usb_servo_calibration_action_json(const char *action,
                                         int value,
                                         bool has_value,
                                         int min_angle,
                                         int max_angle,
                                         int limit_min,
                                         int limit_max,
                                         int offset_us,
                                         int step_us);
#endif

#endif // INTERFACE_WEB_H
