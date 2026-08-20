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

#ifdef __cplusplus
}
#endif

#endif // INTERFACE_WEB_H
