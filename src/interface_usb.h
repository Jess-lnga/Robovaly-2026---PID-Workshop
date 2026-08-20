/*
Kit Robovaly - Prototype PID - 2026
Author: Jerome ESSOLA ELANGA - Microtechnique EPFL
mail  : jerome.essolaelanga@epfl.ch
github: jess-lnga
*/

#ifndef INTERFACE_USB_H
#define INTERFACE_USB_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool launch_interface_usb(void);
bool usb_interface_client_connected(void);
bool usb_interface_has_priority(void);

#ifdef __cplusplus
}
#endif

#endif // INTERFACE_USB_H
