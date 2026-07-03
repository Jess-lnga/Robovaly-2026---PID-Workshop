/*
Kit Robovaly - Prototype PID - 2026
Author: Jérôme ESSOLA ELANGA - Microtechnique EPFL
mail  : jerome.essolaelanga@epfl.ch
github: jess-lnga
*/

#ifndef TOF_H
#define TOF_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool launch_tof_mes(void);

int16_t get_mes_tof_1(void);
int16_t get_mes_tof_2(void);

bool tof_1_is_initialized(void);
bool tof_2_is_initialized(void);

uint32_t get_tof_1_last_update_ms(void);
uint32_t get_tof_2_last_update_ms(void);

// Print for debug
void tof_display(void);
void init_tof(bool debug);

#ifdef __cplusplus
}
#endif

#endif
