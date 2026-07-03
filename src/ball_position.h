/*
Kit Robovaly - Prototype PID - 2026
Author: Jérôme ESSOLA ELANGA - Microtechnique EPFL
mail  : jerome.essolaelanga@epfl.ch
github: jess-lnga
*/

#ifndef BALL_POS_H
#define BALL_POS_H
void update_tof_distances();
bool compute_ball_position();
bool compute_ball_speed();

int get_d1();
int get_d2();
int get_ball_position();
int get_ball_speed();
bool is_ball_speed_valid();


#endif // BALL_POS_H
