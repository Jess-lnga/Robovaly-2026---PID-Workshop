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

int get_d1();
int get_d2();
int get_ball_position();


#endif // BALL_POS_H