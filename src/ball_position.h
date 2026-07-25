/*
Kit Robovaly - Prototype PID - 2026
Author: Jérôme ESSOLA ELANGA - Microtechnique EPFL
mail  : jerome.essolaelanga@epfl.ch
github: jess-lnga
*/

#ifndef BALL_POS_H
#define BALL_POS_H

#define TOF1 1
#define TOF2 2
#define INFINITE_TOF_VALUE 9999

#define POSITION_FILTER_DEFAULT_WINDOW 2
#define SPEED_FILTER_DEFAULT_WINDOW 3
#define FILTER_MIN_WINDOW 1
#define FILTER_MAX_WINDOW 20
#define TABLE_LENGTH_DEFAULT_MM 270

void update_tof_distances();
bool compute_ball_position();
bool compute_ball_speed();

int get_d1();
int get_d2();
int get_ball_position();
int get_ball_speed();
bool is_ball_speed_valid();

void display_distances();

bool set_tof_calibration(int tof_number, int fov, int real_distance_at_fov, 
                    int meas_at_000_mm, int meas_at_072_mm, int meas_at_145_mm);

bool set_position_filter_window(int window);
int get_position_filter_window();

bool set_speed_filter_window(int window);
int get_speed_filter_window();

bool set_table_length_mm(int table_length_mm);
int get_table_length_mm();

void reset_ball_position_advanced_parameters();
    


#endif // BALL_POS_H
