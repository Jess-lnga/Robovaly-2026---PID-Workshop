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

#define TABLE_LENGTH_DEFAULT_MM 290
#define NOISE_PROFILE_POINT_COUNT 5
#define DEFAULT_POSITION_NOISE_DEADBAND_MM 4
#define MIN_SPEED_NOISE_FLOOR_MM_S 30
#define DEFAULT_SPEED_NOISE_DEADBAND_MM_S MIN_SPEED_NOISE_FLOOR_MM_S
#define ALPHA_BETA_DEFAULT_MIN_ALPHA 0.30f
#define ALPHA_BETA_DEFAULT_MAX_ALPHA 0.85f
#define ALPHA_BETA_DEFAULT_MIN_BETA 0.08f
#define ALPHA_BETA_DEFAULT_MAX_BETA 0.60f

void update_tof_distances();
bool compute_ball_position();
bool compute_ball_speed();

int get_d1();
int get_d2();
int get_ball_position();
int get_ball_position_raw();
int get_ball_position_from_tof(int tof_number);
int get_tof_fov_position_mm(int tof_number);
int get_ball_speed();
bool is_ball_speed_valid();

void display_distances();

bool set_tof_calibration(int tof_number, int fov, int real_distance_at_fov, 
                    int meas_at_000_mm, int meas_at_072_mm, int meas_at_145_mm);

bool set_noise_rejection_profile(const int *positions_mm,
                                 const int *position_noise_mm,
                                 const int *speed_noise_mm_s,
                                 int count);
void get_noise_rejection_profile(int *positions_mm,
                                 int *position_noise_mm,
                                 int *speed_noise_mm_s,
                                 int count);
int get_position_noise_deadband_mm(int position_mm);
int get_speed_noise_deadband_mm_s(int position_mm);
bool set_tof_position_noise_profile(int tof_number,
                                    const int *position_noise_mm,
                                    int count);
int get_tof_position_noise_deadband_mm(int tof_number, int position_mm);
bool set_alpha_beta_parameters(float min_alpha, float max_alpha,
                               float min_beta, float max_beta);
void get_alpha_beta_parameters(float *min_alpha, float *max_alpha,
                               float *min_beta, float *max_beta);

bool set_table_length_mm(int table_length_mm);
int get_table_length_mm();

void reset_ball_position_advanced_parameters();
    


#endif // BALL_POS_H
