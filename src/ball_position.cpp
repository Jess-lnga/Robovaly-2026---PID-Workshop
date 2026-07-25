#include <Arduino.h>
#include "ball_position.h"
#include "tof.h"


// ------ GEOMETRICAL PARAMETERS ------
static const int BALL_RADIUS_MM  = 20;
static const int TABLE_INCERT_MM = 5;  
static int table_length_mm = TABLE_LENGTH_DEFAULT_MM;



// ------ TOF CALLIBRATION ------
static const int MAX_VALUE_TOF1_MM = 260;
static const int MAX_VALUE_TOF2_MM = 170;
static const int MIN_VALUE_TOF1_MM = 115; // Not the absolute minimum, but the minimum which shows that we need to only use this tof
static const int MIN_VALUE_TOF2_MM = 120;

static const int MIN_ACCEPTABLE_TOF_VALUE_MM = 80; //If both tofs give values below this threshold, we consider that there is a problem
static int tof1_offset_mm = -15;
static int tof2_offset_mm = 0;

// ------- BETTER TOF CALIBRATION -------
static bool tof1_calibrated = false;
static bool tof2_calibrated = false;

static int MEAS_AT_FOV_TOF1 = 145;
static int MEAS_AT_FOV_TOF2 = 145;

static int REAL_DISTANCE_AT_FOV1 = 145;
static int REAL_DISTANCE_AT_FOV2 = 145;

static int MEAS_AT_000_MM_TOF1 = 0;
static int MEAS_AT_072_MM_TOF1 = INFINITE_TOF_VALUE;
static int MEAS_AT_145_MM_TOF1 = INFINITE_TOF_VALUE;

static int MEAS_AT_000_MM_TOF2 = 0;
static int MEAS_AT_072_MM_TOF2 = INFINITE_TOF_VALUE;
static int MEAS_AT_145_MM_TOF2 = INFINITE_TOF_VALUE;


static bool fov1_greater_than_145_mm = false;
static bool fov1_greater_than_072_mm = false;
static bool fov1_greater_than_000_mm = false;

static bool fov2_greater_than_145_mm = false;
static bool fov2_greater_than_072_mm = false;  
static bool fov2_greater_than_000_mm = false;


// ------ MOVING AVERAGES ------
static int position_filter_window = POSITION_FILTER_DEFAULT_WINDOW;
static int speed_filter_window = SPEED_FILTER_DEFAULT_WINDOW;
static int mv_avg_d1[FILTER_MAX_WINDOW] = {0};
static int mv_avg_d2[FILTER_MAX_WINDOW] = {0};
static int mv_avg_speed[FILTER_MAX_WINDOW] = {0};

static int index_mv_avg1 = 0;
static int index_mv_avg2 = 0;
static int index_mv_avg_speed = 0;

static int d1 = 0;
static int d2 = 0;

static int count_d1 = 0;
static int count_d2 = 0;
static int count_speed = 0;

// ------ BALL POSITION - SPEED - TIMESTAMPS ------
static int ball_position_mm = 0;
static int ball_position_prev_mm = 0;

static uint32_t last_ball_position_update_ms = 0;
static bool ball_position_prev_valid = false;

static int ball_speed_mm_per_s = 0;
static float ball_speed_filtered_mm_per_s = 0.0f;

// ------ WATCHDOG AND TIMEOUTS ------
static const uint32_t TOF_TIMEOUT_MS = 120;
static const int FOV_BLEND_MARGIN_MM = 30;
static const float FOV_EDGE_MIN_WEIGHT = 0.10f;

static uint32_t last_valid_d1_ms = 0;
static uint32_t last_valid_d2_ms = 0;
static uint32_t last_processed_d1_ms = 0;
static uint32_t last_processed_d2_ms = 0;

static bool d1_valid = false;
static bool d2_valid = false;
static bool speed_valid = false;

//////////////////////////////////////

static int clamp_int_local(int value, int min_value, int max_value) {
    if(value < min_value) return min_value;
    if(value > max_value) return max_value;
    return value;
}

static void reset_position_filter_buffers() {
    memset(mv_avg_d1, 0, sizeof(mv_avg_d1));
    memset(mv_avg_d2, 0, sizeof(mv_avg_d2));
    index_mv_avg1 = 0;
    index_mv_avg2 = 0;
    count_d1 = 0;
    count_d2 = 0;
    d1 = 0;
    d2 = 0;
}

static void reset_d1_filter_buffer() {
    memset(mv_avg_d1, 0, sizeof(mv_avg_d1));
    index_mv_avg1 = 0;
    count_d1 = 0;
    d1 = 0;
}

static void reset_d2_filter_buffer() {
    memset(mv_avg_d2, 0, sizeof(mv_avg_d2));
    index_mv_avg2 = 0;
    count_d2 = 0;
    d2 = 0;
}

static void reset_speed_filter_buffer() {
    memset(mv_avg_speed, 0, sizeof(mv_avg_speed));
    index_mv_avg_speed = 0;
    count_speed = 0;
    ball_speed_mm_per_s = 0;
    ball_speed_filtered_mm_per_s = 0.0f;
    speed_valid = false;
}

static const int EPSILON = 1;   
int linearise_measure(int min_meas, int min_val, int max_meas, int max_val, int meas){
    if(abs(min_meas - max_meas) < EPSILON){
        return min_val;
    }

    float slope = (float)(max_val - min_val) / (float)(max_meas - min_meas);
    float intercept = (float)min_val - slope * (float)min_meas;

    return (int)lroundf(slope * (float)meas + intercept);
}

int linearise_tof_measure(int tof_number, int meas){
    if(tof_number == TOF1){
        if (!tof1_calibrated) return -1;

        if(meas > MEAS_AT_FOV_TOF1) {return -1;}

        if(meas > MEAS_AT_145_MM_TOF1) {
            return linearise_measure(MEAS_AT_145_MM_TOF1, 145, MEAS_AT_FOV_TOF1, REAL_DISTANCE_AT_FOV1, meas);
        
        }else if(meas > MEAS_AT_072_MM_TOF1) {
            if(fov1_greater_than_145_mm){
                return linearise_measure(MEAS_AT_072_MM_TOF1, 72, MEAS_AT_145_MM_TOF1, 145, meas);
            }else{
                return linearise_measure(MEAS_AT_072_MM_TOF1, 72, MEAS_AT_FOV_TOF1, REAL_DISTANCE_AT_FOV1, meas);
            }
        
        }else if(meas > MEAS_AT_000_MM_TOF1) {
            if(fov1_greater_than_072_mm){
                return linearise_measure(MEAS_AT_000_MM_TOF1, 0, MEAS_AT_072_MM_TOF1, 72, meas);
            }else{
                return linearise_measure(MEAS_AT_000_MM_TOF1, 0, MEAS_AT_FOV_TOF1, REAL_DISTANCE_AT_FOV1, meas);
            }
        }else if(meas > 0){
            return 0;

        }else {
            return -1;
        }

    } else if(tof_number == TOF2){
        if (!tof2_calibrated) return -1;

        if(meas > MEAS_AT_FOV_TOF2) {return -1;}

        if(meas > MEAS_AT_145_MM_TOF2) {
            return linearise_measure(MEAS_AT_145_MM_TOF2, 145, MEAS_AT_FOV_TOF2, REAL_DISTANCE_AT_FOV2, meas);
        
        }else if(meas > MEAS_AT_072_MM_TOF2) {
            if(fov2_greater_than_145_mm){
                return linearise_measure(MEAS_AT_072_MM_TOF2, 72, MEAS_AT_145_MM_TOF2, 145, meas);
            }else{
                return linearise_measure(MEAS_AT_072_MM_TOF2, 72, MEAS_AT_FOV_TOF2, REAL_DISTANCE_AT_FOV2, meas);
            }
        
        }else if(meas > MEAS_AT_000_MM_TOF2) {
            if(fov2_greater_than_072_mm){
                return linearise_measure(MEAS_AT_000_MM_TOF2, 0, MEAS_AT_072_MM_TOF2, 72, meas);
            }else{
                return linearise_measure(MEAS_AT_000_MM_TOF2, 0, MEAS_AT_FOV_TOF2, REAL_DISTANCE_AT_FOV2, meas);
            }
        }else if(meas > 0){
            return 0;

        }else {
            return -1;
        }
    }
    else{
        return -1;
    }

}

bool set_tof_calibration(int tof_number, int fov, int real_distance_at_fov, 
                    int meas_at_000_mm, int meas_at_072_mm, int meas_at_145_mm){
    bool calibration_valid = false;
    
    if((meas_at_000_mm <= meas_at_072_mm) && (meas_at_072_mm <= meas_at_145_mm)){
        if(tof_number == TOF1){
            tof1_calibrated = true;
            calibration_valid = true;
        }
        else if(tof_number == TOF2){
            tof2_calibrated = true;
            calibration_valid = true;
        }
    }
    else{
        if(tof_number == TOF1){
            tof1_calibrated = false;
        }
        else if(tof_number == TOF2){
            tof2_calibrated = false;
        }
    }

    if(tof_number == TOF1){
        MEAS_AT_FOV_TOF1 = fov;
        REAL_DISTANCE_AT_FOV1 = real_distance_at_fov;
        MEAS_AT_000_MM_TOF1   = meas_at_000_mm;
        MEAS_AT_072_MM_TOF1   = meas_at_072_mm;
        MEAS_AT_145_MM_TOF1   = meas_at_145_mm;

        fov1_greater_than_145_mm = (real_distance_at_fov > 145);
        fov1_greater_than_072_mm = (real_distance_at_fov > 72);
        fov1_greater_than_000_mm = (real_distance_at_fov > 0);

    }
    else if(tof_number == TOF2){
        MEAS_AT_FOV_TOF2 = fov;
        REAL_DISTANCE_AT_FOV2 = real_distance_at_fov;
        MEAS_AT_000_MM_TOF2   = meas_at_000_mm;
        MEAS_AT_072_MM_TOF2   = meas_at_072_mm;
        MEAS_AT_145_MM_TOF2   = meas_at_145_mm;

        fov2_greater_than_145_mm = (real_distance_at_fov > 145);
        fov2_greater_than_072_mm = (real_distance_at_fov > 72);
        fov2_greater_than_000_mm = (real_distance_at_fov > 0);
    }

    return calibration_valid;
}

static float tof_fov_weight(int corrected_distance_mm, int real_distance_at_fov_mm) {
    if(corrected_distance_mm < 0) return 0.0f;
    if(real_distance_at_fov_mm <= 0) return 1.0f;

    int margin_to_fov_mm = real_distance_at_fov_mm - corrected_distance_mm;
    if(margin_to_fov_mm >= FOV_BLEND_MARGIN_MM) return 1.0f;
    if(margin_to_fov_mm <= 0) return FOV_EDGE_MIN_WEIGHT;

    float normalized_margin = (float)margin_to_fov_mm / (float)FOV_BLEND_MARGIN_MM;
    return FOV_EDGE_MIN_WEIGHT + normalized_margin * (1.0f - FOV_EDGE_MIN_WEIGHT);
}



void update_tof_distances(){
    int staff_d1  = get_mes_tof_1();
    int staff_d2  = get_mes_tof_2();
    uint32_t tof_1_update_ms = get_tof_1_last_update_ms();
    uint32_t tof_2_update_ms = get_tof_2_last_update_ms();

    if(tof_1_update_ms != 0 && tof_1_update_ms != last_processed_d1_ms){
        last_processed_d1_ms = tof_1_update_ms;

        if(staff_d1 >= 0){
            mv_avg_d1[index_mv_avg1] = staff_d1;
            index_mv_avg1 = (index_mv_avg1 + 1) % position_filter_window;
            if (count_d1 < position_filter_window) count_d1++;

            last_valid_d1_ms = tof_1_update_ms;
            d1_valid = true;
        } else {
            reset_d1_filter_buffer();
            d1_valid = false;
        }
    }

    if(tof_2_update_ms != 0 && tof_2_update_ms != last_processed_d2_ms){
        last_processed_d2_ms = tof_2_update_ms;

        if(staff_d2 >= 0){
            mv_avg_d2[index_mv_avg2] = staff_d2;
            index_mv_avg2 = (index_mv_avg2 + 1) % position_filter_window;
            if (count_d2 < position_filter_window) count_d2++;

            last_valid_d2_ms = tof_2_update_ms;
            d2_valid = true;
        } else {
            reset_d2_filter_buffer();
            d2_valid = false;
        }
    }

    uint32_t now = millis();

    if (d1_valid && now - last_valid_d1_ms > TOF_TIMEOUT_MS) {
        reset_d1_filter_buffer();
        d1_valid = false;
    }

    if (d2_valid && now - last_valid_d2_ms > TOF_TIMEOUT_MS) {
        reset_d2_filter_buffer();
        d2_valid = false;
    }

    int sum_d1 = 0;
    int sum_d2 = 0;

    
    for (int i = 0; i < count_d1; i++) {
        sum_d1 += mv_avg_d1[i];
    }

    for (int i = 0; i < count_d2; i++) {
        sum_d2 += mv_avg_d2[i];
    }

    if (count_d1 > 0) d1 = sum_d1 / count_d1;
    if (count_d2 > 0) d2 = sum_d2 / count_d2;
}

bool compute_ball_speed(){
    uint32_t now = millis();

    if(ball_position_mm < 0){
        ball_position_prev_valid = false;
        speed_valid = false;
        ball_speed_mm_per_s = 0;
        ball_speed_filtered_mm_per_s = 0.0f;
        return false;
    }

    if(!ball_position_prev_valid){
        ball_position_prev_mm = ball_position_mm;
        last_ball_position_update_ms = now;
        ball_position_prev_valid = true;
        speed_valid = false;
        ball_speed_mm_per_s = 0;
        ball_speed_filtered_mm_per_s = 0.0f;
        return false;
    }

    uint32_t dt_ms = now - last_ball_position_update_ms;

    if(dt_ms == 0){
        speed_valid = false;
        return false;
    }

    int delta_position_mm = ball_position_mm - ball_position_prev_mm;
    int raw_speed_mm_per_s = (delta_position_mm * 1000) / (int)dt_ms;

    mv_avg_speed[index_mv_avg_speed] = raw_speed_mm_per_s;
    index_mv_avg_speed = (index_mv_avg_speed + 1) % speed_filter_window;
    if(count_speed < speed_filter_window) count_speed++;

    int sum_speed = 0;
    for(int i = 0; i < count_speed; i++){
        sum_speed += mv_avg_speed[i];
    }

    ball_speed_mm_per_s = (count_speed > 0) ? (sum_speed / count_speed) : 0;
    ball_speed_filtered_mm_per_s = (float)ball_speed_mm_per_s;

    ball_position_prev_mm = ball_position_mm;
    last_ball_position_update_ms = now;
    speed_valid = true;

    return true;
}

bool compute_ball_position(){
    int d1_corr = 0;
    int d2_corr = 0;
    int x_from_tof1 = 0;
    int x_from_tof2 = 0;
    float tof1_weight = 0.0f;
    float tof2_weight = 0.0f;
    bool position_valid = false;

    if(d1_valid){
        d1_corr = linearise_tof_measure(TOF1, d1);
        if(d1_corr >= 0){
            x_from_tof1 = clamp_int_local(table_length_mm - d1_corr, 0, table_length_mm);
            tof1_weight = tof_fov_weight(d1_corr, REAL_DISTANCE_AT_FOV1);
        }
    } else {
        d1_corr = -1;
    }

    if(d2_valid){
        d2_corr = linearise_tof_measure(TOF2, d2);
        if(d2_corr >= 0){
            x_from_tof2 = clamp_int_local(d2_corr, 0, table_length_mm);
            tof2_weight = tof_fov_weight(d2_corr, REAL_DISTANCE_AT_FOV2);
        }
    } else {
        d2_corr = -1;
    }

    if((d1_corr >= 0) && (d2_corr >= 0) &&
       (d1_corr < MIN_ACCEPTABLE_TOF_VALUE_MM) &&
       (d2_corr < MIN_ACCEPTABLE_TOF_VALUE_MM)){
        ball_position_mm = -1;
    } else {
        float total_weight = tof1_weight + tof2_weight;

        if(total_weight > 0.0f){
            float fused_position = ((tof1_weight * (float)x_from_tof1) +
                                    (tof2_weight * (float)x_from_tof2)) /
                                   total_weight;
            ball_position_mm = clamp_int_local((int)lroundf(fused_position), 0, table_length_mm);
            position_valid = true;
        } else {
            ball_position_mm = -1;
        }
    }

    compute_ball_speed();
    return position_valid;
}

int get_d1(){
    if(d1_valid){
        return d1;
    }
    else{
        return -1;
    }   
}

int get_d2(){
    if(d2_valid){
        return d2;
    }
    else{
        return -1;
    }
}

int get_ball_position(){
    return ball_position_mm;
}

int get_ball_speed(){
    if(speed_valid){
        return ball_speed_mm_per_s;
    }
    else{
        return 0;
    }
}

bool is_ball_speed_valid(){
    return speed_valid;
}

bool set_position_filter_window(int window){
    if(window < FILTER_MIN_WINDOW || window > FILTER_MAX_WINDOW){
        return false;
    }

    if(window != position_filter_window){
        position_filter_window = window;
        reset_position_filter_buffers();
    }

    return true;
}

int get_position_filter_window(){
    return position_filter_window;
}

bool set_speed_filter_window(int window){
    if(window < FILTER_MIN_WINDOW || window > FILTER_MAX_WINDOW){
        return false;
    }

    if(window != speed_filter_window){
        speed_filter_window = window;
        reset_speed_filter_buffer();
        ball_position_prev_valid = false;
    }

    return true;
}

int get_speed_filter_window(){
    return speed_filter_window;
}

bool set_table_length_mm(int new_table_length_mm){
    if(new_table_length_mm <= 0 || new_table_length_mm > 300){
        return false;
    }

    table_length_mm = new_table_length_mm;
    ball_position_prev_valid = false;
    reset_speed_filter_buffer();
    return true;
}

int get_table_length_mm(){
    return table_length_mm;
}

void reset_ball_position_advanced_parameters(){
    table_length_mm = TABLE_LENGTH_DEFAULT_MM;
    position_filter_window = POSITION_FILTER_DEFAULT_WINDOW;
    speed_filter_window = SPEED_FILTER_DEFAULT_WINDOW;
    reset_position_filter_buffers();
    reset_speed_filter_buffer();
    ball_position_prev_valid = false;
}

static size_t previous_line_length = 0;
static int16_t previous_tof1 = INT16_MIN;
static int16_t previous_tof2 = INT16_MIN;
static int16_t previous_ball_position = INT16_MIN;
static int16_t previous_ball_speed = INT16_MIN;
static bool previous_ball_speed_valid = false;

void display_distances(){
  update_tof_distances();
  compute_ball_position();

  int16_t tof1 = get_d1();
  int16_t tof2 = get_d2();
  int16_t ball_position = get_ball_position();
  int16_t ball_speed = get_ball_speed();
  bool ball_speed_valid = is_ball_speed_valid();

  if (tof1 == previous_tof1 && tof2 == previous_tof2 &&
      ball_position == previous_ball_position &&
      ball_speed == previous_ball_speed &&
      ball_speed_valid == previous_ball_speed_valid) {
    return;
  }

  char line[110];

  if (ball_speed_valid) {
    snprintf(line, sizeof(line),
             "Distance 1: %4d mm    Distance 2: %4d mm    X: %4d mm    V: %5d mm/s",
             tof1, tof2, ball_position, ball_speed);
  }
  else {
    snprintf(line, sizeof(line),
             "Distance 1: %4d mm    Distance 2: %4d mm    X: %4d mm    V:    -- mm/s",
             tof1, tof2, ball_position);
  }

  for (size_t i = 0; i < previous_line_length; i++) {
    Serial.print('\b');
  }

  Serial.print(line);
  previous_line_length = strlen(line);
  previous_tof1 = tof1;
  previous_tof2 = tof2;
  previous_ball_position = ball_position;
  previous_ball_speed = ball_speed;
  previous_ball_speed_valid = ball_speed_valid;

}




