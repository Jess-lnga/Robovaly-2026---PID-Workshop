#include <Arduino.h>
#include "ball_position.h"
#include "tof.h"


// ------ GEOMETRICAL PARAMETERS ------
static const int BALL_RADIUS_MM  = 20;
static const int TABLE_LENGTH_MM = 290;
static const int TABLE_INCERT_MM = 5;  



// ------ TOF CALLIBRATION ------
static const int MAX_VALUE_TOF1_MM = 170;
static const int MAX_VALUE_TOF2_MM = 170;
static const int MIN_VALUE_TOF1_MM = 115; // Not the absolute minimum, but the minimum which shows that we need to only use this tof
static const int MIN_VALUE_TOF2_MM = 120;

static const int MIN_ACCEPTABLE_TOF_VALUE_MM = 80; //If both tofs give values below this threshold, we consider that there is a problem
static int tof1_offset_mm = 0;
static int tof2_offset_mm = 0;



// ------ MOVING AVERAGES ------
static const int MV_AVG_LENGTH = 5;
static int mv_avg_d1[MV_AVG_LENGTH] = {0};
static int mv_avg_d2[MV_AVG_LENGTH] = {0};

static int index_mv_avg1 = 0;
static int index_mv_avg2 = 0;

static int d1 = 0;
static int d2 = 0;

static int count_d1 = 0;
static int count_d2 = 0;

// ------ BALL POSITION - SPEED - TIMESTAMPS ------
static int ball_position_mm = 0;
static int ball_position_prev_mm = 0;

static uint32_t last_ball_position_update_ms = 0;
static bool ball_position_prev_valid = false;

static int ball_speed_mm_per_s = 0;

// ------ WATCHDOG AND TIMEOUTS ------
static const uint32_t TOF_TIMEOUT_MS = 300;

static uint32_t last_valid_d1_ms = 0;
static uint32_t last_valid_d2_ms = 0;
static uint32_t last_processed_d1_ms = 0;
static uint32_t last_processed_d2_ms = 0;

static bool d1_valid = false;
static bool d2_valid = false;
static bool speed_valid = false;

//////////////////////////////////////

void update_tof_distances(){
    int staff_d1  = get_mes_tof_1();
    int staff_d2  = get_mes_tof_2();
    uint32_t tof_1_update_ms = get_tof_1_last_update_ms();
    uint32_t tof_2_update_ms = get_tof_2_last_update_ms();

    if(staff_d1 >= 0 &&
       tof_1_update_ms != 0 &&
       tof_1_update_ms != last_processed_d1_ms){
        mv_avg_d1[index_mv_avg1] = staff_d1;
        index_mv_avg1 = (index_mv_avg1 + 1) % MV_AVG_LENGTH;
        if (count_d1 < MV_AVG_LENGTH) count_d1++;

        last_processed_d1_ms = tof_1_update_ms;
        last_valid_d1_ms = tof_1_update_ms;
        d1_valid = true;
    }

    if(staff_d2 >= 0 &&
       tof_2_update_ms != 0 &&
       tof_2_update_ms != last_processed_d2_ms){
        mv_avg_d2[index_mv_avg2] = staff_d2;
        index_mv_avg2 = (index_mv_avg2 + 1) % MV_AVG_LENGTH;
        if (count_d2 < MV_AVG_LENGTH) count_d2++;

        last_processed_d2_ms = tof_2_update_ms;
        last_valid_d2_ms = tof_2_update_ms;
        d2_valid = true;
    }

    uint32_t now = millis();

    if (now - last_valid_d1_ms > TOF_TIMEOUT_MS) {
        d1_valid = false;
    }

    if (now - last_valid_d2_ms > TOF_TIMEOUT_MS) {
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
        return false;
    }

    if(!ball_position_prev_valid){
        ball_position_prev_mm = ball_position_mm;
        last_ball_position_update_ms = now;
        ball_position_prev_valid = true;
        speed_valid = false;
        ball_speed_mm_per_s = 0;
        return false;
    }

    uint32_t dt_ms = now - last_ball_position_update_ms;

    if(dt_ms == 0){
        speed_valid = false;
        return false;
    }

    int delta_position_mm = ball_position_mm - ball_position_prev_mm;
    ball_speed_mm_per_s = (delta_position_mm * 1000) / (int)dt_ms;

    ball_position_prev_mm = ball_position_mm;
    last_ball_position_update_ms = now;
    speed_valid = true;

    return true;
}

bool compute_ball_position(){
    int d1_corr = 0;
    int d2_corr = 0;
    bool position_valid = false;

    if(d1_valid && d2_valid){
        d1_corr = d1 + tof1_offset_mm;
        d2_corr = d2 + tof2_offset_mm;

        if(d1_corr > MAX_VALUE_TOF1_MM){
            if(d2_corr > MAX_VALUE_TOF2_MM){
                ball_position_mm = -1;
            }
            else{
                ball_position_mm = d2_corr;
                position_valid = true;
            }
        }else{
            if(d1_corr < MIN_ACCEPTABLE_TOF_VALUE_MM && d2_corr < MIN_ACCEPTABLE_TOF_VALUE_MM){
                ball_position_mm = -1;
            }
            else if((d1_corr < MIN_VALUE_TOF1_MM)||(d2_corr > MAX_VALUE_TOF2_MM)){
                ball_position_mm = TABLE_LENGTH_MM - d1_corr;
                position_valid = true;

            }else{
                ball_position_mm = (TABLE_LENGTH_MM - d1_corr + d2_corr) / 2;
                position_valid = true;
            }
        }
    }else if(d1_valid){

        d1_corr = d1 + tof1_offset_mm;

        if(d1_corr > MAX_VALUE_TOF1_MM){
            ball_position_mm = -1;
        }
        else{
            ball_position_mm = TABLE_LENGTH_MM - d1_corr;
            position_valid = true;
        }
    }else if(d2_valid){

        d2_corr = d2 + tof2_offset_mm;

        if(d2_corr > MAX_VALUE_TOF2_MM){
            ball_position_mm = -1;
        }
        else{
            ball_position_mm = d2_corr;
            position_valid = true;
        }
    }else{
        ball_position_mm = -1;
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




