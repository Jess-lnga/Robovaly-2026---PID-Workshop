#include <Arduino.h>
#include "ball_position.h"
#include "tof.h"


// ------ GEOMETRICAL PARAMETERS ------
static const int BALL_RADIUS_MM  = 20;
static const int TABLE_LENGTH_MM = 290;
static const int TABLE_INCERT_MM = 5;  



// ------ TOF CALLIBRATION ------
static const int MAX_VALUE_TOF_MM = 170;
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

static int ball_position_mm = 0;

// ------ WATCHDOG AND TIMEOUTS ------
static const uint32_t TOF_TIMEOUT_MS = 300;

static uint32_t last_valid_d1_ms = 0;
static uint32_t last_valid_d2_ms = 0;

static bool d1_valid = false;
static bool d2_valid = false;

//////////////////////////////////////

void update_tof_distances(){
    int staff_d1  = get_mes_tof_1();
    int staff_d2  = get_mes_tof_2();

    if(staff_d1 >= 0){
        mv_avg_d1[index_mv_avg1] = staff_d1;
        index_mv_avg1 = (index_mv_avg1 + 1) % MV_AVG_LENGTH;
        if (count_d1 < MV_AVG_LENGTH) count_d1++;

        last_valid_d1_ms = millis();
        d1_valid = true;
    }

    if(staff_d2 >= 0){
        mv_avg_d2[index_mv_avg2] = staff_d2;
        index_mv_avg2 = (index_mv_avg2 + 1) % MV_AVG_LENGTH;
        if (count_d2 < MV_AVG_LENGTH) count_d2++;

        last_valid_d2_ms = millis();
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

bool compute_ball_position(){
    int d1_corr = 0;
    int d2_corr = 0;

    if(d1_valid && d2_valid){
        d1_corr = d1 + tof1_offset_mm;
        d2_corr = d2 + tof2_offset_mm;

        if(d1_corr > MAX_VALUE_TOF_MM){
            if(d2_corr > MAX_VALUE_TOF_MM){
                ball_position_mm = -1;
                return false;
            }
            else{
                ball_position_mm = d2_corr;
                return true;
            }
        }else{
            if(d2_corr > MAX_VALUE_TOF_MM){
                ball_position_mm = TABLE_LENGTH_MM - d1_corr;
                return true;
            }
            else{
                ball_position_mm = (TABLE_LENGTH_MM - d1_corr + d2_corr) / 2;
                return true;
            }
        }
    }else if(d1_valid){

        d1_corr = d1 + tof1_offset_mm;

        if(d1_corr > MAX_VALUE_TOF_MM){
            ball_position_mm = -1;
            return false;
        }

        ball_position_mm = TABLE_LENGTH_MM - d1_corr;
        return true;
    }else if(d2_valid){

        d2_corr = d2 + tof2_offset_mm;

        if(d2_corr > MAX_VALUE_TOF_MM){
            ball_position_mm = -1;
            return false;
        }

        ball_position_mm = d2_corr;
        return true;
    }

    ball_position_mm = -1;
    return false;
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




