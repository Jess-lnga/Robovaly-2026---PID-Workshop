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


static int d1 = 0;
static int d2 = 0;

// ------ BALL POSITION - SPEED - TIMESTAMPS ------
static int ball_position_raw_mm = 0;
static int ball_position_mm = 0;
static float ball_position_estimate_mm = 0.0f;
static float ball_speed_estimate_mm_s = 0.0f;

static uint32_t last_ball_position_update_ms = 0;
static uint32_t latest_tof_input_update_ms = 0;
static bool ball_position_prev_valid = false;

static int ball_speed_mm_per_s = 0;

// ------ WATCHDOG AND TIMEOUTS ------
static const uint32_t TOF_TIMEOUT_MS = 120;
static const int FOV_BLEND_MARGIN_MM = 30;
static const float FOV_EDGE_MIN_WEIGHT = 0.10f;
static const uint32_t SPEED_ESTIMATOR_RESET_DT_MS = 250;
static float alpha_beta_min_alpha = ALPHA_BETA_DEFAULT_MIN_ALPHA;
static float alpha_beta_max_alpha = ALPHA_BETA_DEFAULT_MAX_ALPHA;
static float alpha_beta_min_beta = ALPHA_BETA_DEFAULT_MIN_BETA;
static float alpha_beta_max_beta = ALPHA_BETA_DEFAULT_MAX_BETA;

static int noise_profile_positions_mm[NOISE_PROFILE_POINT_COUNT] = {0, 72, 145, 218, 290};
static int position_noise_deadband_mm[NOISE_PROFILE_POINT_COUNT] = {
    DEFAULT_POSITION_NOISE_DEADBAND_MM,
    DEFAULT_POSITION_NOISE_DEADBAND_MM,
    DEFAULT_POSITION_NOISE_DEADBAND_MM,
    DEFAULT_POSITION_NOISE_DEADBAND_MM,
    DEFAULT_POSITION_NOISE_DEADBAND_MM
};
static int tof1_position_noise_deadband_mm[NOISE_PROFILE_POINT_COUNT] = {
    DEFAULT_POSITION_NOISE_DEADBAND_MM,
    DEFAULT_POSITION_NOISE_DEADBAND_MM,
    DEFAULT_POSITION_NOISE_DEADBAND_MM,
    DEFAULT_POSITION_NOISE_DEADBAND_MM,
    DEFAULT_POSITION_NOISE_DEADBAND_MM
};
static int tof2_position_noise_deadband_mm[NOISE_PROFILE_POINT_COUNT] = {
    DEFAULT_POSITION_NOISE_DEADBAND_MM,
    DEFAULT_POSITION_NOISE_DEADBAND_MM,
    DEFAULT_POSITION_NOISE_DEADBAND_MM,
    DEFAULT_POSITION_NOISE_DEADBAND_MM,
    DEFAULT_POSITION_NOISE_DEADBAND_MM
};
static int speed_noise_deadband_mm_s[NOISE_PROFILE_POINT_COUNT] = {
    DEFAULT_SPEED_NOISE_DEADBAND_MM_S,
    DEFAULT_SPEED_NOISE_DEADBAND_MM_S,
    DEFAULT_SPEED_NOISE_DEADBAND_MM_S,
    DEFAULT_SPEED_NOISE_DEADBAND_MM_S,
    DEFAULT_SPEED_NOISE_DEADBAND_MM_S
};

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

static float clamp_float_local(float value, float min_value, float max_value) {
    if(value < min_value) return min_value;
    if(value > max_value) return max_value;
    return value;
}

static void reset_d1_filter_buffer() {
    d1 = 0;
}

static void reset_d2_filter_buffer() {
    d2 = 0;
}

static void reset_speed_estimator() {
    ball_speed_mm_per_s = 0;
    ball_speed_estimate_mm_s = 0.0f;
    speed_valid = false;
}

static float noise_weight_for_position(int position_mm) {
    int noise_mm = max(1, get_position_noise_deadband_mm(position_mm));
    return 1.0f / ((float)noise_mm * (float)noise_mm);
}

static float tof_noise_weight_for_position(int tof_number, int position_mm) {
    int noise_mm = max(1, get_tof_position_noise_deadband_mm(tof_number, position_mm));
    return 1.0f / ((float)noise_mm * (float)noise_mm);
}

static int interpolate_noise_value(const int *values, int position_mm) {
    int clamped_position = clamp_int_local(position_mm, 0, table_length_mm);

    if(clamped_position <= noise_profile_positions_mm[0]){
        return values[0];
    }

    for(int i = 1; i < NOISE_PROFILE_POINT_COUNT; i++){
        int x0 = noise_profile_positions_mm[i - 1];
        int x1 = noise_profile_positions_mm[i];

        if(clamped_position <= x1){
            if(x1 <= x0) return values[i];

            float t = (float)(clamped_position - x0) / (float)(x1 - x0);
            return (int)lroundf((float)values[i - 1] +
                                t * (float)(values[i] - values[i - 1]));
        }
    }

    return values[NOISE_PROFILE_POINT_COUNT - 1];
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
    bool processed_new_input = false;
    uint32_t newest_input_ms = latest_tof_input_update_ms;

    if(tof_1_update_ms != 0 && tof_1_update_ms != last_processed_d1_ms){
        last_processed_d1_ms = tof_1_update_ms;
        processed_new_input = true;
        newest_input_ms = max(newest_input_ms, tof_1_update_ms);

        if(staff_d1 >= 0){
            d1 = staff_d1;
            last_valid_d1_ms = tof_1_update_ms;
            d1_valid = true;
        } else {
            reset_d1_filter_buffer();
            d1_valid = false;
        }
    }

    if(tof_2_update_ms != 0 && tof_2_update_ms != last_processed_d2_ms){
        last_processed_d2_ms = tof_2_update_ms;
        processed_new_input = true;
        newest_input_ms = max(newest_input_ms, tof_2_update_ms);

        if(staff_d2 >= 0){
            d2 = staff_d2;
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
        processed_new_input = true;
        newest_input_ms = now;
    }

    if (d2_valid && now - last_valid_d2_ms > TOF_TIMEOUT_MS) {
        reset_d2_filter_buffer();
        d2_valid = false;
        processed_new_input = true;
        newest_input_ms = now;
    }

    if(processed_new_input){
        latest_tof_input_update_ms = newest_input_ms;
    }

}

bool compute_ball_speed(){
    uint32_t sample_ms = latest_tof_input_update_ms;

    if(sample_ms == 0){
        speed_valid = false;
        return false;
    }

    if(sample_ms == last_ball_position_update_ms){
        return speed_valid;
    }

    if(ball_position_raw_mm < 0){
        ball_position_prev_valid = false;
        speed_valid = false;
        ball_speed_mm_per_s = 0;
        ball_speed_estimate_mm_s = 0.0f;
        ball_position_mm = -1;
        last_ball_position_update_ms = sample_ms;
        return false;
    }

    if(!ball_position_prev_valid){
        ball_position_estimate_mm = (float)ball_position_raw_mm;
        ball_speed_estimate_mm_s = 0.0f;
        ball_position_mm = ball_position_raw_mm;
        last_ball_position_update_ms = sample_ms;
        ball_position_prev_valid = true;
        speed_valid = false;
        ball_speed_mm_per_s = 0;
        return false;
    }

    uint32_t dt_ms = sample_ms - last_ball_position_update_ms;

    if(dt_ms == 0){
        return speed_valid;
    }

    if(dt_ms > SPEED_ESTIMATOR_RESET_DT_MS){
        ball_position_estimate_mm = (float)ball_position_raw_mm;
        ball_speed_estimate_mm_s = 0.0f;
        ball_position_mm = ball_position_raw_mm;
        last_ball_position_update_ms = sample_ms;
        reset_speed_estimator();
        ball_position_prev_valid = true;
        return false;
    }

    float dt_s = (float)dt_ms * 0.001f;
    float measured_position_mm = (float)ball_position_raw_mm;
    int local_position_noise_mm = max(1, get_position_noise_deadband_mm(ball_position_raw_mm));

    float noise_scale = (float)local_position_noise_mm / (float)DEFAULT_POSITION_NOISE_DEADBAND_MM;
    noise_scale = clamp_float_local(noise_scale, 1.0f, 4.0f);

    float alpha = clamp_float_local(alpha_beta_max_alpha / noise_scale,
                                    alpha_beta_min_alpha,
                                    alpha_beta_max_alpha);
    float beta = clamp_float_local(alpha_beta_max_beta / noise_scale,
                                   alpha_beta_min_beta,
                                   alpha_beta_max_beta);

    float predicted_position_mm = ball_position_estimate_mm + ball_speed_estimate_mm_s * dt_s;
    float innovation_mm = measured_position_mm - predicted_position_mm;

    float innovation_scale = fabsf(innovation_mm) / (float)(local_position_noise_mm * 8);
    if(innovation_scale > 1.0f){
        float trust_scale = 1.0f / sqrtf(innovation_scale);
        alpha *= clamp_float_local(trust_scale, 0.55f, 1.0f);
        beta *= clamp_float_local(trust_scale, 0.55f, 1.0f);
    }

    ball_position_estimate_mm = predicted_position_mm + alpha * innovation_mm;
    ball_position_estimate_mm = clamp_float_local(ball_position_estimate_mm, 0.0f, (float)table_length_mm);
    ball_speed_estimate_mm_s += beta * innovation_mm / dt_s;

    ball_position_mm = clamp_int_local((int)lroundf(ball_position_estimate_mm), 0, table_length_mm);
    ball_speed_mm_per_s = (int)lroundf(ball_speed_estimate_mm_s);

    last_ball_position_update_ms = sample_ms;
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
            tof1_weight *= tof_noise_weight_for_position(TOF1, x_from_tof1);
        }
    } else {
        d1_corr = -1;
    }

    if(d2_valid){
        d2_corr = linearise_tof_measure(TOF2, d2);
        if(d2_corr >= 0){
            x_from_tof2 = clamp_int_local(d2_corr, 0, table_length_mm);
            tof2_weight = tof_fov_weight(d2_corr, REAL_DISTANCE_AT_FOV2);
            tof2_weight *= tof_noise_weight_for_position(TOF2, x_from_tof2);
        }
    } else {
        d2_corr = -1;
    }

    if((d1_corr >= 0) && (d2_corr >= 0) &&
       (d1_corr < MIN_ACCEPTABLE_TOF_VALUE_MM) &&
       (d2_corr < MIN_ACCEPTABLE_TOF_VALUE_MM)){
        ball_position_raw_mm = -1;
    } else {
        float total_weight = tof1_weight + tof2_weight;

        if(total_weight > 0.0f){
            float fused_position = ((tof1_weight * (float)x_from_tof1) +
                                    (tof2_weight * (float)x_from_tof2)) /
                                   total_weight;
            ball_position_raw_mm = clamp_int_local((int)lroundf(fused_position), 0, table_length_mm);
            position_valid = true;
        } else {
            ball_position_raw_mm = -1;
        }
    }

    compute_ball_speed();
    return position_valid && ball_position_mm >= 0;
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

int get_ball_position_from_tof(int tof_number){
    if(tof_number == TOF1){
        if(!d1_valid) return -1;

        int d1_corr = linearise_tof_measure(TOF1, d1);
        if(d1_corr < 0) return -1;

        return clamp_int_local(table_length_mm - d1_corr, 0, table_length_mm);
    }

    if(tof_number == TOF2){
        if(!d2_valid) return -1;

        int d2_corr = linearise_tof_measure(TOF2, d2);
        if(d2_corr < 0) return -1;

        return clamp_int_local(d2_corr, 0, table_length_mm);
    }

    return -1;
}

int get_tof_fov_position_mm(int tof_number){
    if(tof_number == TOF1){
        return clamp_int_local(table_length_mm - REAL_DISTANCE_AT_FOV1, 0, table_length_mm);
    }

    if(tof_number == TOF2){
        return clamp_int_local(REAL_DISTANCE_AT_FOV2, 0, table_length_mm);
    }

    return -1;
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

bool set_noise_rejection_profile(const int *positions_mm,
                                 const int *position_noise_mm,
                                 const int *speed_noise_mm_s,
                                 int count){
    if(positions_mm == nullptr || position_noise_mm == nullptr ||
       speed_noise_mm_s == nullptr || count != NOISE_PROFILE_POINT_COUNT){
        return false;
    }

    for(int i = 0; i < count; i++){
        if(positions_mm[i] < 0 || positions_mm[i] > table_length_mm ||
           position_noise_mm[i] < 0 || position_noise_mm[i] > 100 ||
           speed_noise_mm_s[i] < 0 || speed_noise_mm_s[i] > 1000){
            return false;
        }

        if(i > 0 && positions_mm[i] <= positions_mm[i - 1]){
            return false;
        }
    }

    for(int i = 0; i < count; i++){
        noise_profile_positions_mm[i] = positions_mm[i];
        position_noise_deadband_mm[i] = position_noise_mm[i];
        speed_noise_deadband_mm_s[i] = speed_noise_mm_s[i];
    }

    reset_speed_estimator();
    ball_position_prev_valid = false;
    return true;
}

void get_noise_rejection_profile(int *positions_mm,
                                 int *position_noise_mm,
                                 int *speed_noise_mm_s,
                                 int count){
    if(count > NOISE_PROFILE_POINT_COUNT){
        count = NOISE_PROFILE_POINT_COUNT;
    }

    for(int i = 0; i < count; i++){
        if(positions_mm != nullptr) positions_mm[i] = noise_profile_positions_mm[i];
        if(position_noise_mm != nullptr) position_noise_mm[i] = position_noise_deadband_mm[i];
        if(speed_noise_mm_s != nullptr) speed_noise_mm_s[i] = speed_noise_deadband_mm_s[i];
    }
}

int get_position_noise_deadband_mm(int position_mm){
    return interpolate_noise_value(position_noise_deadband_mm, position_mm);
}

int get_speed_noise_deadband_mm_s(int position_mm){
    return interpolate_noise_value(speed_noise_deadband_mm_s, position_mm);
}

bool set_tof_position_noise_profile(int tof_number,
                                    const int *position_noise_mm,
                                    int count){
    if(position_noise_mm == nullptr || count != NOISE_PROFILE_POINT_COUNT){
        return false;
    }

    for(int i = 0; i < count; i++){
        if(position_noise_mm[i] < 0 || position_noise_mm[i] > 100){
            return false;
        }
    }

    int *target = nullptr;
    if(tof_number == TOF1){
        target = tof1_position_noise_deadband_mm;
    } else if(tof_number == TOF2){
        target = tof2_position_noise_deadband_mm;
    } else {
        return false;
    }

    for(int i = 0; i < count; i++){
        target[i] = position_noise_mm[i];
    }

    ball_position_prev_valid = false;
    reset_speed_estimator();
    return true;
}

int get_tof_position_noise_deadband_mm(int tof_number, int position_mm){
    if(tof_number == TOF1){
        return interpolate_noise_value(tof1_position_noise_deadband_mm, position_mm);
    }

    if(tof_number == TOF2){
        return interpolate_noise_value(tof2_position_noise_deadband_mm, position_mm);
    }

    return get_position_noise_deadband_mm(position_mm);
}

bool set_alpha_beta_parameters(float min_alpha, float max_alpha,
                               float min_beta, float max_beta){
    if(min_alpha < 0.0f || min_alpha > 1.0f ||
       max_alpha < 0.0f || max_alpha > 1.0f ||
       min_beta < 0.0f || min_beta > 2.0f ||
       max_beta < 0.0f || max_beta > 2.0f ||
       min_alpha > max_alpha || min_beta > max_beta){
        return false;
    }

    alpha_beta_min_alpha = min_alpha;
    alpha_beta_max_alpha = max_alpha;
    alpha_beta_min_beta = min_beta;
    alpha_beta_max_beta = max_beta;
    reset_speed_estimator();
    ball_position_prev_valid = false;
    return true;
}

void get_alpha_beta_parameters(float *min_alpha, float *max_alpha,
                               float *min_beta, float *max_beta){
    if(min_alpha != nullptr) *min_alpha = alpha_beta_min_alpha;
    if(max_alpha != nullptr) *max_alpha = alpha_beta_max_alpha;
    if(min_beta != nullptr) *min_beta = alpha_beta_min_beta;
    if(max_beta != nullptr) *max_beta = alpha_beta_max_beta;
}

bool set_table_length_mm(int new_table_length_mm){
    if(new_table_length_mm <= 0 || new_table_length_mm > 300){
        return false;
    }

    table_length_mm = new_table_length_mm;
    ball_position_prev_valid = false;
    reset_speed_estimator();
    return true;
}

int get_table_length_mm(){
    return table_length_mm;
}

void reset_ball_position_advanced_parameters(){
    table_length_mm = TABLE_LENGTH_DEFAULT_MM;
    set_alpha_beta_parameters(ALPHA_BETA_DEFAULT_MIN_ALPHA,
                              ALPHA_BETA_DEFAULT_MAX_ALPHA,
                              ALPHA_BETA_DEFAULT_MIN_BETA,
                              ALPHA_BETA_DEFAULT_MAX_BETA);
    reset_d1_filter_buffer();
    reset_d2_filter_buffer();
    reset_speed_estimator();
    ball_position_prev_valid = false;
    int positions[NOISE_PROFILE_POINT_COUNT] = {0, 72, 145, 218, TABLE_LENGTH_DEFAULT_MM};
    int position_noise[NOISE_PROFILE_POINT_COUNT] = {
        DEFAULT_POSITION_NOISE_DEADBAND_MM,
        DEFAULT_POSITION_NOISE_DEADBAND_MM,
        DEFAULT_POSITION_NOISE_DEADBAND_MM,
        DEFAULT_POSITION_NOISE_DEADBAND_MM,
        DEFAULT_POSITION_NOISE_DEADBAND_MM
    };
    int speed_noise[NOISE_PROFILE_POINT_COUNT] = {
        DEFAULT_SPEED_NOISE_DEADBAND_MM_S,
        DEFAULT_SPEED_NOISE_DEADBAND_MM_S,
        DEFAULT_SPEED_NOISE_DEADBAND_MM_S,
        DEFAULT_SPEED_NOISE_DEADBAND_MM_S,
        DEFAULT_SPEED_NOISE_DEADBAND_MM_S
    };
    set_noise_rejection_profile(positions, position_noise, speed_noise, NOISE_PROFILE_POINT_COUNT);
    set_tof_position_noise_profile(TOF1, position_noise, NOISE_PROFILE_POINT_COUNT);
    set_tof_position_noise_profile(TOF2, position_noise, NOISE_PROFILE_POINT_COUNT);
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




