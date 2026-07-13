#include "motor_control.h"
#include "ir_distance.h"
#include "tim.h"
#include <math.h>
#include <stdbool.h>

// ========== HARDWARE MAPPING ==========
// Right Encoder: TIM2
// Left Encoder:  TIM5
// Right Motor:   TIM1 CH1 & CH4 (PA8 & PA11)
// Left Motor:    TIM3 CH3 & CH4 (PB0 & PB1)

// ========== CONFIGURATION ==========
#define WHEELBASE_CM 7.8f  
#define WHEEL_DIAMETER_CM 3.16f
#define ENCODER_COUNTS_PER_MOTOR_REV 1440
#define GEAR_RATIO 1.527f

#define WHEEL_CIRCUMFERENCE_CM (3.14159f * WHEEL_DIAMETER_CM)  
#define COUNTS_PER_WHEEL_REV (ENCODER_COUNTS_PER_MOTOR_REV / GEAR_RATIO)  
#define CM_PER_COUNT (WHEEL_CIRCUMFERENCE_CM / COUNTS_PER_WHEEL_REV)  
#define COUNTS_PER_CM (COUNTS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_CM)  

#define DEADBAND 1000
#define MAX_PWM  4999   

#define POSITION_TOLERANCE 2  

// PID Constants
float Kp_right = 10.0f;
float Ki_right = 0.0f;   
float Kd_right = 650.0f;
float Kp_left = 10.0f;
float Ki_left = 0.0f;
float Kd_left = 650.0f;

// PID Variables
int32_t target_position_right = 0;  
static int32_t start_position_right = 0;   
static float integral_right = 0.0f;
static int32_t prev_error_right = 0;

int32_t target_position_left = 0;  
static int32_t start_position_left = 0;   
static float integral_left = 0.0f;
static int32_t prev_error_left = 0;

static uint8_t movement_complete = 1;

static int32_t abs_int(int32_t num) {
    return (num < 0) ? -num : num;
}

static int32_t apply_deadband(int32_t pwm) {
    if (pwm > 0 && pwm < DEADBAND) return DEADBAND;
    if (pwm < 0 && pwm > -DEADBAND) return -DEADBAND;
    return pwm;
}

int32_t get_encoder_right(void) {
    // Return inverted if needed so forward is always positive
    int32_t count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    return INVERT_RIGHT_MOTOR ? -count : count;
}

int32_t get_encoder_left(void) {
    int32_t count = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);
    return INVERT_LEFT_MOTOR ? -count : count;
}

static void reset_pid(void) {
    integral_right = 0.0f;
    prev_error_right = 0;
    integral_left = 0.0f;
    prev_error_left = 0;
}

void Motor_Control_Init(void) {
    // Start Encoders
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

    // Start Right Motor PWM
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    // Start Left Motor PWM
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    // Start TIM4 for PID Interrupt Loop
    HAL_TIM_Base_Start_IT(&htim4);
}

void Motor_SetPWM_Right(int32_t pwm) {
    // Apply Hardware Polarity Inversion if defined
    if (INVERT_RIGHT_MOTOR) {
        pwm = -pwm;
    }
    
    // Apply Feedforward scale
    pwm = (int32_t)(pwm * MOTOR_RIGHT_SCALE);

    if (pwm > MAX_PWM) pwm = MAX_PWM;
    if (pwm < -MAX_PWM) pwm = -MAX_PWM;

    // Normal forward logic
    if (pwm > 0) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    } else if (pwm < 0) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, -pwm);
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    }
}

void Motor_SetPWM_Left(int32_t pwm) {
    // Apply Hardware Polarity Inversion if defined
    if (INVERT_LEFT_MOTOR) {
        pwm = -pwm;
    }
    
    // Apply Feedforward scale
    pwm = (int32_t)(pwm * MOTOR_LEFT_SCALE);

    if (pwm > MAX_PWM) pwm = MAX_PWM;
    if (pwm < -MAX_PWM) pwm = -MAX_PWM;

    // Normal forward logic
    if (pwm > 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    } else if (pwm < 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, -pwm);
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    }
}

void Motor_Control_Stop(void) {
    Motor_SetPWM_Right(0);
    Motor_SetPWM_Left(0);
    target_position_right = get_encoder_right();
    target_position_left = get_encoder_left();
    start_position_right = target_position_right;
    start_position_left = target_position_left;
    movement_complete = 1;
}

void Motor_Move_Cm(float distance_cm) {
    int32_t target_counts = (int32_t)(distance_cm * COUNTS_PER_CM);
    reset_pid();
    start_position_right = get_encoder_right();
    start_position_left = get_encoder_left();
    target_position_right = start_position_right + target_counts;
    target_position_left = start_position_left + target_counts;
    movement_complete = 0;
}

void Motor_Turn_Degrees(float angle) {
    if (WHEELBASE_CM == 0.0f) return;
    float arc_length_cm = (WHEELBASE_CM * 3.14159f * angle) / 360.0f;
    int32_t target_counts = (int32_t)(arc_length_cm * COUNTS_PER_CM);
    
    reset_pid();
    start_position_right = get_encoder_right();
    start_position_left = get_encoder_left();
    
    // Positive angle = turn right (left wheel forward, right wheel backward)
    if (angle > 0) {
        target_position_left = start_position_left + target_counts;
        target_position_right = start_position_right - target_counts;
    } else {
        target_position_left = start_position_left - target_counts;
        target_position_right = start_position_right + target_counts;
    }
    movement_complete = 0;
}

uint8_t Motor_IsMovementComplete(void) {
    return movement_complete;
}

// Stubs for navigation integration
uint8_t is_turning_gyro = 0;
uint8_t Motor_IsTurning(void) {
    return is_turning_gyro;
}

void Motor_Turn_To_Heading(float target_heading) {
    // Currently a stub: The real integration will use state_estimation.c
    // For now, call the standard turn.
    is_turning_gyro = 1;
    // Calculation would be: float diff = target_heading - current_heading;
    // Motor_Turn_Degrees(diff);
}

void Motor_Move_Cm_Gyro(float distance_cm, float target_yaw) {
    // Stub
    Motor_Move_Cm(distance_cm);
}


// ========== PID CONTROL (Called from TIM4 Interrupt) ==========
void Motor_Control_Update(void) {
    if (movement_complete) return;

    int32_t current_pos_right = get_encoder_right();
    int32_t current_pos_left = get_encoder_left();

    // Absolute error tracking
    int32_t error_right = target_position_right - current_pos_right;
    int32_t error_left = target_position_left - current_pos_left;

    // Check completion condition
    if (abs_int(error_left) <= POSITION_TOLERANCE && abs_int(error_right) <= POSITION_TOLERANCE) {
        Motor_SetPWM_Right(0);
        Motor_SetPWM_Left(0);
        is_turning_gyro = 0;
        movement_complete = 1;
        return;
    }

    // Right PID
    float P_right = Kp_right * error_right;
    integral_right += error_right;
    float I_right = Ki_right * integral_right;
    int32_t derivative_right = error_right - prev_error_right;
    float D_right = Kd_right * derivative_right;
    prev_error_right = error_right;

    // Left PID
    float P_left = Kp_left * error_left;
    integral_left += error_left;
    float I_left = Ki_left * integral_left;
    int32_t derivative_left = error_left - prev_error_left;
    float D_left = Kd_left * derivative_left;
    prev_error_left = error_left;

    int32_t pwm_right = (int32_t)(P_right + I_right + D_right);
    int32_t pwm_left = (int32_t)(P_left + I_left + D_left);

    // The IR balancing is removed from here! 
    // It will be handled by the high-level Kalman Filter adjusting target_position_left/right.

    // Apply Output (with deadband)
    Motor_SetPWM_Right(apply_deadband(pwm_right));
    Motor_SetPWM_Left(apply_deadband(pwm_left));
}
