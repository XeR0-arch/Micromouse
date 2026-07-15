#include "motor_control.h"
#include "ir_distance.h"
#include "mpu6050.h"
#include "tim.h"
#include <math.h>
#include "stdbool.h"

// ========== HARDWARE MAPPING ==========
// Right Encoder: TIM2
// Left Encoder:  TIM5
// Right Motor:   TIM1 CH1 & CH4 (PA8 & PA11)
// Left Motor:    TIM3 CH3 & CH4 (PB0 & PB1)

// ========== CONFIGURATION ==========
#define WHEELBASE_CM 7.8f  
#define WHEEL_DIAMETER_CM 3.9f
#define ENCODER_COUNTS_PER_MOTOR_REV 1440//576  // (4 * 144) 1440
#define GEAR_RATIO 1.0f

#define WHEEL_CIRCUMFERENCE_CM (3.14159f * WHEEL_DIAMETER_CM)  
#define COUNTS_PER_WHEEL_REV (ENCODER_COUNTS_PER_MOTOR_REV / GEAR_RATIO)  
#define CM_PER_COUNT (WHEEL_CIRCUMFERENCE_CM / COUNTS_PER_WHEEL_REV)  
#define COUNTS_PER_CM (COUNTS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_CM)  

#define DEADBAND 1600
#define MAX_PWM  4999   

#define POSITION_TOLERANCE 2  

#define TARGET_IR_LEFT  2000 // ADC value when perfectly centered
#define TARGET_IR_RIGHT 2000 // ADC value when perfectly centered

// ========== MPU HEADING LOOP CONFIG ==========
// Outer loop: after an encoder-based turn finishes, we check the MPU's
// integrated yaw against where we actually wanted to end up. If it's off
// (wheel slip, etc.) we issue another (smaller) encoder turn for exactly
// the remaining error, and repeat until within tolerance.
#define HEADING_TOLERANCE_DEG   2.0f   // acceptable final heading error
#define MAX_TURN_CORRECTIONS    5      // safety cap on correction attempts
#define HEADING_SETTLE_MS       80     // let the robot/gyro settle after a stop before reading yaw

// If commanding Motor_Turn_Degrees(+angle) makes mpu.yaw_angle move the
// WRONG way (decrease instead of increase), flip this to -1.0f.
// Check this first on the bench before trusting the square routine.
#define MPU_YAW_SIGN             1.0f

// PID Constants
float Kp_right = 1.5f;
float Ki_right = 0.0f;   
float Kd_right = 650.0f;
float Kp_left = 1.5f;
float Ki_left = 0.0f;
float Kd_left = 650.0f;
float Kp_heading = 0.0f;  // straight-line heading-hold gain (PWM per degree of yaw error) — tune this, start low
float Kp_balancer = 0.0f;

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

// Absolute heading (degrees) the robot should currently be holding/seeking.
// Starts at 0 and accumulates as turns are commanded — this is the "truth"
// heading, tracked against mpu.yaw_angle (which also starts at 0 after
// MPU6050_Calibrate()).
static float target_heading_deg = 0.0f;

// True while an encoder-driven turn is in flight. Heading-hold correction
// in Motor_Control_Update() is suppressed while this is set, so it doesn't
// fight the turn PID.
static uint8_t is_turning = 0;

static int32_t abs_int(int32_t num) {
    return (num < 0) ? -num : num;
}

static int32_t apply_deadband(int32_t pwm) {
    if (pwm > 0 && pwm < DEADBAND) return DEADBAND;
    if (pwm < 0 && pwm > -DEADBAND) return -DEADBAND;
    return pwm;
}

int32_t get_encoder_right(void) {
    return (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
}

int32_t get_encoder_left(void) {
    return (int32_t)__HAL_TIM_GET_COUNTER(&htim5);
}

static void reset_pid(void) {
    integral_right = 0.0f;
    prev_error_right = 0;
    integral_left = 0.0f;
    prev_error_left = 0;
}

/* Blocking motion helpers must continue servicing the foreground MPU
 * scheduler. The actual I2C read is never performed from an ISR. */
static void Motor_ServiceDelay(uint32_t ms) {
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < ms) {
        MPU6050_Service();
        HAL_Delay(1);
    }
}

void Motor_Control_Init(void) {
    // Start Encoders
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

    // Start Right Motor PWM — TIM1 normal channels (CH1 on PA8, CH4 on PA11)
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    // Start Left Motor PWM — TIM3 channels (CH3, CH4 on PB0, PB1)
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    // Start TIM4 for PID Interrupt Loop
    HAL_TIM_Base_Start_IT(&htim4);

    target_heading_deg = 0.0f;
    is_turning = 0;
}

void Motor_SetPWM_Right(int32_t pwm) {
    if (pwm > MAX_PWM) pwm = MAX_PWM;
    if (pwm < -MAX_PWM) pwm = -MAX_PWM;

    if (pwm > 0) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm);
    } else if (pwm < 0) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -pwm);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    }
}

void Motor_SetPWM_Left(int32_t pwm) {
    if (pwm > MAX_PWM) pwm = MAX_PWM;
    if (pwm < -MAX_PWM) pwm = -MAX_PWM;

    if (pwm > 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm);
    } else if (pwm < 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, -pwm);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    }
}

void Motor_Control_Stop(void) {
    Motor_SetPWM_Right(0);
    Motor_SetPWM_Left(0);
    target_position_right = 0;
    target_position_left = 0;
    start_position_right = get_encoder_right();
    start_position_left = get_encoder_left();
    movement_complete = 1;
}

void Motor_Move_Cm(float distance_cm) {
    int32_t target_counts = (int32_t)(distance_cm * COUNTS_PER_CM);
    reset_pid();
    start_position_right = get_encoder_right();
    start_position_left = get_encoder_left();
    target_position_right = target_counts;
    target_position_left = target_counts;
    is_turning = 0;   // straight segment: hold current heading against MPU
    movement_complete = 0;
}

void Motor_Turn_Degrees(float angle) {
    if (WHEELBASE_CM == 0.0f) return;
    float arc_length_cm = (WHEELBASE_CM * 3.14159f * angle) / 360.0f;
    int32_t target_counts = (int32_t)(arc_length_cm * COUNTS_PER_CM);

    reset_pid();
    start_position_right = get_encoder_right();
    start_position_left = get_encoder_left();

	target_position_right = -target_counts;
	target_position_left = target_counts;

	is_turning = 1;
    movement_complete = 0;
}

uint8_t Motor_IsMovementComplete(void) {
    return movement_complete;
}

// ========== PID CONTROL (Called from TIM4 Interrupt) ==========
void Motor_Control_Update(void) {
    if (movement_complete) return;

    int32_t current_pos_right = get_encoder_right();
    int32_t current_pos_left = get_encoder_left();

    int32_t current_distance_right = current_pos_right - start_position_right;
    int32_t error_right = target_position_right - current_distance_right;
    int32_t current_distance_left = current_pos_left - start_position_left;
    int32_t error_left = target_position_left - current_distance_left;

    // Check completion condition
    if (abs_int(error_left) <= POSITION_TOLERANCE && abs_int(error_right) <= POSITION_TOLERANCE) {
        Motor_SetPWM_Right(0);
        Motor_SetPWM_Left(0);
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

    // Wheel-sync balancer (encoder-based, always active — keeps both sides
    // converging together during a move/turn regardless of heading loop)
    int16_t encoder_difference = error_right - error_left;
    int32_t sync_correction = (int32_t)(Kp_balancer * encoder_difference);

    // Heading-hold (MPU-based) — only while driving straight (a square's
    // sides). Suppressed during turns so it doesn't fight the turn PID;
    // turns are corrected separately/iteratively in Motor_Turn_Degrees_MPU().
    int32_t heading_correction = 0;
    if (!is_turning) {
        float heading_error_deg = target_heading_deg - (MPU_YAW_SIGN * mpu.yaw_angle);
        heading_correction = (int32_t)(Kp_heading * heading_error_deg);
    }

    int32_t correction = sync_correction + heading_correction;

    // Apply Symmetrically
//    pwm_right -= correction;
//    pwm_left  += correction;

    // Apply Output
    Motor_SetPWM_Right(apply_deadband(pwm_right));
    Motor_SetPWM_Left(apply_deadband(pwm_left));
}

/**
  * @brief  Current heading estimate (degrees), sign-corrected to match
  *         Motor_Turn_Degrees' convention.
  */
float Motor_Get_Heading(void) {
    return MPU_YAW_SIGN * mpu.yaw_angle;
}

/**
  * @brief  Closed-loop turn: commands an encoder-based turn for `angle_deg`,
  *         then checks the ACTUAL rotation achieved via the MPU. If wheel
  *         slip left it short/long of target, issues another encoder turn
  *         for just the leftover error, and repeats until within
  *         HEADING_TOLERANCE_DEG (or MAX_TURN_CORRECTIONS is hit).
  *
  *         Blocking — call from main loop, not from an ISR.
  */
void Motor_Turn_Degrees_MPU(float angle_deg) {
    target_heading_deg += angle_deg;

    float remaining = angle_deg;

    for (int attempt = 0; attempt < MAX_TURN_CORRECTIONS; attempt++) {
        Motor_Turn_Degrees(remaining);

        while (!Motor_IsMovementComplete()) {
            Motor_ServiceDelay(5);
        }

        Motor_ServiceDelay(HEADING_SETTLE_MS);  // let the robot/gyro settle before reading yaw

        float heading_error = target_heading_deg - Motor_Get_Heading();

        if (fabsf(heading_error) <= HEADING_TOLERANCE_DEG) {
            break;
        }

        remaining = heading_error;  // next turn covers exactly what's left
    }

    is_turning = 0;  // done turning; next Move_Cm will hold this heading
}

/**
  * @brief  Drive one side_cm x side_cm square, correcting each 90-degree
  *         turn against the MPU so slip doesn't accumulate corner-to-corner.
  *         Blocking.
  */
void Motor_Drive_Square(float side_cm) {
    for (int side = 0; side < 4; side++) {
        Motor_Move_Cm(side_cm);
        while (!Motor_IsMovementComplete()) {
            Motor_ServiceDelay(5);
        }
        Motor_ServiceDelay(1000);

        Motor_Turn_Degrees_MPU(90.0f);
        Motor_ServiceDelay(1000);
    }
}
