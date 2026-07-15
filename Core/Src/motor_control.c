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
#define ENCODER_COUNTS_PER_MOTOR_REV 1440
#define GEAR_RATIO 1.0f
#define WHEEL_CIRCUMFERENCE_CM (3.14159f * WHEEL_DIAMETER_CM)
#define COUNTS_PER_WHEEL_REV (ENCODER_COUNTS_PER_MOTOR_REV / GEAR_RATIO)
#define CM_PER_COUNT (WHEEL_CIRCUMFERENCE_CM / COUNTS_PER_WHEEL_REV)
#define COUNTS_PER_CM (COUNTS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_CM)
#define DEADBAND 1600
#define MAX_PWM 4999
#define POSITION_TOLERANCE 2

// ========== MPU HEADING LOOP CONFIG ==========
#define HEADING_TOLERANCE_DEG 3.0f
#define MAX_TURN_CORRECTIONS 5
#define HEADING_SETTLE_MS 100

// FIXED: +1 for standard right-hand rule (CCW = positive yaw)
#define MPU_YAW_SIGN (1.0f)

// FIXED: Kd reduced from 650 → 0.5. Start here. If it feels sluggish, go to 1.0.
// If it oscillates, go back to 0.0 (P-only).
float Kp_right = 2.0f;
float Ki_right = 0.0f;
float Kd_right = 0.5f;
float Kp_left = 2.0f;
float Ki_left = 0.0f;
float Kd_left = 0.5f;
float Kp_heading = 0.0f;
float Kp_balancer = 0.0f;

// PID state
int32_t target_position_right = 0;
static int32_t start_position_right = 0;
static float integral_right = 0.0f;
static int32_t prev_error_right = 0;
int32_t target_position_left = 0;
static int32_t start_position_left = 0;
static float integral_left = 0.0f;
static int32_t prev_error_left = 0;
static uint8_t movement_complete = 1;
// Absolute heading the robot should hold/seek (degrees)
static float target_heading_deg = 0.0f;
// 1 while encoder turn is running — suppresses straight-line heading hold
static uint8_t is_turning = 0;
// ---------- helpers ----------
static int32_t abs_int(int32_t num)
{
    return (num < 0) ? -num : num;
}
static float normalize_angle_deg(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}
static float clampf(float v, float lo, float hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}
static int32_t apply_deadband(int32_t pwm)
{
    if (pwm > 0 && pwm < DEADBAND)
        return DEADBAND;
    if (pwm < 0 && pwm > -DEADBAND)
        return -DEADBAND;
    return pwm;
}
int32_t get_encoder_right(void)
{
    return (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
}
int32_t get_encoder_left(void)
{
    return (int32_t)__HAL_TIM_GET_COUNTER(&htim5);
}
static void reset_pid(void)
{
    integral_right = 0.0f;
    prev_error_right = 0;
    integral_left = 0.0f;
    prev_error_left = 0;
}
/* Blocking helpers must still pump MPU6050_Service() (I2C not from ISR). */
static void Motor_ServiceDelay(uint32_t ms)
{
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < ms)
    {
        MPU6050_Service();
        HAL_Delay(1);
    }
}
// ---------- public API ----------
void Motor_Control_Init(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
    // Right motor PWM — TIM1 CH1 (PA8), CH4 (PA11)
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    // Left motor PWM — TIM3 CH3 (PB0), CH4 (PB1)
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    // PID interrupt timer
    HAL_TIM_Base_Start_IT(&htim4);
    target_heading_deg = 0.0f;
    is_turning = 0;
    movement_complete = 1;
}
void Motor_SetPWM_Right(int32_t pwm)
{
    if (pwm > MAX_PWM)
        pwm = MAX_PWM;
    if (pwm < -MAX_PWM)
        pwm = -MAX_PWM;
    if (pwm > 0)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm);
    }
    else if (pwm < 0)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -pwm);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    }
}
void Motor_SetPWM_Left(int32_t pwm)
{
    if (pwm > MAX_PWM)
        pwm = MAX_PWM;
    if (pwm < -MAX_PWM)
        pwm = -MAX_PWM;
    if (pwm > 0)
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm);
    }
    else if (pwm < 0)
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, -pwm);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    }
}
void Motor_Control_Stop(void)
{
    Motor_SetPWM_Right(0);
    Motor_SetPWM_Left(0);
    target_position_right = 0;
    target_position_left = 0;
    start_position_right = get_encoder_right();
    start_position_left = get_encoder_left();
    movement_complete = 1;
    is_turning = 0;
}
void Motor_Move_Cm(float distance_cm)
{
    int32_t target_counts = (int32_t)(distance_cm * COUNTS_PER_CM);
    reset_pid();
    start_position_right = get_encoder_right();
    start_position_left = get_encoder_left();
    target_position_right = target_counts;
    target_position_left = target_counts;
    is_turning = 0;
    movement_complete = 0;
}
void Motor_Turn_Degrees(float angle)
{
    if (WHEELBASE_CM == 0.0f)
        return;
    float arc_length_cm = (WHEELBASE_CM * 3.14159f * angle) / 360.0f;
    int32_t target_counts = (int32_t)(arc_length_cm * COUNTS_PER_CM);
    reset_pid();
    start_position_right = get_encoder_right();
    start_position_left = get_encoder_left();
    // +angle: left forward, right reverse → CCW/left when +encoder = forward.
    // If +angle turns the wrong physical way, swap these two lines.
    target_position_right = -target_counts;
    target_position_left = target_counts;
    is_turning = 1;
    movement_complete = 0;
}
uint8_t Motor_IsMovementComplete(void)
{
    return movement_complete;
}
uint8_t Motor_IsTurning(void)
{
    return is_turning;
}
// ========== PID (TIM4 interrupt) ==========
void Motor_Control_Update(void)
{
    if (movement_complete)
        return;

    int32_t current_pos_right = get_encoder_right();
    int32_t current_pos_left  = get_encoder_left();
    int32_t current_distance_right = current_pos_right - start_position_right;
    int32_t current_distance_left  = current_pos_left  - start_position_left;
    int32_t error_right = target_position_right - current_distance_right;
    int32_t error_left  = target_position_left  - current_distance_left;

    if (abs_int(error_left) <= POSITION_TOLERANCE && abs_int(error_right) <= POSITION_TOLERANCE)
    {
        Motor_SetPWM_Right(0);
        Motor_SetPWM_Left(0);
        movement_complete = 1;
        return;
    }

    /* === DECELERATION RAMP ===
     * Scale down PID output as we approach target.
     * Prevents full-speed slam into the end.
     */
    const int32_t DECEL_ZONE_COUNTS = 150;  // ~1.3 cm of deceleration
    float decel_scale = 1.0f;

    int32_t min_err = (abs_int(error_right) < abs_int(error_left)) ?
                       abs_int(error_right) : abs_int(error_left);

    if (min_err < DECEL_ZONE_COUNTS) {
        decel_scale = (float)min_err / DECEL_ZONE_COUNTS;
        if (decel_scale < 0.25f) decel_scale = 0.25f;  // Never go below 25%
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

    int32_t pwm_right = (int32_t)((P_right + I_right + D_right) * decel_scale);
    int32_t pwm_left  = (int32_t)((P_left  + I_left  + D_left)  * decel_scale);

    // Sync & heading hold
    int16_t encoder_difference = (int16_t)(error_right - error_left);
    int32_t sync_correction = (int32_t)(Kp_balancer * encoder_difference);
    int32_t heading_correction = 0;

    if (!is_turning && Kp_heading != 0.0f) {
        float heading_error_deg = normalize_angle_deg(target_heading_deg - Motor_Get_Heading());
        heading_correction = (int32_t)(Kp_heading * heading_error_deg);
    }

    int32_t correction = sync_correction + heading_correction;
    pwm_right -= correction;
    pwm_left  += correction;

    Motor_SetPWM_Right(apply_deadband(pwm_right));
    Motor_SetPWM_Left(apply_deadband(pwm_left));
}
float Motor_Get_Heading(void)
{
    return MPU_YAW_SIGN * mpu.yaw_angle;
}
/**
 * Closed-loop turn:
 *   1) encoder turn for `angle_deg`
 *   2) read MPU residual
 *   3) small correction turns until within HEADING_TOLERANCE_DEG
 *
 * Safety: residual clamped to ±MAX_CORRECTION_DEG, plus a first-attempt
 * sign-inversion guard so a wrong MPU_YAW_SIGN cannot produce ~270°.
 * Blocking — call from main, never from an ISR.
 */
/**
 * Pure MPU turn — no encoder PID involved.
 * Uses 3-speed bang-bang with slowdown zones for clean 90° corners.
 * Blocking — call from main, never ISR.
 */
void Motor_Turn_Degrees_MPU(float angle_deg)
{
    float target = Motor_Get_Heading() + angle_deg;
    float error;
    uint32_t timeout = HAL_GetTick() + 3000;

    /* Tunables — start conservative, raise if too sluggish */
    const float max_speed    = 1800.0f;   // Hard cap (lower = gentler)
    const float min_speed    = 600.0f;    // Minimum to break static friction
    const float creep_speed  = 400.0f;    // Final approach (very slow)
    const float Kp_turn      = 30.0f;     // PWM per degree of error
    const float slowdown_deg = 45.0f;     // Start decelerating at 25° left

    do {
        MPU6050_Service();
        error = MPU6050_ShortestAngleError(target, Motor_Get_Heading());

        /* === Proportional speed ===
         * Far from target  → fast
         * Near target      → slow (automatic deceleration)
         */
        float speed = fabsf(error) * Kp_turn;

        /* === Smooth deceleration curve ===
         * When error < 25°, blend linearly from min_speed down to creep_speed.
         * This prevents the "slamming brakes" feel.
         */
        if (fabsf(error) < slowdown_deg) {
            float ratio = fabsf(error) / slowdown_deg;   // 0.0 at target, 1.0 at 25°
            speed = creep_speed + (min_speed - creep_speed) * ratio;
        }

        /* Clamp */
        if (speed > max_speed) speed = max_speed;
        if (speed < creep_speed && fabsf(error) > HEADING_TOLERANCE_DEG)
            speed = creep_speed;

        /* Apply */
        if (error > 0) {          // CCW / left turn
            Motor_SetPWM_Right(-(int32_t)speed);
            Motor_SetPWM_Left( (int32_t)speed);
        } else {                  // CW / right turn
            Motor_SetPWM_Right( (int32_t)speed);
            Motor_SetPWM_Left(-(int32_t)speed);
        }

        HAL_Delay(5);

    } while (fabsf(error) > HEADING_TOLERANCE_DEG && HAL_GetTick() < timeout);

    /* === Gentle stop ===
     * Cut power and let momentum coast out naturally.
     * For a top-heavy bot, this prevents the "bending" from hard braking.
     */
    Motor_SetPWM_Right(0);
    Motor_SetPWM_Left(0);

    uint32_t coast_start = HAL_GetTick();
    while (HAL_GetTick() - coast_start < 100) {
        MPU6050_Service();
        HAL_Delay(5);
    }

    /* Optional micro-correction if we drifted during coast */
    error = MPU6050_ShortestAngleError(target, Motor_Get_Heading());
    if (fabsf(error) > HEADING_TOLERANCE_DEG) {
        float fix_speed = (fabsf(error) > 5.0f) ? creep_speed : min_speed;
        if (error > 0) {
            Motor_SetPWM_Right(-(int32_t)fix_speed);
            Motor_SetPWM_Left( (int32_t)fix_speed);
        } else {
            Motor_SetPWM_Right( (int32_t)fix_speed);
            Motor_SetPWM_Left(-(int32_t)fix_speed);
        }
        while (fabsf(error) > HEADING_TOLERANCE_DEG && HAL_GetTick() < timeout) {
            MPU6050_Service();
            error = MPU6050_ShortestAngleError(target, Motor_Get_Heading());
            HAL_Delay(10);
        }
        Motor_SetPWM_Right(0);
        Motor_SetPWM_Left(0);
    }

    target_heading_deg = normalize_angle_deg(target);
    is_turning = 0;
}
void Motor_Turn_To_Heading(float target_heading)
{
    float current = Motor_Get_Heading();
    float delta = MPU6050_ShortestAngleError(target_heading, current);
    target_heading_deg = current;
    Motor_Turn_Degrees_MPU(delta);
    target_heading_deg = normalize_angle_deg(target_heading);
}
void Motor_Move_Cm_Gyro(float distance_cm, float target_yaw)
{
    target_heading_deg = normalize_angle_deg(target_yaw);
    Motor_Move_Cm(distance_cm);
}
void Motor_Drive_Square(float side_cm)
{
    for (int side = 0; side < 4; side++)
    {
        Motor_Move_Cm(side_cm);
        while (!Motor_IsMovementComplete())
        {
            Motor_ServiceDelay(5);
        }
        Motor_ServiceDelay(200);
        Motor_Turn_Degrees_MPU(90.0f);
        Motor_ServiceDelay(200);
    }
}
