#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H
#include "main.h"
// PID constants (exposed for live tuning if needed)
extern float Kp_right;
extern float Ki_right;
extern float Kd_right;
extern float Kp_left;
extern float Ki_left;
extern float Kd_left;
extern float Kp_balancer;
extern float Kp_heading;
// Target position variables (can be nudged by IR wall correction)
extern int32_t target_position_right;
extern int32_t target_position_left;
// Initialize encoders + PWMs + PID timer
void Motor_Control_Init(void);
// Immediately stop motors and clear motion state
void Motor_Control_Stop(void);
// Direct PWM (manual / alignment modes)
void Motor_SetPWM_Right(int32_t pwm);
void Motor_SetPWM_Left(int32_t pwm);
// Non-blocking motion setup (PID runs in TIM4 ISR)
void Motor_Move_Cm(float distance_cm);
void Motor_Turn_Degrees(float angle);
// Gyro-aware helpers
void Motor_Move_Cm_Gyro(float distance_cm, float target_yaw);
void Motor_Turn_To_Heading(float target_heading); // blocking
void Motor_Turn_Degrees_MPU(float angle_deg);     // blocking
// Status
uint8_t Motor_IsMovementComplete(void);
uint8_t Motor_IsTurning(void);
// Core PID loop — call from TIM4 1 kHz interrupt callback
void Motor_Control_Update(void);
// Heading (sign-corrected to match Motor_Turn_Degrees convention)
float Motor_Get_Heading(void);
// Calibration helper (blocking)
void Motor_Drive_Square(float side_cm);
#endif /* __MOTOR_CONTROL_H */