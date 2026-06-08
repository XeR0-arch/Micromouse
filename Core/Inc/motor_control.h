#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "main.h"

// PID Constants (Exposed for tuning interface if needed)
extern float Kp_right;
extern float Ki_right;
extern float Kd_right;

extern float Kp_left;
extern float Ki_left;
extern float Kd_left;

extern float Kp_balancer;
extern float Kp_wall; // Added for wall centering tuning

// Initialize Encoders and PWMs
void Motor_Control_Init(void);

// Immediately stop the motors
void Motor_Control_Stop(void);

// Actively lock the motors in place to prevent coasting
void Motor_Active_Brake(void);

// Set motor PWM directly (for manual modes)
void Motor_SetPWM_Right(int32_t pwm);
void Motor_SetPWM_Left(int32_t pwm);

// Movement commands (non-blocking setup)
// Move a distance in cm
void Motor_Move_Cm(float distance_cm);

// Move a precise number of encoder ticks (for small stepper-like movements)
void Motor_Move_Ticks(int32_t target_ticks);

// Turn a specific angle in degrees
void Motor_Turn_Degrees(float angle);

// Returns 1 if the current move/turn is complete, 0 otherwise
uint8_t Motor_IsMovementComplete(void);

// The Core PID Loop: Must be called inside the TIM4 1kHz Interrupt
void Motor_Control_Update(void);

#endif /* __MOTOR_CONTROL_H */