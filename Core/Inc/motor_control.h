#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "main.h"

// ========== HARDWARE CONFIGURATION ==========
// Set to 1 if the motor drives backward when given a positive PWM.
// Set to 0 if the motor drives forward when given a positive PWM.
#define INVERT_LEFT_MOTOR  0
#define INVERT_RIGHT_MOTOR 0

// Feedforward Scale: Used to balance base motor speeds before PID
// If the bot drifts right, it means the right motor is faster (or left is slower).
// Set the faster motor to < 1.0 (e.g., 0.95) and keep the other at 1.0.
#define MOTOR_LEFT_SCALE  1.0f
#define MOTOR_RIGHT_SCALE 1.0f  // Adjust this in testing if drifting right

// PID Constants (Exposed for tuning interface if needed)
extern float Kp_right;
extern float Ki_right;
extern float Kd_right;

extern float Kp_left;
extern float Ki_left;
extern float Kd_left;

// Target position variables (modified by high-level navigation/Kalman filter)
extern int32_t target_position_right;
extern int32_t target_position_left;

// Initialize Encoders and PWMs
void Motor_Control_Init(void);

// Immediately stop the motors
void Motor_Control_Stop(void);

// Set motor PWM directly (for manual modes)
void Motor_SetPWM_Right(int32_t pwm);
void Motor_SetPWM_Left(int32_t pwm);

// Movement commands (non-blocking setup)
// Move a distance in cm
void Motor_Move_Cm(float distance_cm);

// Move straight a distance in cm while maintaining a target yaw heading using gyro
void Motor_Move_Cm_Gyro(float distance_cm, float target_yaw);

// Turn to an absolute heading in degrees using gyro feedback
void Motor_Turn_To_Heading(float target_heading);

// Turn a specific angle in degrees (legacy encoder-based)
void Motor_Turn_Degrees(float angle);

// Returns 1 if the current move/turn is complete, 0 otherwise
uint8_t Motor_IsMovementComplete(void);

// Returns 1 if the motor state is currently turning using the gyro, 0 otherwise
uint8_t Motor_IsTurning(void);

// The Core PID Loop: Must be called inside the TIM4 1kHz Interrupt
void Motor_Control_Update(void);

#endif /* __MOTOR_CONTROL_H */
