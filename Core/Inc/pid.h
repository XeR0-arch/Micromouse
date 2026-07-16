/**
 * @file    pid.h
 * @brief   Motor velocity PID controller
 *
 * Each motor is represented by a Motor_t struct that holds encoder state,
 * RPM tracking, and PID controller state. The PID operates on velocity
 * (RPM error), not position — matching the reference architecture.
 */
#ifndef PID_H
#define PID_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

/* Control loop time step in seconds.
 * TIM4: 100 MHz / 100 / 1001 ≈ 999 Hz ≈ 1 ms */
#define PID_TIME_STEP  0.001f

typedef enum {
    MOTOR_LEFT  = 0,
    MOTOR_RIGHT = 1
} MotorSide_t;

typedef struct {

    MotorSide_t side;

    /* RPM tracking */
    float set_rpm;
    float act_rpm;
    float act_rpm_filtered;
    float prev_rpm;

    /* PID state */
    float e;
    float e_prev;
    float e_total;
    float out;          /* normalised output [-1.0 … +1.0] */
    float kp;
    float ki;
    float kd;

    /* Encoder state */
    int32_t enc;
    int32_t encPrev;
    int32_t encDiff;

    /* Distance tracking (mm) */
    float dist;
    float totalDist;

    /* Enable flag */
    bool pidEnable;

} Motor_t;

extern Motor_t motorLeft;
extern Motor_t motorRight;

void PID_Init(Motor_t *motor, MotorSide_t side, float kp, float ki, float kd);
void PID_Controller(Motor_t *motor);
void PID_Enable(Motor_t *motor);
void PID_Disable(Motor_t *motor);
bool PID_IsEnabled(Motor_t *motor);

#endif /* PID_H */
