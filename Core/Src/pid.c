/**
 * @file    pid.c
 * @brief   Motor velocity PID controller
 *
 * Classic PID on RPM error with integral wind-up clamping.
 * Output is normalised to [-1.0 … +1.0] for the motor driver.
 */
#include "pid.h"
#include "motors.h"

Motor_t motorLeft;
Motor_t motorRight;

void PID_Init(Motor_t *motor, MotorSide_t side, float kp, float ki, float kd)
{
    motor->side      = side;
    motor->pidEnable = false;
    motor->kp        = kp;
    motor->ki        = ki;
    motor->kd        = kd;

    motor->set_rpm          = 0.0f;
    motor->act_rpm          = 0.0f;
    motor->act_rpm_filtered = 0.0f;
    motor->prev_rpm         = 0.0f;
    motor->e                = 0.0f;
    motor->e_prev           = 0.0f;
    motor->e_total          = 0.0f;
    motor->out              = 0.0f;
    motor->enc              = 0;
    motor->encPrev          = 0;
    motor->encDiff          = 0;
    motor->totalDist        = 0.0f;
    motor->dist             = 0.0f;
}

void PID_Controller(Motor_t *motor)
{
    motor->e_prev = motor->e;
    motor->e      = motor->set_rpm - motor->act_rpm_filtered;
    motor->e_total += motor->e;

    /* Integral clamping */
    if (motor->e_total > 1750.0f)
        motor->e_total = 1750.0f;
    else if (motor->e_total < -1750.0f)
        motor->e_total = -1750.0f;

    motor->out = motor->kp * motor->e
               + motor->ki * motor->e_total * PID_TIME_STEP
               + motor->kd * (motor->e - motor->e_prev) / PID_TIME_STEP;

    /* Clamp output to normalised range */
    if (motor->out > 1.0f)
        motor->out = 1.0f;
    else if (motor->out < -1.0f)
        motor->out = -1.0f;

    Motors_SetSpeed(motor, motor->out);
}

void PID_Enable(Motor_t *motor)
{
    motor->pidEnable = true;
}

void PID_Disable(Motor_t *motor)
{
    motor->pidEnable = false;
}

bool PID_IsEnabled(Motor_t *motor)
{
    return motor->pidEnable;
}
