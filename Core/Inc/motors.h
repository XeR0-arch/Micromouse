/**
 * @file    motors.h
 * @brief   Motor driver — PWM + direction control
 *
 * Hardware mapping (from .ioc):
 *   Right motor: TIM1_CH1 (PA8) = m1, TIM1_CH4 (PA11) = m2
 *   Left motor:  TIM3_CH3 (PB0) = m2, TIM3_CH4 (PB1) = m1
 *
 * Direction convention (from existing motor_control.c):
 *   Forward (positive speed):
 *     Right: CH1=0, CH4=PWM    Left: CH3=0, CH4=PWM
 *   Reverse (negative speed):
 *     Right: CH1=PWM, CH4=0    Left: CH3=PWM, CH4=0
 */
#ifndef MOTORS_H
#define MOTORS_H

#include "stm32f4xx_hal.h"
#include "pid.h"

/* Max PWM value = TIM ARR for TIM1/TIM3 */
#define MOTOR_MAX_PWM  4999

void Motors_Init(void);
void Motors_SetSpeed(Motor_t *motor, float speed);
void Motors_Stop(void);

#endif /* MOTORS_H */
