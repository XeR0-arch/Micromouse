/**
 * @file    encoders.h
 * @brief   Encoder reading + differential-drive odometry
 *
 * Hardware mapping (from .ioc):
 *   Left encoder:  TIM5 (PA0=CH1, PA1=CH2)
 *   Right encoder: TIM2 (PA15=CH1, PB3=CH2)
 *
 * Physical constants — tune for your hardware:
 */
#ifndef ENCODERS_H
#define ENCODERS_H

#include "stm32f4xx_hal.h"
#include "pid.h"
#include <math.h>

/* Wheel geometry — from existing motor_control.c */
#define WHEEL_DIAMETER_MM       39.0f
#define WHEEL_CIRCUMFERENCE_MM  (3.14159265f * WHEEL_DIAMETER_MM)
#define DISTANCE_BETWEEN_WHEELS 90.0f  /* mm, wheelbase */
#define ENCODER_CPR             1440.0f /* counts per wheel revolution */

#define DEG_TO_RAD  0.0174532925f
#define RAD_TO_DEG  57.295779513f

#ifndef M_PI
#define M_PI 3.14159265359f
#endif

/* Forward declaration of Mouse_t — full definition in mouse.h */
struct Mouse_tag;

int32_t Encoders_GetValue(Motor_t *motor);
void    Encoders_UpdatePosition(struct Mouse_tag *mouse, Motor_t *left, Motor_t *right);

#endif /* ENCODERS_H */
