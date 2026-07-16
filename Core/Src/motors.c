/**
 * @file    motors.c
 * @brief   Motor driver — init encoders/PWM, set speed
 *
 * Uses HAL to start encoder timers and PWM channels.
 * Motors_SetSpeed() converts a normalised [-1.0 … +1.0] value into
 * the correct PWM channel assignment for direction + duty cycle.
 */
#include "motors.h"
#include "tim.h"
#include <math.h>

void Motors_Init(void)
{
    /* Start encoder timers */
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);  /* Left encoder  */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);  /* Right encoder */

    /* Ensure motors are stopped BEFORE starting PWM to prevent glitches */
    Motors_Stop();

    /* Start right motor PWM: TIM1 CH1 (PA8) + CH4 (PA11) */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    /* Start left motor PWM: TIM3 CH3 (PB0) + CH4 (PB1) */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    /* Start control loop timer interrupt */
    HAL_TIM_Base_Start_IT(&htim4);
}

/**
 * @brief  Set motor speed from normalised value.
 * @param  motor  Pointer to motor struct (for side identification)
 * @param  speed  Normalised speed [-1.0 … +1.0]
 *                Positive = forward, Negative = reverse
 */
void Motors_SetSpeed(Motor_t *motor, float speed)
{
    uint16_t pwm = (uint16_t)(fabsf(speed) * (float)MOTOR_MAX_PWM);
    if (pwm > MOTOR_MAX_PWM) pwm = MOTOR_MAX_PWM;

    switch (motor->side)
    {
        case MOTOR_RIGHT:
            if (speed > 0.01f)
            {
                /* Forward: CH1=0, CH4=PWM */
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm);
            }
            else if (speed < -0.01f)
            {
                /* Reverse: CH1=PWM, CH4=0 */
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
            }
            else
            {
                /* Stop */
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
            }
            break;

        case MOTOR_LEFT:
            if (speed > 0.01f)
            {
                /* Forward: CH3=0, CH4=PWM */
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm);
            }
            else if (speed < -0.01f)
            {
                /* Reverse: CH3=PWM, CH4=0 */
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
            }
            else
            {
                /* Stop */
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
            }
            break;
    }
}

void Motors_Stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
}
