/**
 * @file    encoders.c
 * @brief   Encoder reading + differential-drive odometry
 *
 * Adapted from reference encoders.c. Core logic:
 * 1. Read encoder counters (TIM5=left, TIM2=right)
 * 2. Compute delta counts since last read
 * 3. Convert to distance (mm) per wheel
 * 4. Update heading angle via differential
 * 5. Update X/Y position
 * 6. Compute filtered RPM for PID
 */
#include "encoders.h"
#include "mouse.h"
#include "tim.h"
#include <math.h>
int32_t Encoders_GetValue(Motor_t *motor)
{
    switch (motor->side)
    {
        case MOTOR_LEFT:
            return (int32_t)__HAL_TIM_GET_COUNTER(&htim5);
        case MOTOR_RIGHT:
            return (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
        default:
            return 0;
    }
}

void Encoders_UpdatePosition(Mouse_t *mouse, Motor_t *left, Motor_t *right)
{
    float distance;
    float actual_angle_temporary;

    /* --- Left encoder --- */
    left->encPrev = left->enc;
    left->enc     = Encoders_GetValue(left);
    left->encDiff = -(left->enc - left->encPrev);
    left->dist    = (float)left->encDiff / ENCODER_CPR * WHEEL_CIRCUMFERENCE_MM;
    left->totalDist += left->dist;

    /* --- Right encoder --- */
    right->encPrev = right->enc;
    right->enc     = Encoders_GetValue(right);
    right->encDiff = -(right->enc - right->encPrev);
    right->dist    = (float)right->encDiff / ENCODER_CPR * WHEEL_CIRCUMFERENCE_MM;
    right->totalDist += right->dist;

    /* --- Heading from differential --- */
    actual_angle_temporary = ((left->totalDist - right->totalDist))
                             / DISTANCE_BETWEEN_WHEELS * RAD_TO_DEG;
    mouse->actual_angle = fmodf(actual_angle_temporary, 360.0f);

    if (mouse->actual_angle < -180.0f)
        mouse->actual_angle += 360.0f;
    else if (mouse->actual_angle > 180.0f)
        mouse->actual_angle -= 360.0f;

    /* --- Position from average distance --- */
    distance = (left->dist + right->dist) / 2.0f;
    mouse->actual_position_x += distance * sinf(mouse->actual_angle * DEG_TO_RAD);
    mouse->actual_position_y += distance * cosf(mouse->actual_angle * DEG_TO_RAD);

    /* --- RPM calculation --- */
    left->act_rpm  = ((float)left->encDiff  / PID_TIME_STEP * 60.0f) / ENCODER_CPR;
    right->act_rpm = ((float)right->encDiff / PID_TIME_STEP * 60.0f) / ENCODER_CPR;

    /* --- Low-pass filter on RPM (Butterworth-inspired 2nd order) --- */
    left->act_rpm_filtered  = 0.854f * left->act_rpm_filtered
                            + 0.0728f * left->act_rpm
                            + 0.0728f * left->prev_rpm;
    left->prev_rpm = left->act_rpm;

    right->act_rpm_filtered = 0.854f * right->act_rpm_filtered
                            + 0.0728f * right->act_rpm
                            + 0.0728f * right->prev_rpm;
    right->prev_rpm = right->act_rpm;
}
