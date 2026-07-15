#ifndef MPU6050_H
#define MPU6050_H
#include "stm32f4xx_hal.h"
typedef struct
{
    int16_t raw_gz;
    int16_t cal_gz;  // residual software trim after hardware offsets
    float gz_vel;    // deg/s
    float yaw_angle; // integrated yaw, degrees
    uint8_t is_calibrated;
} MPU6050_t;
extern volatile MPU6050_t mpu;
uint8_t MPU6050_Init(void);
void MPU6050_SetHardwareOffsets(int16_t ax_off, int16_t ay_off, int16_t az_off,
                                int16_t gx_off, int16_t gy_off, int16_t gz_off);
void MPU6050_Calibrate(void);
void MPU6050_Update(float dt);
/* ISR-safe schedule + foreground service (I2C never in ISR) */
void MPU6050_ScheduleUpdate(void);
void MPU6050_Service(void);

float MPU6050_ShortestAngleError(float target_deg, float current_deg);
#endif // MPU6050_H
