#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f4xx_hal.h"  // Adjust to stm32f1xx_hal.h if using F1 series

typedef struct {
    int16_t raw_gz;
    int16_t cal_gz;         // Fine software trim (usually 0 after hardware offsets)
    float gz_vel;           // Angular velocity in deg/s
    float yaw_angle;        // Integrated yaw angle in degrees
    uint8_t is_calibrated;  // Flag indicating ready state
} MPU6050_t;

extern MPU6050_t mpu;

/* Function Prototypes */
uint8_t MPU6050_Init(void);
void MPU6050_SetHardwareOffsets(int16_t ax_off, int16_t ay_off, int16_t az_off,
                                int16_t gx_off, int16_t gy_off, int16_t gz_off);
void MPU6050_Calibrate(void);
void MPU6050_Update(float dt);

#endif // MPU6050_H
