#ifndef __MPU6050_H
#define __MPU6050_H

#include "main.h"

// MPU6050 Control Structure
typedef struct {
    int16_t raw_ax;
    int16_t raw_ay;
    int16_t raw_az;
    float ax;           // Acceleration in g's
    float ay;           // Acceleration in g's
    
    int16_t raw_gz;
    float gz_vel;
    float yaw_angle;    // Accumulated yaw angle in degrees
    int16_t cal_gz;     // Calibration offset
    uint8_t is_calibrated;
} MPU6050_t;

// Externally accessible struct
extern MPU6050_t mpu;

// Initialization and Configuration
uint8_t MPU6050_Init(void);

// Calculate calibration offset (Blocks for ~1-2 seconds)
void MPU6050_Calibrate(void);

// Read raw data and update yaw angle. 
// Should be called continuously at a fixed dt (e.g. 1ms TIM4 interrupt)
void MPU6050_Update(float dt);

#endif /* __MPU6050_H */
