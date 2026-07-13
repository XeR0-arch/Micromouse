#include "mpu6050.h"
#include "i2c.h"
#include "status.h"
#include <math.h>

#define MPU_6050_ADDR         (0x68 << 1)
#define PWR_MGMT_1            0x6B
#define CONFIG_REG            0x1A // Named CONFIG_REG to avoid conflict
#define SMPLRT_DIV            0x19
#define GYRO_CONFIG           0x1B
#define ACCEL_CONFIG          0x1C
#define OUTPUT_START_REGISTER 0x3B

MPU6050_t mpu = {0};

uint8_t MPU6050_Init(void) {
    uint8_t pwr_mgmt_1_config = 0x01;  // set clock source bit 0
    uint8_t config_config = 0x02;      // dlpf value to 2 (98Hz bandwidth, 3ms delay for Gyro)
    uint8_t smprt_div_config = 0x04;   // 200Hz sampling (if dlpf is on)
    uint8_t gyro_config_config = 0x08; // range +-500 deg/s
    uint8_t accel_config_config = 0x00; // range +-2g
    
    // Check if MPU6050 replies
    if (HAL_I2C_IsDeviceReady(&hi2c1, MPU_6050_ADDR, 3, 100) != HAL_OK) {
        return 0; // Failed
    }
    
    HAL_I2C_Mem_Write(&hi2c1, MPU_6050_ADDR, PWR_MGMT_1, 1, &pwr_mgmt_1_config, 1, 100);
    HAL_Delay(10);
    HAL_I2C_Mem_Write(&hi2c1, MPU_6050_ADDR, CONFIG_REG, 1, &config_config, 1, 100);
    HAL_Delay(10);
    HAL_I2C_Mem_Write(&hi2c1, MPU_6050_ADDR, SMPLRT_DIV, 1, &smprt_div_config, 1, 100);
    HAL_Delay(10);
    HAL_I2C_Mem_Write(&hi2c1, MPU_6050_ADDR, GYRO_CONFIG, 1, &gyro_config_config, 1, 100);
    HAL_Delay(10);
    HAL_I2C_Mem_Write(&hi2c1, MPU_6050_ADDR, ACCEL_CONFIG, 1, &accel_config_config, 1, 100);
    HAL_Delay(10);

    return 1; // Success
}

void MPU6050_Calibrate(void) {
    uint8_t mpu_raw_data[14];
    long buff_gz = 0;
    uint16_t num_samples = 1000;

    for (int i = 0; i < num_samples; i++) {
        HAL_I2C_Mem_Read(&hi2c1, MPU_6050_ADDR, OUTPUT_START_REGISTER, 1, mpu_raw_data, 14, 100);
        buff_gz += (int16_t)(mpu_raw_data[12] << 8 | mpu_raw_data[13]);
        HAL_Delay(1); // 1ms delay yielding 1s total calibration time
    }

    mpu.cal_gz = (int16_t)(buff_gz / num_samples);
    mpu.yaw_angle = 0.0f;
    mpu.is_calibrated = 1;
}

void MPU6050_Update(float dt) {
    if (!mpu.is_calibrated) return;

    uint8_t data[14];
    // Start reading 14 bytes from 0x3B (Accel X) to 0x48 (Gyro Z L)
    if (HAL_I2C_Mem_Read(&hi2c1, MPU_6050_ADDR, OUTPUT_START_REGISTER, 1, data, 14, 10) == HAL_OK) {
        
        // Accel data (Registers 0x3B to 0x40)
        mpu.raw_ax = (int16_t)(data[0] << 8 | data[1]);
        mpu.raw_ay = (int16_t)(data[2] << 8 | data[3]);
        mpu.raw_az = (int16_t)(data[4] << 8 | data[5]);
        
        // Convert to g's. For +-2g range, scale factor is 16384 LSB/g
        mpu.ax = (float)mpu.raw_ax / 16384.0f;
        mpu.ay = (float)mpu.raw_ay / 16384.0f;

        // Gyro data (Registers 0x43 to 0x48) - Z axis is at index 12 & 13
        mpu.raw_gz = (int16_t)(data[12] << 8 | data[13]);
        
        // Remove offset
        int16_t gz_calibrated = mpu.raw_gz - mpu.cal_gz;
        
        // Apply deadband (noise floor exclusion)
        if (gz_calibrated > -10 && gz_calibrated < 10) {
            gz_calibrated = 0;
        }

        // Convert to degrees per second. 
        // For +-500 deg/s range, the scale factor is 65.5 LSB/(deg/s)
        mpu.gz_vel = (float)gz_calibrated / 65.5f;

        // Integrate
        mpu.yaw_angle += (mpu.gz_vel * dt);
    }
}
