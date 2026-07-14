#include "mpu6050.h"
#include "i2c.h"
#include "status.h"
#include <math.h>

/* I2C Address and Configuration Registers */
#define MPU_6050_ADDR         (0x68 << 1)
#define PWR_MGMT_1            0x6B
#define CONFIG_REG            0x1A // Named CONFIG_REG to avoid conflict
#define SMPLRT_DIV            0x19
#define GYRO_CONFIG           0x1B
#define ACCEL_CONFIG          0x1C
#define OUTPUT_START_REGISTER 0x3B

/* MPU6050 Hardware Offset Registers (Electronic Cats / i2cdevlib addresses) */
#define MPU_XA_OFFS_H         0x06
#define MPU_YA_OFFS_H         0x08
#define MPU_ZA_OFFS_H         0x0A
#define MPU_XG_OFFS_USRH      0x13
#define MPU_YG_OFFS_USRH      0x15
#define MPU_ZG_OFFS_USRH      0x17

/* Your Calibrated Hardware Offsets */
#define CAL_AX_OFFSET  (-2882)
#define CAL_AY_OFFSET  (994)
#define CAL_AZ_OFFSET  (571)
#define CAL_GX_OFFSET  (15)
#define CAL_GY_OFFSET  (-16)
#define CAL_GZ_OFFSET  (-18)

MPU6050_t mpu = {0};

/**
  * @brief  Helper to write a 16-bit signed offset to a specific register pair (High + Low byte).
  */
static void MPU6050_WriteWord(uint8_t regAddr, int16_t value) {
    uint8_t data[2];
    data[0] = (uint8_t)((value >> 8) & 0xFF); // High byte
    data[1] = (uint8_t)(value & 0xFF);        // Low byte
    HAL_I2C_Mem_Write(&hi2c1, MPU_6050_ADDR, regAddr, 1, data, 2, 100);
    HAL_Delay(1); // Small delay to ensure DMP/register write completion
}

/**
  * @brief  Write all 6-axis hardware offsets directly into the MPU6050 chip registers.
  */
void MPU6050_SetHardwareOffsets(int16_t ax_off, int16_t ay_off, int16_t az_off,
                                int16_t gx_off, int16_t gy_off, int16_t gz_off) {
    MPU6050_WriteWord(MPU_XA_OFFS_H, ax_off);
    MPU6050_WriteWord(MPU_YA_OFFS_H, ay_off);
    MPU6050_WriteWord(MPU_ZA_OFFS_H, az_off);
    MPU6050_WriteWord(MPU_XG_OFFS_USRH, gx_off);
    MPU6050_WriteWord(MPU_YG_OFFS_USRH, gy_off);
    MPU6050_WriteWord(MPU_ZG_OFFS_USRH, gz_off);
}

/**
  * @brief  Initialize MPU6050 and load pre-calibrated hardware offsets.
  */
uint8_t MPU6050_Init(void) {
    uint8_t pwr_mgmt_1_config = 0x01;  // Set clock source bit 0 (PLL with X Gyro reference)
    uint8_t config_config     = 0x02;  // DLPF value to 2 (98Hz bandwidth, 3ms delay for Gyro)
    uint8_t smprt_div_config  = 0x04;  // 200Hz sampling (if DLPF is on: 1kHz / (1 + 4))
    uint8_t gyro_config       = 0x08;  // Range +-500 deg/s (Scale factor: 65.5 LSB/deg/s)
    uint8_t accel_config      = 0x00;  // Range +-2g

    // Check if MPU6050 replies
    if (HAL_I2C_IsDeviceReady(&hi2c1, MPU_6050_ADDR, 3, 100) != HAL_OK) {
        return 0; // Device not ready
    }

    HAL_I2C_Mem_Write(&hi2c1, MPU_6050_ADDR, PWR_MGMT_1, 1, &pwr_mgmt_1_config, 1, 100);
    HAL_Delay(10);
    HAL_I2C_Mem_Write(&hi2c1, MPU_6050_ADDR, CONFIG_REG, 1, &config_config, 1, 100);
    HAL_Delay(10);
    HAL_I2C_Mem_Write(&hi2c1, MPU_6050_ADDR, SMPLRT_DIV, 1, &smprt_div_config, 1, 100);
    HAL_Delay(10);
    HAL_I2C_Mem_Write(&hi2c1, MPU_6050_ADDR, GYRO_CONFIG, 1, &gyro_config, 1, 100);
    HAL_Delay(10);
    HAL_I2C_Mem_Write(&hi2c1, MPU_6050_ADDR, ACCEL_CONFIG, 1, &accel_config, 1, 100);
    HAL_Delay(10);

    /* --- Apply Electronic Cats / i2cdevlib Calibrated Offsets directly to Hardware --- */
    MPU6050_SetHardwareOffsets(CAL_AX_OFFSET, CAL_AY_OFFSET, CAL_AZ_OFFSET,
                               CAL_GX_OFFSET, CAL_GY_OFFSET, CAL_GZ_OFFSET);

    mpu.cal_gz = 0;
    mpu.yaw_angle = 0.0f;
    mpu.is_calibrated = 1; // Immediately ready to run!

    return 1; // Success
}

/**
  * @brief  Optional: Quick runtime fine-trim for temperature/mount bias.
  *         Since hardware offsets are already loaded, this will just measure any residual ~1-2 LSB drift.
  */
void MPU6050_Calibrate(void) {
    uint8_t mpu_raw_data[14];
    long buff_gz = 0;
    uint16_t num_samples = 500; // Reduced to 500ms since hardware is already calibrated

    for (int i = 0; i < num_samples; i++) {
        HAL_I2C_Mem_Read(&hi2c1, MPU_6050_ADDR, OUTPUT_START_REGISTER, 1, mpu_raw_data, 14, 100);
        buff_gz += (int16_t)(mpu_raw_data[12] << 8 | mpu_raw_data[13]);
        HAL_Delay(1);
    }

    mpu.cal_gz = (int16_t)(buff_gz / num_samples);
    mpu.yaw_angle = 0.0f;
    mpu.is_calibrated = 1;
}

/**
  * @brief  Update Yaw Angle. Call this in your timer/control loop at interval dt.
  */
void MPU6050_Update(float dt) {
    if (!mpu.is_calibrated) return;

    uint8_t data[2];
    // Start reading at Gyro Z H (register 0x47 = GYRO_ZOUT_H, 0x48 = GYRO_ZOUT_L)
    if (HAL_I2C_Mem_Read(&hi2c1, MPU_6050_ADDR, 0x47, 1, data, 2, 10) == HAL_OK) {
        mpu.raw_gz = (int16_t)(data[0] << 8 | data[1]);

        // Subtract residual software trim (if MPU6050_Calibrate() was called; otherwise cal_gz is 0)
        int16_t gz_calibrated = mpu.raw_gz - mpu.cal_gz;

        // Apply deadband (noise floor exclusion)
        // With hardware offsets active, idle noise is usually within +-2 to +-4 LSB.
        if (gz_calibrated > -4 && gz_calibrated < 4) {
            gz_calibrated = 0;
        }

        // Convert to degrees per second.
        // For +-500 deg/s range, the scale factor is 65.5 LSB/(deg/s)
        mpu.gz_vel = (float)gz_calibrated / 65.5f;

        // Integrate yaw
        mpu.yaw_angle += (mpu.gz_vel * dt);
    }
}
