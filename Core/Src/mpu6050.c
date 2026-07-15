#include "mpu6050.h"
#include "i2c.h"
#include "status.h"
#include <math.h>

/* I2C Address and Configuration Registers */
#define MPU_6050_ADDR (0x68 << 1)
#define PWR_MGMT_1 0x6B
#define CONFIG_REG 0x1A
#define SMPLRT_DIV 0x19
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define OUTPUT_START_REGISTER 0x3B

/* MPU6050 Hardware Offset Registers */
#define MPU_XA_OFFS_H 0x06
#define MPU_YA_OFFS_H 0x08
#define MPU_ZA_OFFS_H 0x0A
#define MPU_XG_OFFS_USRH 0x13
#define MPU_YG_OFFS_USRH 0x15
#define MPU_ZG_OFFS_USRH 0x17

/* Your Calibrated Hardware Offsets */
#define CAL_AX_OFFSET (-2882)
#define CAL_AY_OFFSET (994)
#define CAL_AZ_OFFSET (571)
#define CAL_GX_OFFSET (15)
#define CAL_GY_OFFSET (-16)
#define CAL_GZ_OFFSET (-18)

/* Low-pass filter tuning.
 * alpha in (0,1]: lower = smoother but more lag.
 * 0.2 is a reasonable start at ~200 Hz sample rate. */
#define GYRO_LPF_ALPHA 0.2f

volatile MPU6050_t mpu = {0};

/* Set by TIM10 ISR, consumed by MPU6050_Service() in foreground. */
static volatile uint8_t mpu_update_pending = 0;

/* EMA filter state for gz_vel. Not volatile: only touched from
 * MPU6050_Update(), which only ever runs in foreground context
 * via MPU6050_Service(). */
static float gz_vel_filtered = 0.0f;

static void MPU6050_WriteWord(uint8_t regAddr, int16_t value)
{
    uint8_t data[2];
    data[0] = (uint8_t)((value >> 8) & 0xFF);
    data[1] = (uint8_t)(value & 0xFF);
    HAL_I2C_Mem_Write(&hi2c1, MPU_6050_ADDR, regAddr, 1, data, 2, 100);
    HAL_Delay(1);
}

void MPU6050_SetHardwareOffsets(int16_t ax_off, int16_t ay_off, int16_t az_off,
                                int16_t gx_off, int16_t gy_off, int16_t gz_off)
{
    MPU6050_WriteWord(MPU_XA_OFFS_H, ax_off);
    MPU6050_WriteWord(MPU_YA_OFFS_H, ay_off);
    MPU6050_WriteWord(MPU_ZA_OFFS_H, az_off);
    MPU6050_WriteWord(MPU_XG_OFFS_USRH, gx_off);
    MPU6050_WriteWord(MPU_YG_OFFS_USRH, gy_off);
    MPU6050_WriteWord(MPU_ZG_OFFS_USRH, gz_off);
}

uint8_t MPU6050_Init(void)
{
    uint8_t pwr_mgmt_1_config = 0x01; // PLL with X Gyro
    uint8_t config_config = 0x02;     // DLPF 2 (~98 Hz gyro BW)
    uint8_t smprt_div_config = 0x04;  // 200 Hz sample
    uint8_t gyro_config = 0x08;       // ±500 deg/s → 65.5 LSB/(deg/s)
    uint8_t accel_config = 0x00;      // ±2g

    if (HAL_I2C_IsDeviceReady(&hi2c1, MPU_6050_ADDR, 3, 100) != HAL_OK)
    {
        return 0;
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

    MPU6050_SetHardwareOffsets(CAL_AX_OFFSET, CAL_AY_OFFSET, CAL_AZ_OFFSET,
                               CAL_GX_OFFSET, CAL_GY_OFFSET, CAL_GZ_OFFSET);

    /* === FIX #1: Actually calibrate zero-rate offset instead of faking it ===
     * Robot MUST be perfectly still on the ground during this (~500 ms).
     */
    MPU6050_Calibrate();

    return 1;
}

void MPU6050_Calibrate(void)
{
    uint8_t mpu_raw_data[14];
    long buff_gz = 0;
    uint16_t num_samples = 500;

    for (int i = 0; i < num_samples; i++)
    {
        HAL_I2C_Mem_Read(&hi2c1, MPU_6050_ADDR, OUTPUT_START_REGISTER, 1,
                         mpu_raw_data, 14, 100);
        buff_gz += (int16_t)(mpu_raw_data[12] << 8 | mpu_raw_data[13]);
        HAL_Delay(1);
    }

    mpu.cal_gz = (int16_t)(buff_gz / num_samples);
    mpu.yaw_angle = 0.0f;
    mpu.is_calibrated = 1;
    gz_vel_filtered = 0.0f;
}

void MPU6050_ScheduleUpdate(void)
{
    mpu_update_pending = 1;
}

void MPU6050_Service(void)
{
    static uint32_t last_update_tick = 0;
    uint8_t update_pending;

    __disable_irq();
    update_pending = mpu_update_pending;
    mpu_update_pending = 0;
    __enable_irq();

    if (!update_pending || !mpu.is_calibrated)
    {
        return;
    }

    uint32_t now = HAL_GetTick();

    /* === FIX #2: Default dt matches 200 Hz (5 ms), not 40 Hz (25 ms) === */
    float dt = 0.005f;
    if (last_update_tick != 0U)
    {
        uint32_t elapsed = now - last_update_tick;
        if (elapsed > 0U && elapsed <= 100U)
        {
            dt = (float)elapsed / 1000.0f;
        }
    }
    last_update_tick = now;

    MPU6050_Update(dt);
}

void MPU6050_Update(float dt)
{
    if (!mpu.is_calibrated)
        return;

    uint8_t data[2];
    // GYRO_ZOUT_H / L
    if (HAL_I2C_Mem_Read(&hi2c1, MPU_6050_ADDR, 0x47, 1, data, 2, 10) == HAL_OK)
    {
        mpu.raw_gz = (int16_t)(data[0] << 8 | data[1]);
        int16_t gz_calibrated = mpu.raw_gz - mpu.cal_gz;

        // Tight deadband: ±4 was zeroing real slow-turn rates and
        // under-integrating yaw during 90° spins.
        // Applied BEFORE the filter so small noise below threshold
        // doesn't leak into the running average and drift yaw at rest.
        if (gz_calibrated > -2 && gz_calibrated < 2)
        {
            gz_calibrated = 0;
        }

        // ±500 deg/s range → 65.5 LSB/(deg/s)
        float gz_vel_raw = (float)gz_calibrated / 65.5f;

        // --- EMA low-pass filter ---
        gz_vel_filtered = GYRO_LPF_ALPHA * gz_vel_raw + (1.0f - GYRO_LPF_ALPHA) * gz_vel_filtered;

        mpu.gz_vel = gz_vel_filtered;
        mpu.yaw_angle += (mpu.gz_vel * dt);
    }
}

/*
 * Returns the shortest signed angular error (target - current),
 * wrapped into (-180, 180]. Always integrate mpu.yaw_angle as a raw,
 * UNBOUNDED accumulator (never wrap it in place) and only wrap at the
 * point of comparison, using this function. Wrapping the accumulator
 * itself creates a discontinuity that can make a turn loop's exit
 * condition re-trigger mid-turn and overshoot past the target.
 */
float MPU6050_ShortestAngleError(float target_deg, float current_deg)
{
    float error = fmodf(target_deg - current_deg, 360.0f);
    if (error > 180.0f)
    {
        error -= 360.0f;
    }
    else if (error <= -180.0f)
    {
        error += 360.0f;
    }
    return error;
}

/* Returns yaw wrapped to (-180, +180] for display/logging.
 * NEVER use this for integration or turn control. */
float MPU6050_GetYawWrapped(void)
{
    float yaw = fmodf(mpu.yaw_angle, 360.0f);
    if (yaw > 180.0f)  yaw -= 360.0f;
    if (yaw <= -180.0f) yaw += 360.0f;
    return yaw;
}

/* Alternative: 0–360° wrapping */
float MPU6050_GetYaw360(void)
{
    float yaw = fmodf(mpu.yaw_angle, 360.0f);
    if (yaw < 0) yaw += 360.0f;
    return yaw;
}
