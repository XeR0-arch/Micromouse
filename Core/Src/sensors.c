/**
 * @file    sensors.c
 * @brief   IR distance sensor reading with ambient rejection
 *
 * Adapted exactly from the reference adc.c approach. Each sensor is read
 * individually by toggling the emitter, reading ambient, reading active,
 * and computing the difference.
 */
#include "sensors.h"
#include "adc.h"
#include "main.h"
#include <math.h>

extern ADC_HandleTypeDef hadc1;

/* ---- Helpers ---- */

/** Turn emitter OFF */
static void emitter_off(void)
{
    HAL_GPIO_WritePin(ir_led_1_GPIO_Port, ir_led_1_Pin, GPIO_PIN_RESET);
}

/** Turn emitter ON */
static void emitter_on(void)
{
    HAL_GPIO_WritePin(ir_led_1_GPIO_Port, ir_led_1_Pin, GPIO_PIN_SET);
}

/** Simple microsecond-ish busy-wait (approximate at 100 MHz) */
static void delay_short(uint32_t us)
{
    volatile uint32_t count = us * 25;  /* ~25 cycles/us at 100 MHz */
    while (count--) {}
}

/** Read a single ADC channel via HAL polling */
static uint16_t read_adc_channel(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel      = channel;
    sConfig.Rank          = 1;
    sConfig.SamplingTime  = ADC_SAMPLETIME_84CYCLES;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        return 0;

    uint16_t val = 0;
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
        val = HAL_ADC_GetValue(&hadc1);
    else
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR);

    HAL_ADC_Stop(&hadc1);
    return val;
}

/**
 * @brief  Generic sensor read with ambient rejection.
 * @param  channel   ADC channel to read
 * @param  unit      Desired output unit (RAW / MM / CM)
 * @param  log_a     Logarithmic curve coefficient A
 * @param  log_b     Logarithmic curve coefficient B
 * @param  log_c     Logarithmic curve coefficient C
 */
static double sensor_read(uint32_t channel, SensorUnit_t unit,
                           double log_a, double log_b, double log_c)
{
    double ambient_sum = 0.0;
    double active_sum  = 0.0;
    double raw_value;

    /* 1. Emitter OFF — read ambient */
    emitter_off();
    for (int i = 0; i < SENSOR_NUM_SAMPLES; i++)
        ambient_sum += read_adc_channel(channel);

    /* 2. Emitter ON — wait for LED to settle, then read active */
    emitter_on();
    delay_short(SENSOR_LED_SETTLE_US);
    for (int i = 0; i < SENSOR_NUM_SAMPLES; i++)
        active_sum += read_adc_channel(channel);

    /* 3. Emitter OFF */
    emitter_off();

    /* 4. Compute ambient-rejected value */
    raw_value = (ambient_sum / SENSOR_NUM_SAMPLES)
              - (active_sum / SENSOR_NUM_SAMPLES);

    /* 5. Apply distance curve if not RAW */
    if (unit != SENSOR_RAW)
{
    if (raw_value > 1.0)
    {
        raw_value = (log_a / log(raw_value - log_b)) - log_c;
        if (raw_value > 220.0) raw_value = 220.0;
        if (raw_value < 0.0)   raw_value = 0.0;
    }
    else
    {
        /* No meaningful reflected signal => far object or open side.
           Report max range, not the near-zero raw diff. */
        raw_value = 220.0;
    }
}

    if (unit == SENSOR_CM)
        raw_value /= 10.0;

    return raw_value;
}

/* ---- Public API ---- */

/* Calibration curves — these coefficients MUST be tuned for your sensors.
 * Initial values taken from reference project.
 * Format: distance_mm = A / ln(raw - B) - C                              */

double Sensor_GetLeftFront(SensorUnit_t unit)
{
    return sensor_read(ADC_CHANNEL_4, unit, 2694.1, 121.6, 295.2);
}

double Sensor_GetRightFront(SensorUnit_t unit)
{
    return sensor_read(ADC_CHANNEL_3, unit, 2294.1, 296.5, 249.0);
}

double Sensor_GetLeftSide(SensorUnit_t unit)
{
    return sensor_read(ADC_CHANNEL_5, unit, 2079.3, 182.8, 225.0);
}

double Sensor_GetRightSide(SensorUnit_t unit)
{
    return sensor_read(ADC_CHANNEL_2, unit, 1985.9, 427.7, 215.1);
}
