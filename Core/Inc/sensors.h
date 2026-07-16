/**
 * @file    sensors.h
 * @brief   IR distance sensor reading with ambient rejection
 *
 * Following the reference architecture: each sensor is read individually.
 * Since we only have one emitter pin (PB2), all emitters will turn on
 * when reading a specific sensor, but the logic matches the reference's
 * sequential approach.
 */
#ifndef SENSORS_H
#define SENSORS_H

#include "stm32f4xx_hal.h"

/* Number of IR averages per reading */
#define SENSOR_NUM_SAMPLES  50

/* Delay (microseconds) for IR LED to stabilise after switching on */
#define SENSOR_LED_SETTLE_US  200

typedef enum { SENSOR_RAW = 0, SENSOR_MM = 1, SENSOR_CM = 2 } SensorUnit_t;

/* Read individual sensor distances.
 * These are BLOCKING but run in main loop context only. */
double Sensor_GetLeftFront(SensorUnit_t unit);
double Sensor_GetRightFront(SensorUnit_t unit);
double Sensor_GetLeftSide(SensorUnit_t unit);
double Sensor_GetRightSide(SensorUnit_t unit);

#endif /* SENSORS_H */
