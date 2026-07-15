#ifndef __IR_DISTANCE_H
#define __IR_DISTANCE_H

#include "main.h"
#include "stdbool.h"

// Number of IR sensors
#define IR_SENSOR_COUNT 4

typedef struct {
	uint16_t ambient[IR_SENSOR_COUNT];
	uint16_t active[IR_SENSOR_COUNT];
	int16_t value[IR_SENSOR_COUNT]; // ambient - active
	int16_t distance[IR_SENSOR_COUNT];
	bool walls[IR_SENSOR_COUNT+2]; // temporary because the flood fill algorithm was designed with 6 sensors
} IR_Data_t;

extern IR_Data_t ir_data;

// Steering error computed by IR_Distance(): distance[0] - distance[3]
// Positive = closer to left wall, negative = closer to right wall
extern int32_t ir_steering_error;

// Initialize IR subsystem
void IR_Distance_Init(void);

// Non-blocking IR read state machine — call at 40Hz from TIM10
// Alternates: read ambient (emitters OFF) → read active (emitters ON)
// Effective update rate: 20Hz, ISR duration: ~200us (no blocking delays)
void IR_Tick(void);

// Convert raw IR values to distance using calibration curves
void IR_Distance(void);

// Determine wall presence from distance data
void GET_Walls(void);

#endif /* __IR_DISTANCE_H */
