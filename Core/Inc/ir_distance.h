#ifndef __IR_DISTANCE_H
#define __IR_DISTANCE_H

#include "main.h"

// Number of IR sensors
#define IR_SENSOR_COUNT 6

typedef struct {
    uint16_t ambient[IR_SENSOR_COUNT];
    uint16_t active[IR_SENSOR_COUNT];
    int16_t  value[IR_SENSOR_COUNT]; // active - ambient
} IR_Data_t;

extern IR_Data_t ir_data;

// Initialize IR subsystem
void IR_Distance_Init(void);

// Perform a full reading of all IR sensors (Ambient Rejection)
// Note: Blocks for a short period (~few milliseconds) to allow capacitors to charge
void IR_Distance_Read(void);

#endif /* __IR_DISTANCE_H */
