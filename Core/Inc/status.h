#ifndef __STATUS_H
#define __STATUS_H

#include "main.h"

// System States
typedef enum {
    STATE_EXPLORE,
    STATE_TRY_1,
    STATE_TRY_2,
    STATE_TRY_3,
    STATE_RUN
} SystemState_t;

// PC13 Error / Status Codes (Blinks)
typedef enum {
    STATUS_OK = 0,
    STATUS_MPU_DETECTED = 2,
    STATUS_MPU_CALIBRATED = 3,
    STATUS_IR_CALIBRATED = 4,
    // Add more as needed
    ERROR_GENERAL = 10,
    ERROR_I2C = 11,
    ERROR_TIMEOUT = 12
} StatusCode_t;

// Init function
void Status_Init(void);

// Non-blocking update function (call in main while(1))
void Status_Update(void);

// Set System State
void Status_SetState(SystemState_t new_state);
SystemState_t Status_GetState(void);

// Set PC13 Status/Error Code (Number of blinks)
// is_fast: if true, blinks fast (error), else slow (status)
void Status_SetMessage(StatusCode_t code, uint8_t is_fast);

// Directly set mode LEDs
void Status_SetModeLEDs(uint8_t led1, uint8_t led2, uint8_t led3);

#endif /* __STATUS_H */
