#ifndef STATUS_H
#define STATUS_H

#include "stm32f4xx_hal.h"

// Define the system states
typedef enum {
    STATE_EXPLORE,
    STATE_TRY_1,
    STATE_TRY_2,
    STATE_TRY_3,
    STATE_RUN
} SystemState_t;

// Define specific blink codes for your components
typedef enum {
    STATUS_OK = 0,               // Solid ON or OFF
    STATUS_ERR_MOTOR = 1,        // 1 Blink: N20 Motor/Encoder missing signal
    STATUS_ERR_IR = 2,           // 2 Blinks: IR sensor disconnected/bad read
    STATUS_ERR_MPU6050 = 3,      // 3 Blinks: MPU6050 I2C failure
    STATUS_ERR_BUCK = 4          // 4 Blinks: MP1584 Voltage drop (if monitoring ADC)
} StatusCode_t;

void Status_Init(void);
void Status_SetState(SystemState_t new_state);
SystemState_t Status_GetState(void);
void Status_SetModeLEDs(uint8_t led1, uint8_t led2, uint8_t led3);
void Status_SetMessage(StatusCode_t code, uint8_t is_fast);
void Status_Update(void);

#endif // STATUS_H