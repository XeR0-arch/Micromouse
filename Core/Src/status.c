#include "status.h"
#include "gpio.h" // For LED and Button pins

// -- Hardware Definitions --
#define EXT_ERR_LED_PORT GPIOA
#define EXT_ERR_LED_PIN  GPIO_PIN_15 // Update this to your actual external LED pin

// -- Private Variables --
static SystemState_t current_state = STATE_EXPLORE;

// UI sequence settings
static uint32_t blink_last_tick = 0;
static StatusCode_t current_message = STATUS_OK;
static uint8_t message_is_fast = 0;
static uint8_t blink_count = 0;
static uint8_t led_is_on = 0;
static uint8_t message_active = 0;
static uint32_t message_cooldown = 0;

// Button debouncing
static uint32_t btn1_last_press = 0;
static uint32_t btn2_last_press = 0;
static uint8_t btn1_state = GPIO_PIN_SET; 
static uint8_t btn2_state = GPIO_PIN_SET; 

#define DEBOUNCE_DELAY_MS 50

void Status_Init(void) {
    Status_SetState(STATE_EXPLORE);
    Status_SetMessage(STATUS_OK, 0);
    
    // Ensure external LED starts turned off
    HAL_GPIO_WritePin(EXT_ERR_LED_PORT, EXT_ERR_LED_PIN, GPIO_PIN_RESET);
}

void Status_SetState(SystemState_t new_state) {
    current_state = new_state;

    switch (current_state) {
        case STATE_EXPLORE:
            Status_SetModeLEDs(0, 0, 0);
            break;
        case STATE_TRY_1:
            Status_SetModeLEDs(1, 0, 0);
            break;
        case STATE_TRY_2:
            Status_SetModeLEDs(0, 1, 0);
            break;
        case STATE_TRY_3:
            Status_SetModeLEDs(0, 0, 1);
            break;
        case STATE_RUN:
            Status_SetModeLEDs(1, 1, 1);
            break;
    }
}

SystemState_t Status_GetState(void) {
    return current_state;
}

void Status_SetModeLEDs(uint8_t led1, uint8_t led2, uint8_t led3) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, led1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, led2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, led3 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Status_SetMessage(StatusCode_t code, uint8_t is_fast) {
    // Only reset the sequence if the error code is actually changing
    if (current_message != code) {
        current_message = code;
        message_is_fast = is_fast;
        blink_count = 0;
        led_is_on = 0;
        message_active = 1;
        message_cooldown = 0;
        
        // Turn off both LEDs initially
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // Onboard Active Low
        HAL_GPIO_WritePin(EXT_ERR_LED_PORT, EXT_ERR_LED_PIN, GPIO_PIN_RESET); // External Active High
    }
}

void Status_Update(void) {
    uint32_t current_tick = HAL_GetTick();

    // 1. Handle Error Blinking on PC13 and External LED
    if (current_message != STATUS_OK) {
        uint32_t blink_period = message_is_fast ? 100 : 250; // ms per toggle

        if (message_cooldown > 0) {
            // Wait between message repeats (1000ms = 1 second)
            if (current_tick - blink_last_tick >= 1000) {
                message_cooldown = 0;
                blink_count = 0;
                blink_last_tick = current_tick;
            }
        } 
        else if (current_tick - blink_last_tick >= blink_period) {
            blink_last_tick = current_tick;

            if (led_is_on) {
                // Turn OFF LEDs
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); 
                HAL_GPIO_WritePin(EXT_ERR_LED_PORT, EXT_ERR_LED_PIN, GPIO_PIN_RESET);
                led_is_on = 0;
                blink_count++;

                // Have we finished blinking the code?
                if (blink_count >= current_message) {
                    message_cooldown = 1; // Wait 1 second before repeating
                }
            } else {
                // Turn ON LEDs
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); 
                HAL_GPIO_WritePin(EXT_ERR_LED_PORT, EXT_ERR_LED_PIN, GPIO_PIN_SET);
                led_is_on = 1;
            }
        }
    } else {
        // Keep LEDs off if STATUS_OK
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_GPIO_WritePin(EXT_ERR_LED_PORT, EXT_ERR_LED_PIN, GPIO_PIN_RESET);
    }

    // 2. Handle Button Inputs (PB8 = Try Switch, PB9 = Run)
    uint8_t btn1_now = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
    uint8_t btn2_now = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);

    if (btn1_now == GPIO_PIN_RESET && btn1_state == GPIO_PIN_SET && (current_tick - btn1_last_press > DEBOUNCE_DELAY_MS)) {
        btn1_last_press = current_tick;
        if (current_state == STATE_EXPLORE) Status_SetState(STATE_TRY_1);
        else if (current_state == STATE_TRY_1) Status_SetState(STATE_TRY_2);
        else if (current_state == STATE_TRY_2) Status_SetState(STATE_TRY_3);
        else Status_SetState(STATE_EXPLORE);
    }
    btn1_state = btn1_now;

    if (btn2_now == GPIO_PIN_RESET && btn2_state == GPIO_PIN_SET && (current_tick - btn2_last_press > DEBOUNCE_DELAY_MS)) {
        btn2_last_press = current_tick;
        Status_SetState(STATE_RUN);
    }
    btn2_state = btn2_now;
}