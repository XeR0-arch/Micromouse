#include "status.h"
#include "gpio.h" // For LED and Button pins

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
static uint8_t btn1_state = GPIO_PIN_SET; // Pull-up assumed or active low/high. Let's assume push gives SET if not pulled up.
static uint8_t btn2_state = GPIO_PIN_SET; 

#define DEBOUNCE_DELAY_MS 50

void Status_Init(void) {
    // Buttons are PB8 and PB9
    // LEDs are PA8, PA11, PA12
    // PC13 is onboard LED
    Status_SetState(STATE_EXPLORE);
    Status_SetMessage(STATUS_OK, 0);
}

void Status_SetState(SystemState_t new_state) {
    current_state = new_state;

    // Based on the state, set the LEDs
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
            Status_SetModeLEDs(1, 1, 1); // Or a sweeping sequence
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
    current_message = code;
    message_is_fast = is_fast;
    blink_count = 0;
    led_is_on = 0;
    message_active = 1;
    message_cooldown = 0; // start immediately
    // PC13 is usually active low on Bluepill/Blackpill
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // turn off initially
}

void Status_Update(void) {
    uint32_t current_tick = HAL_GetTick();

    // 1. Handle PC13 Blinking Message
    if (current_message != STATUS_OK) {
        uint32_t blink_period = message_is_fast ? 100 : 500;
        
        if (message_cooldown > 0) {
            // Wait between message repeats (e.g. 2 seconds)
            if (current_tick - blink_last_tick >= 2000) {
                message_cooldown = 0;
                blink_count = 0;
                blink_last_tick = current_tick;
            }
        } 
        else if (current_tick - blink_last_tick >= blink_period) {
            blink_last_tick = current_tick;
            
            if (led_is_on) {
                // Turn off
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // Active Low
                led_is_on = 0;
                blink_count++;
                
                // Have we finished blinking the code?
                if (blink_count >= current_message) {
                    message_cooldown = 1; // Wait before repeating
                }
            } else {
                // Turn on
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
                led_is_on = 1;
            }
        }
    } else {
        // Keep PC13 off if STATUS_OK
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    }

    // 2. Handle Button Inputs (PB8 = Try Switch, PB9 = Run)
    // Assuming buttons pull down to GND, so 0 is pressed.
    uint8_t btn1_now = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
    uint8_t btn2_now = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);

    if (btn1_now == GPIO_PIN_RESET && btn1_state == GPIO_PIN_SET && (current_tick - btn1_last_press > DEBOUNCE_DELAY_MS)) {
        btn1_last_press = current_tick;
        // Cycle Try modes
        if (current_state == STATE_EXPLORE) Status_SetState(STATE_TRY_1);
        else if (current_state == STATE_TRY_1) Status_SetState(STATE_TRY_2);
        else if (current_state == STATE_TRY_2) Status_SetState(STATE_TRY_3);
        else Status_SetState(STATE_EXPLORE);
    }
    btn1_state = btn1_now;

    if (btn2_now == GPIO_PIN_RESET && btn2_state == GPIO_PIN_SET && (current_tick - btn2_last_press > DEBOUNCE_DELAY_MS)) {
        btn2_last_press = current_tick;
        // Start run mode
        Status_SetState(STATE_RUN);
    }
    btn2_state = btn2_now;
}
