/**
 * @file    ui.c
 * @brief   State machine, LED heartbeat, button handling
 *
 * Adapted from reference ui.c.
 */
#include "ui.h"
#include "pid.h"
#include "motors.h"
#include "main.h"

volatile Button_t button1 = {false, false, 0, 0};
volatile Button_t button2 = {false, false, 0, 0};

static uint32_t last_heartbeat_tick = 0;

/* ---------- State Machine ---------- */
void State_Handle(void)
{
    switch (mouse.state)
    {
        case MOUSE_INIT:
            /* Blink on-board LED during init */
            HAL_GPIO_TogglePin(on_board_led_GPIO_Port, on_board_led_Pin);
            HAL_Delay(500);
            break;

        case MOUSE_CRITICAL:
            /* Emergency: stop everything */
            Motors_Stop();
            PID_Disable(&motorLeft);
            PID_Disable(&motorRight);
            Mouse_ControllerDisable(&mouse);
            mouse.state = MOUSE_STOP;
            break;

        case MOUSE_STOP:
            Motors_Stop();
            PID_Disable(&motorLeft);
            PID_Disable(&motorRight);
            Mouse_ControllerDisable(&mouse);
            break;

        case MOUSE_IDLE:
            /* Waiting for user input */
            break;

        case MOUSE_RUN:
            /* Autonomous maze solving (future) */
            break;

        case MOUSE_TEST:
            /* Test mode (future) */
            break;

        case MOUSE_PID:
            /* PID tuning mode: enable PID only, no move controller */
            Mouse_ControllerDisable(&mouse);
            PID_Enable(&motorLeft);
            PID_Enable(&motorRight);
            break;

        case MOUSE_MANUAL:
            /* Manual mode: disable PID, set PWM directly from set_rpm */
            PID_Disable(&motorLeft);
            PID_Disable(&motorRight);
            Mouse_ControllerDisable(&mouse);
            Motors_SetSpeed(&motorLeft,  motorLeft.set_rpm);
            Motors_SetSpeed(&motorRight, motorRight.set_rpm);
            break;

        case MOUSE_MOVE_CONTROLLER:
            /* Full controller: PID + move controller */
            PID_Enable(&motorLeft);
            PID_Enable(&motorRight);
            Mouse_ControllerEnable(&mouse);
            break;
    }
}

/* ---------- LED Heartbeat ---------- */
void LED_Heartbeat(void)
{
    uint32_t now = HAL_GetTick();
    if (now - last_heartbeat_tick >= LED_HEARTBEAT_INTERVAL)
    {
        HAL_GPIO_TogglePin(on_board_led_GPIO_Port, on_board_led_Pin);
        last_heartbeat_tick = now;
    }
}

/* ---------- Button Polling ---------- */
void Button_Poll(void)
{
    uint32_t now = HAL_GetTick();

    /* Button 1: PB8 */
    GPIO_PinState b1 = HAL_GPIO_ReadPin(button_1_GPIO_Port, button_1_Pin);
    if (b1 == GPIO_PIN_RESET && !button1.isPressed)
    {
        if (now - button1.pressStart > BUTTON_DEBOUNCE_MS)
        {
            button1.isPressed  = true;
            button1.wasPressed = false;
            button1.pressStart = now;
        }
    }
    else if (b1 == GPIO_PIN_SET && button1.isPressed)
    {
        button1.duration   = now - button1.pressStart;
        button1.isPressed  = false;
        button1.wasPressed = true;
    }

    /* Button 2: PB9 */
    GPIO_PinState b2 = HAL_GPIO_ReadPin(button_2_GPIO_Port, button_2_Pin);
    if (b2 == GPIO_PIN_RESET && !button2.isPressed)
    {
        if (now - button2.pressStart > BUTTON_DEBOUNCE_MS)
        {
            button2.isPressed  = true;
            button2.wasPressed = false;
            button2.pressStart = now;
        }
    }
    else if (b2 == GPIO_PIN_SET && button2.isPressed)
    {
        button2.duration   = now - button2.pressStart;
        button2.isPressed  = false;
        button2.wasPressed = true;
    }
}
