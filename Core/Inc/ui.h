/**
 * @file    ui.h
 * @brief   State machine, LED control, button handling
 *
 * Adapted from reference ui.h. Provides:
 * - State_Handle() — main loop state machine dispatcher
 * - LED_Heartbeat() — periodic LED toggle for alive indication
 * - Button_Poll() — debounced button reading
 */
#ifndef UI_H
#define UI_H

#include "stm32f4xx_hal.h"
#include "mouse.h"
#include <stdbool.h>

/* LED heartbeat interval (ms) */
#define LED_HEARTBEAT_INTERVAL  250

/* Button debounce */
#define BUTTON_DEBOUNCE_MS  50

typedef struct {
    bool isPressed;
    bool wasPressed;
    uint32_t pressStart;
    uint32_t duration;
} Button_t;

extern volatile Button_t button1;  /* PB8 */
extern volatile Button_t button2;  /* PB9 */

#define SHORT_PRESS(d)  ((d) > 50  && (d) < 500)
#define LONG_PRESS(d)   ((d) > 1000)

void State_Handle(void);
void LED_Heartbeat(void);
void Button_Poll(void);

#endif /* UI_H */
