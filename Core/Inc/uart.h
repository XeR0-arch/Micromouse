#ifndef __UART_H
#define __UART_H

#include "main.h"

// Initialization routine (if any specific config is needed beyond CubeMX)
void UART_Debug_Init(void);

// Safe print wrapper (optional, printf works directly after retargeting)
void UART_Print(const char *msg);

#endif /* __UART_H */
