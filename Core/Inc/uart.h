#ifndef __UART_H
#define __UART_H

#include "main.h"

/* Print a string via UART (blocking) */
void UART_Print(const char *msg);

/* Send a single character */
void UART_SendChar(char c);

#endif /* __UART_H */
