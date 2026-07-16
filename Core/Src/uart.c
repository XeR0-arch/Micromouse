/**
 * @file    uart.c
 * @brief   UART output + printf retargeting
 *
 * Uses CubeMX-initialized huart1 (USART1, PA9/PA10, 115200 baud).
 */
#include "uart.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

void UART_SendChar(char c)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&c, 1, HAL_MAX_DELAY);
}

void UART_Print(const char *msg)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

/* ---- printf retargeting ---- */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}

int _write(int file, char *ptr, int len)
{
    (void)file;
    for (int i = 0; i < len; i++)
        __io_putchar(*ptr++);
    return len;
}
