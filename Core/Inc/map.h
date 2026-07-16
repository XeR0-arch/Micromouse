/**
 * @file    map.h
 * @brief   Simple wall-bitmap maze map
 *
 * Adapted from reference map.h. Each cell is encoded as a single byte
 * with wall bits (NSEW) and a VISITED flag.
 */
#ifndef MAP_H
#define MAP_H

#include "stm32f4xx_hal.h"
#include "mouse.h"

/* VISITED flag in cell byte */
#define MAP_VISITED  128  /* 0b10000000 */

void Map_Init(Mouse_t *m);
void Map_AddWall(Mouse_t *m, uint8_t wall_direction);
void Map_Update(Mouse_t *m);
void Map_Print(Mouse_t *m);

#endif /* MAP_H */
