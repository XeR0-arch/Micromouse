/**
 * @file    floodfill.h
 * @brief   Floodfill maze-solving algorithm
 *
 * Uses BFS flood fill to calculate cell distances from center.
 * Maintains its own 16x16 maze with wall data and flood values.
 */
#ifndef FLOODFILL_H
#define FLOODFILL_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    bool updated;
    bool wall_px;   /* wall in +x direction (east)  */
    bool wall_nx;   /* wall in -x direction (west)  */
    bool wall_py;   /* wall in +y direction (north) */
    bool wall_ny;   /* wall in -y direction (south) */
    uint8_t value;  /* flood distance from goal     */
} MazeCell;

/* Floodfill position and orientation */
extern int x;
extern int y;
extern uint8_t orientiation;
extern MazeCell maze[16][16];

/* Orientation encoding:
 *   192 = facing +y (North)
 *    48 = facing +x (East)
 *    12 = facing -y (South)
 *     3 = facing -x (West)
 */

void Floodfill_Init(void);
void Floodfill_Run(bool backtracking);

void update_cell(bool array[], MazeCell maze[][16]);
void update_maze(bool backtracking, MazeCell maze[][16]);
uint8_t next_move(bool backtracking, MazeCell maze[][16]);
void move(uint8_t direction);

#endif /* FLOODFILL_H */
