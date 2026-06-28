#ifndef __FLOODFILL_H
#define __FLOODFILL_H

#include <stdint.h>
#include <stdbool.h>

// Mirror structure from floodfill.c
typedef struct cell
{
    bool updated;
    bool wall_px;
    bool wall_nx;
    bool wall_py;
    bool wall_ny;
    uint8_t value;
} MazeCell;

// External variables declared in floodfill.c
extern int x;
extern int y;
extern uint8_t orientiation;
extern MazeCell maze[16][16];

// Functions declared in floodfill.c
void update_cell(bool array[], MazeCell maze[][16]);
void update_maze(bool backtracking, MazeCell maze[][16]);
uint8_t next_move(bool backtracking, MazeCell maze[][16]);
void move(uint8_t direction);

#endif /* __FLOODFILL_H */
