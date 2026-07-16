/**
 * @file    map.c
 * @brief   16x16 wall-bitmap maze map
 *
 * Each cell is a byte: bits 0-3 = WESN walls, bit 7 = visited.
 * Cell index = row * 16 + col  (row 0 = south, col 0 = west).
 * Wall mirroring ensures adjacent cells stay consistent.
 */
#include "map.h"
#include "uart.h"
#include <stdio.h>

/* Wall threshold for sensor-based detection (mm) */
#define WALL_DETECT_THRESHOLD  150.0

void Map_Init(Mouse_t *m)
{
    /* Clear all cells */
    for (int i = 0; i < MAP_SIZE; i++)
        m->map[i] = 0;

    /* Set outer boundary walls with a simple loop */
    for (int col = 0; col < MAP_COLS; col++)
    {
        /* South boundary (row 0) */
        m->map[0 * MAP_COLS + col] |= DIR_SOUTH;
        /* North boundary (row 15) */
        m->map[(MAP_ROWS - 1) * MAP_COLS + col] |= DIR_NORTH;
    }

    for (int row = 0; row < MAP_ROWS; row++)
    {
        /* West boundary (col 0) */
        m->map[row * MAP_COLS + 0] |= DIR_WEST;
        /* East boundary (col 15) */
        m->map[row * MAP_COLS + (MAP_COLS - 1)] |= DIR_EAST;
    }

    /* Mouse starts at cell (0, 0) — bottom-left corner, facing north */
    m->mouse_x = 0;
    m->mouse_y = 0;
    m->current_map_index = 0;
}

void Map_AddWall(Mouse_t *m, uint8_t wall_dir)
{
    uint8_t idx = m->current_map_index;
    uint8_t col = idx % MAP_COLS;
    uint8_t row = idx / MAP_COLS;

    m->map[idx] |= wall_dir;

    /* Mirror wall to adjacent cell */
    if (wall_dir == DIR_NORTH && row < (MAP_ROWS - 1))
        m->map[idx + MAP_COLS] |= DIR_SOUTH;
    if (wall_dir == DIR_SOUTH && row > 0)
        m->map[idx - MAP_COLS] |= DIR_NORTH;
    if (wall_dir == DIR_EAST && col < (MAP_COLS - 1))
        m->map[idx + 1] |= DIR_WEST;
    if (wall_dir == DIR_WEST && col > 0)
        m->map[idx - 1] |= DIR_EAST;
}

void Map_Update(Mouse_t *m)
{
    if (m->map[m->current_map_index] & MAP_VISITED)
        return;

    m->map[m->current_map_index] |= MAP_VISITED;

    switch (m->face_direction)
    {
        case DIR_NORTH:
            if (m->left_side_sensor_mm < WALL_DETECT_THRESHOLD)
                Map_AddWall(m, DIR_WEST);
            if (m->right_side_sensor_mm < WALL_DETECT_THRESHOLD)
                Map_AddWall(m, DIR_EAST);
            if (m->right_front_sensor_mm < WALL_DETECT_THRESHOLD &&
                m->left_front_sensor_mm < WALL_DETECT_THRESHOLD)
                Map_AddWall(m, DIR_NORTH);
            break;

        case DIR_EAST:
            if (m->left_side_sensor_mm < WALL_DETECT_THRESHOLD)
                Map_AddWall(m, DIR_NORTH);
            if (m->right_side_sensor_mm < WALL_DETECT_THRESHOLD)
                Map_AddWall(m, DIR_SOUTH);
            if (m->right_front_sensor_mm < WALL_DETECT_THRESHOLD &&
                m->left_front_sensor_mm < WALL_DETECT_THRESHOLD)
                Map_AddWall(m, DIR_EAST);
            break;

        case DIR_SOUTH:
            if (m->left_side_sensor_mm < WALL_DETECT_THRESHOLD)
                Map_AddWall(m, DIR_EAST);
            if (m->right_side_sensor_mm < WALL_DETECT_THRESHOLD)
                Map_AddWall(m, DIR_WEST);
            if (m->right_front_sensor_mm < WALL_DETECT_THRESHOLD &&
                m->left_front_sensor_mm < WALL_DETECT_THRESHOLD)
                Map_AddWall(m, DIR_SOUTH);
            break;

        case DIR_WEST:
            if (m->left_side_sensor_mm < WALL_DETECT_THRESHOLD)
                Map_AddWall(m, DIR_SOUTH);
            if (m->right_side_sensor_mm < WALL_DETECT_THRESHOLD)
                Map_AddWall(m, DIR_NORTH);
            if (m->right_front_sensor_mm < WALL_DETECT_THRESHOLD &&
                m->left_front_sensor_mm < WALL_DETECT_THRESHOLD)
                Map_AddWall(m, DIR_WEST);
            break;
    }
}

void Map_Print(Mouse_t *m)
{
    char buf[64];
    UART_Print("\r\n--- MAP ---\r\n");

    for (int row = MAP_ROWS - 1; row >= 0; row--)
    {
        /* Top border of row */
        for (int col = 0; col < MAP_COLS; col++)
        {
            uint8_t idx = row * MAP_COLS + col;
            UART_Print("+");
            UART_Print((m->map[idx] & DIR_NORTH) ? "--" : "  ");
        }
        UART_Print("+\r\n");

        /* Cell content */
        for (int col = 0; col < MAP_COLS; col++)
        {
            uint8_t idx = row * MAP_COLS + col;
            UART_Print((m->map[idx] & DIR_WEST) ? "|" : " ");
            if (idx == m->current_map_index)
                UART_Print("@");
            else if (m->map[idx] & MAP_VISITED)
                UART_Print("x");
            else
                UART_Print(".");
        }
        /* Right border of last cell */
        {
            uint8_t idx = row * MAP_COLS + (MAP_COLS - 1);
            UART_Print((m->map[idx] & DIR_EAST) ? "|" : " ");
        }
        UART_Print("\r\n");
    }

    /* Bottom border */
    for (int col = 0; col < MAP_COLS; col++)
    {
        uint8_t idx = col;
        UART_Print("+");
        UART_Print((m->map[idx] & DIR_SOUTH) ? "--" : "  ");
    }
    UART_Print("+\r\n");

    sprintf(buf, "Cell:%d (%d,%d) Dir:%d\r\n",
            m->current_map_index, m->mouse_x, m->mouse_y, m->face_direction);
    UART_Print(buf);
}
