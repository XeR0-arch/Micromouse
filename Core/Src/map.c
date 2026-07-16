/**
 * @file    map.c
 * @brief   Simple wall-bitmap maze map
 *
 * Adapted from reference map.c. Uses a compact 18-cell (3×6) map
 * where each cell is a byte: bits 0-3 = WESN walls, bit 7 = visited.
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

    m->current_map_index = 0;

    /* Set outer boundary walls (3×6 grid) */
    /* Bottom row: cells 0, 1, 2 */
    m->current_map_index = 0;
    Map_AddWall(m, DIR_WEST);
    Map_AddWall(m, DIR_SOUTH);
    m->current_map_index = 1;
    Map_AddWall(m, DIR_SOUTH);
    m->current_map_index = 2;
    Map_AddWall(m, DIR_EAST);
    Map_AddWall(m, DIR_SOUTH);

    /* Left column */
    m->current_map_index = 3;  Map_AddWall(m, DIR_WEST);
    m->current_map_index = 6;  Map_AddWall(m, DIR_WEST);
    m->current_map_index = 9;  Map_AddWall(m, DIR_WEST);
    m->current_map_index = 12; Map_AddWall(m, DIR_WEST);

    /* Top row: cells 15, 16, 17 */
    m->current_map_index = 15;
    Map_AddWall(m, DIR_WEST);
    Map_AddWall(m, DIR_NORTH);
    m->current_map_index = 16;
    Map_AddWall(m, DIR_NORTH);
    m->current_map_index = 17;
    Map_AddWall(m, DIR_EAST);
    Map_AddWall(m, DIR_NORTH);

    /* Right column */
    m->current_map_index = 14; Map_AddWall(m, DIR_EAST);
    m->current_map_index = 11; Map_AddWall(m, DIR_EAST);
    m->current_map_index = 8;  Map_AddWall(m, DIR_EAST);
    m->current_map_index = 5;  Map_AddWall(m, DIR_EAST);

    m->current_map_index = 0;
}

void Map_AddWall(Mouse_t *m, uint8_t wall_dir)
{
    m->map[m->current_map_index] |= wall_dir;

    /* Mirror wall to adjacent cell */
    if (wall_dir == DIR_NORTH && (m->current_map_index + 3) < MAP_SIZE)
        m->map[m->current_map_index + 3] |= DIR_SOUTH;
    if (wall_dir == DIR_EAST && (m->current_map_index + 1) < MAP_SIZE)
        m->map[m->current_map_index + 1] |= DIR_WEST;
    if (wall_dir == DIR_SOUTH && m->current_map_index >= 3)
        m->map[m->current_map_index - 3] |= DIR_NORTH;
    if (wall_dir == DIR_WEST && m->current_map_index >= 1)
        m->map[m->current_map_index - 1] |= DIR_EAST;
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

    for (int row = 5; row >= 0; row--)
    {
        /* Top border of row */
        for (int col = 0; col < 3; col++)
        {
            uint8_t idx = row * 3 + col;
            UART_Print("+");
            UART_Print((m->map[idx] & DIR_NORTH) ? "---" : "   ");
        }
        UART_Print("+\r\n");

        /* Cell content */
        for (int col = 0; col < 3; col++)
        {
            uint8_t idx = row * 3 + col;
            UART_Print((m->map[idx] & DIR_WEST) ? "|" : " ");
            if (m->map[idx] & MAP_VISITED)
                UART_Print(" x ");
            else
                UART_Print(" . ");
        }
        /* Right border of last cell */
        {
            uint8_t idx = row * 3 + 2;
            UART_Print((m->map[idx] & DIR_EAST) ? "|" : " ");
        }
        UART_Print("\r\n");
    }

    /* Bottom border */
    for (int col = 0; col < 3; col++)
    {
        uint8_t idx = col;
        UART_Print("+");
        UART_Print((m->map[idx] & DIR_SOUTH) ? "---" : "   ");
    }
    UART_Print("+\r\n");

    sprintf(buf, "Cell: %d  Dir: %d\r\n", m->current_map_index, m->face_direction);
    UART_Print(buf);
}
