/**
 * @file    mouse.h
 * @brief   Central micromouse state + move controller
 *
 * The Mouse_t struct holds all runtime state: position, heading,
 * sensor readings, state machine, controller flags, map data.
 * Inspired by reference sMOUSE.
 */
#ifndef MOUSE_H
#define MOUSE_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* Cardinal directions — bitmask encoding for map walls */
#define DIR_WEST    1   /* 0b0001 */
#define DIR_SOUTH   2   /* 0b0010 */
#define DIR_EAST    4   /* 0b0100 */
#define DIR_NORTH   8   /* 0b1000 */

/* Move controller speed limits (RPM) */
#define FORWARD_MAX_SPEED    180.0f
#define DIRECTION_MAX_SPEED  180.0f

/* Mouse state machine */
typedef enum {
    MOUSE_INIT = 0,
    MOUSE_CRITICAL = 1,
    MOUSE_STOP = 2,
    MOUSE_IDLE = 3,
    MOUSE_RUN = 4,
    MOUSE_TEST = 5,
    MOUSE_PID = 6,
    MOUSE_MANUAL = 7,
    MOUSE_MOVE_CONTROLLER = 8
} MouseState_t;

/* MAP_SIZE: number of cells in the map array */
#define MAP_SIZE 16

typedef struct Mouse_tag {

    MouseState_t state;
    uint8_t face_direction;

    /* Controller flags */
    bool is_controller_enable;
    bool forward_control;
    bool rotation_control;

    /* Controller velocity outputs (RPM) */
    float forward;
    float direction;

    /* Odometry — actual position (mm) and heading (degrees) */
    float actual_position_x;
    float actual_position_y;
    float actual_angle;

    /* Target position (mm) and heading (degrees) */
    float new_position_x;
    float new_position_y;
    float new_angle;

    /* Controller working variables */
    float distance_to_travel;
    float angle_to_achieve;

    /* IR sensor distances (mm) */
    double left_front_sensor_mm;
    double right_front_sensor_mm;
    double left_side_sensor_mm;
    double right_side_sensor_mm;

    /* Map */
    uint8_t map[MAP_SIZE];
    uint8_t current_map_index;

} Mouse_t;

extern Mouse_t mouse;

/* Move controller — called from timer ISR */
void Mouse_ControllerForward(Mouse_t *m);
void Mouse_ControllerDirection(Mouse_t *m);

/* Controller enable/disable */
void Mouse_ControllerEnable(Mouse_t *m);
void Mouse_ControllerDisable(Mouse_t *m);
bool Mouse_ControllerIsEnabled(Mouse_t *m);

/* High-level movement commands */
void Mouse_SetPosition(Mouse_t *m, float new_x, float new_y);
void Mouse_SetOrientation(Mouse_t *m, float new_angle);
void Mouse_MoveCellForward(Mouse_t *m, uint8_t num_cells);

#endif /* MOUSE_H */
