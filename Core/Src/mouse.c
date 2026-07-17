/**
 * @file    mouse.c
 * @brief   Move controller — forward + direction PD controllers
 *
 * Adapted from reference controller.c. The forward controller drives
 * the mouse toward a target XY position. The direction controller
 * either wall-follows (when driving forward with walls) or rotates
 * to a target heading (when turning in place).
 */
#include "mouse.h"
#include "encoders.h"
#include "uart.h"
#include <math.h>
#include <stdio.h>

Mouse_t mouse;

/* PD gains for move controller */
static float Fkp = 2.5f, Fkd = 0.00f;           /* Forward */
static float Rkp_turn = 2.5f, Rkd_turn = 0.00f; /* In-place Rotation */
static float Rkp_wall = 0.3f, Rkd_wall = 0.00f; /* Wall-following — TUNE on hardware */

/* Controller time step — matches PID_TIME_STEP */
#define CTRL_TIME_STEP  0.001f

/* Wall-centering tuning */
#define WALL_PRESENT_THRESHOLD_MM   130.0f  /* TUNE: below this = wall detected */
#define WALL_FOLLOW_TARGET_MM        80.0f  /* single-wall standoff distance */

/* ---------- Forward controller ---------- */
void Mouse_ControllerForward(Mouse_t *m)
{
    float out;
    float prev_dist = m->distance_to_travel;

    /* Compute remaining distance based on facing direction */
    if (m->face_direction == DIR_NORTH)
        m->distance_to_travel = m->new_position_y - m->actual_position_y;
    else if (m->face_direction == DIR_SOUTH)
        m->distance_to_travel = fabsf(m->new_position_y - m->actual_position_y);
    else if (m->face_direction == DIR_WEST)
        m->distance_to_travel = fabsf(m->new_position_x - m->actual_position_x);
    else if (m->face_direction == DIR_EAST)
        m->distance_to_travel = m->new_position_x - m->actual_position_x;

    /* PD control */
    out = (Fkp * m->distance_to_travel)
        + (Fkd * (m->distance_to_travel - prev_dist) / CTRL_TIME_STEP);

    /* Clamp */
    if (out > FORWARD_MAX_SPEED)
        out = FORWARD_MAX_SPEED;
    else if (out < -FORWARD_MAX_SPEED)
        out = -FORWARD_MAX_SPEED;

    /* Front wall emergency stop — prevents overshoot into walls */
    if (m->left_front_sensor_mm < 80.0 && m->right_front_sensor_mm < 80.0)
    {
        m->forward   = 0.0f;
        m->direction = 0.0f;
        m->state     = MOUSE_STOP;
        return;
    }

    /* Arrival check (within 5mm) */
    if (m->distance_to_travel > 5.0f)
    {
        m->forward = out;
    }
    else
    {
        m->forward   = 0.0f;
        m->direction = 0.0f;
        m->state     = MOUSE_STOP;
    }
}

/* ---------- Direction controller ---------- */
void Mouse_ControllerDirection(Mouse_t *m)
{
    float out;
    float prev_ang = m->angle_to_achieve;
    float current_Rkp = 0.0f;
    float current_Rkd = 0.0f;
    uint8_t skip_wrap = false;   /* true when angle_to_achieve is a raw mm error, not a real angle */

    if (m->forward_control)
    {
        current_Rkp = Rkp_wall;
        current_Rkd = Rkd_wall;

        uint8_t left_wall  = (m->left_side_sensor_mm  < WALL_PRESENT_THRESHOLD_MM);
        uint8_t right_wall = (m->right_side_sensor_mm < WALL_PRESENT_THRESHOLD_MM);

        if (left_wall && right_wall)
        {
            /* Centering. Positive => closer to LEFT wall => steer right.
             * VERIFY on the real robot — flip the sign if it steers INTO the wall. */
            m->angle_to_achieve = (float)m->right_side_sensor_mm - (float)m->left_side_sensor_mm;
            skip_wrap = true;
        }
        else if (left_wall)
        {
            m->angle_to_achieve = WALL_FOLLOW_TARGET_MM - (float)m->left_side_sensor_mm;
            skip_wrap = true;
        }
        else if (right_wall)
        {
            m->angle_to_achieve = (float)m->right_side_sensor_mm - WALL_FOLLOW_TARGET_MM;
            skip_wrap = true;
        }
        else
        {
            /* No side walls -> heading-hold toward target position. */
            m->angle_to_achieve = (atan2f((m->new_position_x - m->actual_position_x),
                        (m->new_position_y - m->actual_position_y)) * RAD_TO_DEG)
                - m->actual_angle;
        }
    }
    else
    {
        /* Rotating in place */
        m->angle_to_achieve = m->new_angle - m->actual_angle;
        current_Rkp = Rkp_turn;
        current_Rkd = Rkd_turn;
    }

    /* Wrap to [-180,+180] — only valid for real angle errors. Skipped for
     * wall-centering/-following since those are mm differences, and
     * wrapping them can flip the sign. */
    if (!skip_wrap)
    {
        if (m->angle_to_achieve < -180.0f)
            m->angle_to_achieve += 360.0f;
        else if (m->angle_to_achieve > 180.0f)
            m->angle_to_achieve -= 360.0f;
    }

    /* PD control */
    out = (current_Rkp * m->angle_to_achieve)
        + (current_Rkd * (m->angle_to_achieve - prev_ang) / CTRL_TIME_STEP);

    /* Clamp */
    if (out > DIRECTION_MAX_SPEED)
        out = DIRECTION_MAX_SPEED;
    else if (out < -DIRECTION_MAX_SPEED)
        out = -DIRECTION_MAX_SPEED;

    /* NOTE: for wall-centering this reads "off-center by >2.5mm", not
     * >2.5°, since angle_to_achieve is mm here. Tune separately if needed. */
    if (m->angle_to_achieve < -2.5f || m->angle_to_achieve > 2.5f)
        m->forward *= 0.85f;

    m->direction = out;

    /* In-place rotation arrival check */
    if (!m->forward_control && fabsf(m->angle_to_achieve) < 0.5f)
    {
        m->direction = 0.0f;
        m->state     = MOUSE_STOP;
    }
}

/* ---------- Controller enable/disable ---------- */
void Mouse_ControllerEnable(Mouse_t *m)  { m->is_controller_enable = true; }
void Mouse_ControllerDisable(Mouse_t *m) { m->is_controller_enable = false; }
bool Mouse_ControllerIsEnabled(Mouse_t *m) { return m->is_controller_enable; }

/* ---------- High-level movement commands ---------- */
void Mouse_SetPosition(Mouse_t *m, float new_x, float new_y)
{
    m->new_position_x    = new_x;
    m->new_position_y    = new_y;
    m->state             = MOUSE_MOVE_CONTROLLER;
    m->forward_control   = true;
    m->rotation_control  = true;
    m->distance_to_travel = 0.0f;
    m->angle_to_achieve  = 0.0f;
    m->forward           = 0.0f;
    m->direction         = 0.0f;
}

void Mouse_SetOrientation(Mouse_t *m, float new_angle)
{
    m->new_angle        = new_angle;
    m->state            = MOUSE_MOVE_CONTROLLER;
    m->forward_control  = false;
    m->rotation_control = true;
    m->forward          = 0.0f;
    m->direction        = 0.0f;

    /* Update face direction based on cardinal angle */
    if (new_angle == 0.0f)          m->face_direction = DIR_NORTH;
    else if (new_angle == 90.0f)    m->face_direction = DIR_EAST;
    else if (new_angle == -90.0f)   m->face_direction = DIR_WEST;
    else if (new_angle == 180.0f)   m->face_direction = DIR_SOUTH;
    else if (new_angle == -180.0f)  m->face_direction = DIR_SOUTH;
}

void Mouse_MoveCellForward(Mouse_t *m, uint8_t num_cells)
{
    float cell_size = 267.0f; /* mm — cell internal dimension (without walls) */

    if (m->face_direction == DIR_NORTH)
    {
        Mouse_SetPosition(m, m->actual_position_x,
                          m->actual_position_y + (num_cells * cell_size));
        m->current_map_index += num_cells * MAP_COLS;
        m->mouse_y += num_cells;
    }
    else if (m->face_direction == DIR_SOUTH)
    {
        Mouse_SetPosition(m, m->actual_position_x,
                          m->actual_position_y - (num_cells * cell_size));
        m->current_map_index -= num_cells * MAP_COLS;
        m->mouse_y -= num_cells;
    }
    else if (m->face_direction == DIR_WEST)
    {
        Mouse_SetPosition(m, m->actual_position_x - (num_cells * cell_size),
                          m->actual_position_y);
        m->current_map_index -= num_cells;
        m->mouse_x -= num_cells;
    }
    else if (m->face_direction == DIR_EAST)
    {
        Mouse_SetPosition(m, m->actual_position_x + (num_cells * cell_size),
                          m->actual_position_y);
        m->current_map_index += num_cells;
        m->mouse_x += num_cells;
    }
}
