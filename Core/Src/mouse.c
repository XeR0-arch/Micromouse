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
#include "pid.h"
#include "mpu6050.h"
#include "uart.h"
#include <math.h>
#include <stdio.h>

Mouse_t mouse;

/* PD gains for move controller */
static float Fkp = 2.5f, Fkd = 0.00f;   /* Forward  */
static float Rkp_turn = 2.5f, Rkd_turn = 0.00f; /* In-place Rotation */
static float Rkp_wall = 0.1f, Rkd_wall = 0.00f; /* Wall-following */

/* Controller time step — matches PID_TIME_STEP */
#define CTRL_TIME_STEP  0.001f

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

    /* Front sensor forward calibration — smoothly correct odometry to eliminate forward drift */
    if (m->left_front_sensor_mm < 160.0 && m->right_front_sensor_mm < 160.0)
    {
        /* 77.5mm is the desired stopping distance from the wall (7.5 to 8 cm) */
        float avg_front = (m->left_front_sensor_mm + m->right_front_sensor_mm) / 2.0f;
        float true_dist = avg_front - 77.5f;
        float alpha = 0.1f; /* Gently pull odometry to reality to prevent derivative spikes */

        if (m->face_direction == DIR_NORTH) 
            m->actual_position_y = (1.0f - alpha) * m->actual_position_y + alpha * (m->new_position_y - true_dist);
        else if (m->face_direction == DIR_SOUTH) 
            m->actual_position_y = (1.0f - alpha) * m->actual_position_y + alpha * (m->new_position_y + true_dist);
        else if (m->face_direction == DIR_EAST)  
            m->actual_position_x = (1.0f - alpha) * m->actual_position_x + alpha * (m->new_position_x - true_dist);
        else if (m->face_direction == DIR_WEST)  
            m->actual_position_x = (1.0f - alpha) * m->actual_position_x + alpha * (m->new_position_x + true_dist);

        /* Recompute distance_to_travel after correction */
        if (m->face_direction == DIR_NORTH)      m->distance_to_travel = m->new_position_y - m->actual_position_y;
        else if (m->face_direction == DIR_SOUTH) m->distance_to_travel = fabsf(m->new_position_y - m->actual_position_y);
        else if (m->face_direction == DIR_WEST)  m->distance_to_travel = fabsf(m->new_position_x - m->actual_position_x);
        else if (m->face_direction == DIR_EAST)  m->distance_to_travel = m->new_position_x - m->actual_position_x;
    }

    /* PD control */
    out = (Fkp * m->distance_to_travel)
        + (Fkd * (m->distance_to_travel - prev_dist) / CTRL_TIME_STEP);

    /* Clamp */
    if (out > FORWARD_MAX_SPEED)
        out = FORWARD_MAX_SPEED;
    else if (out < -FORWARD_MAX_SPEED)
        out = -FORWARD_MAX_SPEED;

    /* Front wall emergency stop — lowered to 60mm to prevent premature stop */
    if (m->left_front_sensor_mm < 60.0 && m->right_front_sensor_mm < 60.0)
    {
        m->forward   = 0.0f;
        m->direction = 0.0f;
        m->state     = MOUSE_STOP;
        return;
    }

    /* Arrival check (within 5mm) */
    if (m->distance_to_travel > 5.0f)
    {
        /* Simple acceleration limiter (prevents wheelies)
         * Limits how fast the forward speed can increase, 
         * but lets the PD controller fully handle rapid deceleration. */
        float max_accel = 0.75f; /* Max RPM increase per ms */
        if (out > m->forward + max_accel) {
            m->forward += max_accel;
        } else {
            m->forward = out;
        }
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

    if (m->forward_control)
    {
        /* Driving forward — only align when BOTH walls are present */
        if (m->left_side_sensor_mm < 160.0 && m->right_side_sensor_mm < 160.0)
        {
            /* Two walls — use wall alignment */
            current_Rkp = Rkp_wall;
            current_Rkd = Rkd_wall;
            m->angle_to_achieve = 80.0f - (float)m->left_side_sensor_mm;

            /* Add aggression if error is large (e.g. after a bad turn) */
            if (fabsf(m->angle_to_achieve) > 12.0f)
            {
                current_Rkp = 0.5f;
            }
        }
        else
        {
            /* One wall or no walls — no correction, drive straight */
            current_Rkp = 0.0f;
            current_Rkd = 0.0f;
            m->angle_to_achieve = 0.0f;
        }
    }
    else
    {
        /* Rotating in place */
        m->angle_to_achieve = m->new_angle - m->actual_angle;
        current_Rkp = Rkp_turn;
        current_Rkd = Rkd_turn;
    }

    /* Wrap to [-180, +180] */
    if (m->angle_to_achieve < -180.0f)
        m->angle_to_achieve += 360.0f;
    else if (m->angle_to_achieve > 180.0f)
        m->angle_to_achieve -= 360.0f;

    /* PD control */
    out = (current_Rkp * m->angle_to_achieve)
        + (current_Rkd * (m->angle_to_achieve - prev_ang) / CTRL_TIME_STEP);

    /* Clamp */
    if (out > DIRECTION_MAX_SPEED)
        out = DIRECTION_MAX_SPEED;
    else if (out < -DIRECTION_MAX_SPEED)
        out = -DIRECTION_MAX_SPEED;

    /* Slow down forward speed when angle error is large */
    if (m->angle_to_achieve < -2.5f || m->angle_to_achieve > 2.5f)
        m->forward *= 0.85f;

    m->direction = out;

    /* In-place rotation arrival check */
    if (!m->forward_control && fabsf(m->angle_to_achieve) < 1.5f)
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

    /* Flush motor PID accumulated errors from previous movement */
    PID_ResetState(&motorLeft);
    PID_ResetState(&motorRight);
}

void Mouse_SetOrientation(Mouse_t *m, float new_angle)
{
    m->new_angle         = new_angle;
    m->state             = MOUSE_MOVE_CONTROLLER;
    m->forward_control   = false;
    m->rotation_control  = true;
    m->distance_to_travel = 0.0f;
    m->angle_to_achieve  = 0.0f;   /* flush stale wall-alignment error */
    m->forward           = 0.0f;
    m->direction         = 0.0f;

    /* Flush motor PID accumulated errors from previous movement */
    PID_ResetState(&motorLeft);
    PID_ResetState(&motorRight);

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
