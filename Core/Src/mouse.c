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
#include "mpu6050.h"
#include <math.h>
#include <stdio.h>

Mouse_t mouse;

/* PD gains for move controller */
static float Fkp = 2.5f, Fkd = 0.00f;   /* Forward  */
static float Rkp_turn = 2.5f, Rkd_turn = 0.00f; /* In-place Rotation */
static float Rkp_wall = 0.3f, Rkd_wall = 0.00f; /* Wall-following — Rkp needs tuning on hardware */

/* Controller time step — matches PID_TIME_STEP */
#define CTRL_TIME_STEP  0.001f

/* Wall-following tuning */
#define WALL_PRESENT_THRESHOLD_MM  160.0f  /* below this, a side sensor counts as "wall present" */
#define WALL_FOLLOW_TARGET_MM      80.0f   /* desired standoff distance from a single wall */

/* Front-wall approach + alignment tuning */
#define FRONT_WALL_APPROACH_MM      180.0f /* start braking for the front wall inside this range */
#define FRONT_WALL_TARGET_MM        77.5f  /* midpoint of the desired 75-80mm standoff */
#define FRONT_WALL_TOLERANCE_MM     2.5f   /* +/- band counted as "arrived" -> 75.0-80.0mm */
#define FRONT_WALL_ALIGN_KP         0.6f   /* approach-speed gain — tune on hardware */
#define FRONT_WALL_SAFETY_CUTOFF_MM 40.0f  /* hard stop regardless of controller math */

/* ---------- Forward controller ---------- */
void Mouse_ControllerForward(Mouse_t *m)
{
    float out;
    float prev_dist = m->distance_to_travel;
    float front_avg = ((float)m->left_front_sensor_mm + (float)m->right_front_sensor_mm) / 2.0f;

    /* Hard safety cutoff — independent of the controller math below, in
     * case a sensor glitch produces a bad reading. Last line of defense,
     * not the primary braking mechanism (that's the approach block next). */
    if (m->left_front_sensor_mm < FRONT_WALL_SAFETY_CUTOFF_MM
     || m->right_front_sensor_mm < FRONT_WALL_SAFETY_CUTOFF_MM)
    {
        m->forward   = 0.0f;
        m->direction = 0.0f;
        m->state     = MOUSE_STOP;
        return;
    }

    /* Front-wall approach control. Replaces the old "stop if both sensors
     * < 80mm" binary check — that was an abrupt full-speed-to-zero cutoff
     * with no deceleration profile, so at higher forward speeds the mouse
     * could carry enough momentum past the trigger point to hit the wall.
     * Once within FRONT_WALL_APPROACH_MM, treat the wall itself as the
     * target and brake proportionally down to FRONT_WALL_TARGET_MM
     * (~7.5-8cm standoff), same PD shape as the position controller below,
     * just measuring distance from the wall instead of the XY target. */
    if (front_avg < FRONT_WALL_APPROACH_MM)
    {
        m->distance_to_travel = front_avg - FRONT_WALL_TARGET_MM;

        out = FRONT_WALL_ALIGN_KP * m->distance_to_travel;

        if (out > FORWARD_MAX_SPEED) out = FORWARD_MAX_SPEED;
        if (out < 0.0f)              out = 0.0f; /* never reverse here */

        if (fabsf(m->distance_to_travel) <= FRONT_WALL_TOLERANCE_MM)
        {
            m->forward   = 0.0f;
            m->direction = 0.0f;
            m->state     = MOUSE_STOP;
        }
        else
        {
            m->forward = out;
        }
        return;
    }

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
    /* True when angle_to_achieve holds a millimeter wall-offset error rather
     * than a degree heading error — the ±180 wrap below is only meaningful
     * for real degrees and must be skipped for mm values (e.g. a 220mm
     * offset would otherwise wrap to -140mm and flip the correction's sign). */
    bool skip_wrap = false;

    if (m->forward_control)
    {
        /* Driving forward — wall following */
        current_Rkp = Rkp_wall;
        current_Rkd = Rkd_wall;

        float front_avg = ((float)m->left_front_sensor_mm + (float)m->right_front_sensor_mm) / 2.0f;
        bool front_present = front_avg < FRONT_WALL_APPROACH_MM;
        bool left_present  = m->left_side_sensor_mm  < WALL_PRESENT_THRESHOLD_MM;
        bool right_present = m->right_side_sensor_mm < WALL_PRESENT_THRESHOLD_MM;

        if (front_present)
        {
            /* Close enough to be braking for the front wall — square up
             * against it instead of side-wall following, so the turn that
             * follows starts from a known-perpendicular heading rather than
             * whatever angle side-wall following left us at. Sign is a
             * starting guess (right - left); verify on hardware and flip if
             * it turns the wrong way. */
            m->angle_to_achieve = (float)m->right_front_sensor_mm - (float)m->left_front_sensor_mm;
            skip_wrap = true;
        }
        else if (left_present && right_present)
        {
            /* Both walls present — center between them. Sign here is a
             * starting guess (right - left); verify on hardware and flip
             * if the mouse steers into a wall instead of centering. */
            m->angle_to_achieve = (float)m->right_side_sensor_mm
                                 - (float)m->left_side_sensor_mm;
            skip_wrap = true;
        }
        else if (left_present)
        {
            /* Only left wall — hold standoff distance from it */
            m->angle_to_achieve = WALL_FOLLOW_TARGET_MM - (float)m->left_side_sensor_mm;
            skip_wrap = true;
        }
        else if (right_present)
        {
            /* Only right wall — hold standoff distance from it */
            m->angle_to_achieve = (float)m->right_side_sensor_mm - WALL_FOLLOW_TARGET_MM;
            skip_wrap = true;
        }
        else
        {
            /* No walls — hold heading toward the target XY position.
             * actual_angle is now an unbounded gyro accumulator, so the
             * error must be computed with ShortestAngleError rather than
             * a plain subtraction + single ±360 correction. */
            float target_heading_deg = atan2f((m->new_position_x - m->actual_position_x),
                                               (m->new_position_y - m->actual_position_y))
                                        * RAD_TO_DEG;
            m->angle_to_achieve = MPU6050_ShortestAngleError(target_heading_deg, m->actual_angle);
        }
    }
    else
    {
        /* Rotating in place. actual_angle is unbounded, so use
         * ShortestAngleError rather than a raw subtraction — a raw
         * subtraction plus a single ±360 correction can't correctly wrap
         * once actual_angle has accumulated past ±360 over several turns. */
        m->angle_to_achieve = MPU6050_ShortestAngleError(m->new_angle, m->actual_angle);
        current_Rkp = Rkp_turn;
        current_Rkd = Rkd_turn;
    }

    /* Wrap to [-180, +180] — only for real degree values. Wall-following
     * error is millimeters and must skip this (see skip_wrap above). The
     * two branches above already return pre-wrapped degrees via
     * ShortestAngleError, so this is a harmless no-op for them and only
     * matters if that call is ever changed back to a raw subtraction. */
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

    /* Slow down forward speed when angle error is large */
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
