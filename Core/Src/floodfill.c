/**
 * @file    floodfill.c
 * @brief   Floodfill maze-solving algorithm — hardware integrated
 *
 * Algorithm logic is the user's original implementation.
 * Movement stubs are wired to the mouse controller.
 */
#include "floodfill.h"
#include "mouse.h"
#include "sensors.h"
#include "motors.h"
#include "pid.h"
#include "uart.h"
#include <stdio.h>
#include <string.h>

#define MAX_SIZE 256
#define FLOOD_WALL_THRESHOLD  150.0

/* ---- Floodfill state ---- */
int x = 0;
int y = 0;
uint8_t orientiation = 192;  /* start facing +y (North) */

MazeCell maze[16][16];
MazeCell maze1[16][16];

/* ---- Queue (BFS) ---- */
typedef struct {
    int x;
    int y;
    bool junction;
    uint8_t turn;
} XY;

static XY arr[MAX_SIZE];
static XY stack[MAX_SIZE];
static int top = -1;
static int front = -1;
static int bottom = -1;

/* ---- Stack operations ---- */
static void push(int px, int py, uint8_t turn)
{
    if (top < 255)
    {
        ++top;
        stack[top].x = px;
        stack[top].y = py;
        stack[top].turn = turn;
    }
}

static XY pop(void)
{
    XY not_valid;
    not_valid.x = -1;
    not_valid.y = -1;
    not_valid.junction = false;
    if (top >= 0)
    {
        not_valid = stack[top];
        --top;
        return not_valid;
    }
    return not_valid;
}

/* ---- Queue operations (BFS) ---- */
static void enque(int qx, int qy)
{
    if (bottom < 255 && bottom >= -1)
    {
        ++bottom;
        arr[bottom].x = qx;
        arr[bottom].y = qy;
    }
}

static XY dequeue(void)
{
    if (front < bottom)
    {
        ++front;
        return arr[front];
    }
    XY not_valid;
    not_valid.x = -1;
    not_valid.y = -1;
    return not_valid;
}

static void reset_queue(bool backtracking)
{
    front = -1;
    bottom = -1;
    if (backtracking)
    {
        enque(0, 0);
    }
    else
    {
        enque(7, 7);
        enque(7, 8);
        enque(8, 7);
        enque(8, 8);
    }
}

/* ---- Hardware movement ---- */

/* Sensor flag from main.c ISR */
extern volatile bool flag_sensors;
extern volatile bool flag_sensors_in_progress;

/**
 * Service sensor reads while waiting.
 * Must be called during blocking waits so the wall-following
 * controller in the ISR gets fresh sensor data.
 */
static void service_sensors(void)
{
    if (flag_sensors)
    {
        flag_sensors_in_progress = true;
        mouse.left_front_sensor_mm  = Sensor_GetLeftFront(SENSOR_MM);
        mouse.right_front_sensor_mm = Sensor_GetRightFront(SENSOR_MM);
        mouse.left_side_sensor_mm   = Sensor_GetLeftSide(SENSOR_MM);
        mouse.right_side_sensor_mm  = Sensor_GetRightSide(SENSOR_MM);
        flag_sensors = false;
        flag_sensors_in_progress = false;
    }
}

/** Block until the current movement command completes */
static void wait_for_movement(void)
{
    while (mouse.state == MOUSE_MOVE_CONTROLLER)
    {
        service_sensors();
    }
}

/** Convert floodfill orientation to our angle system */
static float orientation_to_angle(uint8_t ori)
{
    switch (ori)
    {
        case 192: return 0.0f;     /* North */
        case 48:  return 90.0f;    /* East  */
        case 12:  return 180.0f;   /* South */
        case 3:   return -90.0f;   /* West  */
        default:  return 0.0f;
    }
}

/** Physical turn to a relative angle, then move forward 1 cell */
static void do_turn_and_forward(float relative_degrees)
{
    if (relative_degrees != 0.0f)
    {
        float target = orientation_to_angle(orientiation) + relative_degrees;
        if (target > 180.0f)  target -= 360.0f;
        if (target < -180.0f) target += 360.0f;

        Mouse_SetOrientation(&mouse, target);
        wait_for_movement();
    }

    Mouse_MoveCellForward(&mouse, 1);
    wait_for_movement();
}

static bool turn_right(void)
{
    do_turn_and_forward(90.0f);
    return true;
}

static bool turn_left(void)
{
    do_turn_and_forward(-90.0f);
    return true;
}

static bool move_forward(void)
{
    do_turn_and_forward(0.0f);
    return true;
}

static bool turn_around(void)
{
    do_turn_and_forward(180.0f);
    return true;
}

/** Sync floodfill x,y back to mouse struct */
static void sync_position(void)
{
    mouse.mouse_x = (uint8_t)x;
    mouse.mouse_y = (uint8_t)y;
    mouse.current_map_index = (uint8_t)(y * MAP_COLS + x);
}

/* ---- Movement dispatcher (user's original logic) ---- */

/* Direction codes:
 *   192 = turn right + forward
 *    48 = turn left + forward
 *    12 = forward
 *     3 = turn around + forward
 */
void move(uint8_t direction)
{
    if (direction == 192 && turn_right())
    {
        if (orientiation == 192)      { orientiation = 48;  x += 1; }
        else if (orientiation == 48)  { orientiation = 12;  y -= 1; }
        else if (orientiation == 12)  { orientiation = 3;   x -= 1; }
        else if (orientiation == 3)   { orientiation = 192; y += 1; }
    }
    else if (direction == 48 && turn_left())
    {
        if (orientiation == 48)       { orientiation = 192; y += 1; }
        else if (orientiation == 192) { orientiation = 3;   x -= 1; }
        else if (orientiation == 12)  { orientiation = 48;  x += 1; }
        else if (orientiation == 3)   { orientiation = 12;  y -= 1; }
    }
    else if (direction == 12 && move_forward())
    {
        if (orientiation == 12)       { y -= 1; }
        else if (orientiation == 192) { y += 1; }
        else if (orientiation == 48)  { x += 1; }
        else if (orientiation == 3)   { x -= 1; }
    }
    else if (direction == 3 && turn_around())
    {
        if (orientiation == 3)        { orientiation = 48;  x += 1; }
        else if (orientiation == 12)  { orientiation = 192; y += 1; }
        else if (orientiation == 192) { orientiation = 12;  y -= 1; }
        else if (orientiation == 48)  { orientiation = 3;   x -= 1; }
    }

    sync_position();
}

/* ---- Algorithm core (user's original logic) ---- */

static bool is_valid(int vx, int vy, MazeCell mz[][16])
{
    if (vx >= 0 && vy >= 0 && vx <= 15 && vy <= 15 && !mz[vx][vy].updated)
        return true;
    return false;
}

void update_cell(bool array[], MazeCell mz[][16])
{
    /* array: 0=left, 1=left_front, 2=right_front, 3=right */
    bool wall_left  = array[0];
    bool wall_front = array[1] && array[2];
    bool wall_right = array[3];

    /* Map sensor readings to absolute walls based on orientation */
    if (orientiation == 192)        /* Facing North (+y) */
    {
        mz[x][y].wall_nx = wall_left;
        mz[x][y].wall_py = wall_front;
        mz[x][y].wall_px = wall_right;
    }
    else if (orientiation == 48)    /* Facing East (+x) */
    {
        mz[x][y].wall_py = wall_left;
        mz[x][y].wall_px = wall_front;
        mz[x][y].wall_ny = wall_right;
    }
    else if (orientiation == 12)    /* Facing South (-y) */
    {
        mz[x][y].wall_px = wall_left;
        mz[x][y].wall_ny = wall_front;
        mz[x][y].wall_nx = wall_right;
    }
    else if (orientiation == 3)     /* Facing West (-x) */
    {
        mz[x][y].wall_ny = wall_left;
        mz[x][y].wall_nx = wall_front;
        mz[x][y].wall_py = wall_right;
    }

    /* Mirror walls to adjacent cells — user's original boundary logic */
    if (x == 15)
    {
        if (y == 0)
        {
            mz[x - 1][y].wall_px = mz[x][y].wall_nx;
            mz[x][y + 1].wall_ny = mz[x][y].wall_py;
        }
        else if (y == 15)
        {
            mz[x - 1][y].wall_px = mz[x][y].wall_nx;
            mz[x][y - 1].wall_py = mz[x][y].wall_ny;
        }
        else
        {
            mz[x - 1][y].wall_px = mz[x][y].wall_nx;
            mz[x][y + 1].wall_ny = mz[x][y].wall_py;
            mz[x][y - 1].wall_py = mz[x][y].wall_ny;
        }
    }
    else if (x == 0)
    {
        if (y == 0)
        {
            mz[x + 1][y].wall_nx = mz[x][y].wall_px;
            mz[x][y + 1].wall_ny = mz[x][y].wall_py;
        }
        else if (y == 15)
        {
            mz[x + 1][y].wall_nx = mz[x][y].wall_px;
            mz[x][y - 1].wall_py = mz[x][y].wall_ny;
        }
        else
        {
            mz[x + 1][y].wall_nx = mz[x][y].wall_px;
            mz[x][y + 1].wall_ny = mz[x][y].wall_py;
            mz[x][y - 1].wall_py = mz[x][y].wall_ny;
        }
    }
    else if (y == 0)
    {
        mz[x + 1][y].wall_nx = mz[x][y].wall_px;
        mz[x][y + 1].wall_ny = mz[x][y].wall_py;
        mz[x - 1][y].wall_px = mz[x][y].wall_nx;
    }
    else if (y == 15)
    {
        mz[x + 1][y].wall_nx = mz[x][y].wall_px;
        mz[x][y - 1].wall_py = mz[x][y].wall_ny;
        mz[x - 1][y].wall_px = mz[x][y].wall_nx;
    }
    else
    {
        mz[x + 1][y].wall_nx = mz[x][y].wall_px;
        mz[x][y - 1].wall_py = mz[x][y].wall_ny;
        mz[x - 1][y].wall_px = mz[x][y].wall_nx;
        mz[x][y + 1].wall_ny = mz[x][y].wall_py;
    }
}

static void mark_un_updated(bool backtracking, MazeCell mz[][16])
{
    for (int i = 0; i < 16; i++)
        for (int j = 0; j < 16; j++)
            mz[i][j].updated = false;

    if (backtracking)
    {
        mz[0][0].value = 0;
        mz[0][0].updated = true;
    }
    else
    {
        mz[7][7].value = 0;  mz[7][7].updated = true;
        mz[7][8].value = 0;  mz[7][8].updated = true;
        mz[8][7].value = 0;  mz[8][7].updated = true;
        mz[8][8].value = 0;  mz[8][8].updated = true;
    }
}

void update_maze(bool backtracking, MazeCell mz[][16])
{
    reset_queue(backtracking);
    mark_un_updated(backtracking, mz);

    XY obj;
    int x_fill, y_fill;

    while (front < bottom)
    {
        obj = dequeue();
        x_fill = obj.x;
        y_fill = obj.y;

        if (is_valid(x_fill + 1, y_fill, mz) && !mz[x_fill][y_fill].wall_px)
        {
            mz[x_fill + 1][y_fill].value = mz[x_fill][y_fill].value + 1;
            mz[x_fill + 1][y_fill].updated = true;
            enque(x_fill + 1, y_fill);
        }
        if (is_valid(x_fill - 1, y_fill, mz) && !mz[x_fill][y_fill].wall_nx)
        {
            mz[x_fill - 1][y_fill].value = mz[x_fill][y_fill].value + 1;
            mz[x_fill - 1][y_fill].updated = true;
            enque(x_fill - 1, y_fill);
        }
        if (is_valid(x_fill, y_fill + 1, mz) && !mz[x_fill][y_fill].wall_py)
        {
            mz[x_fill][y_fill + 1].value = mz[x_fill][y_fill].value + 1;
            mz[x_fill][y_fill + 1].updated = true;
            enque(x_fill, y_fill + 1);
        }
        if (is_valid(x_fill, y_fill - 1, mz) && !mz[x_fill][y_fill].wall_ny)
        {
            mz[x_fill][y_fill - 1].value = mz[x_fill][y_fill].value + 1;
            mz[x_fill][y_fill - 1].updated = true;
            enque(x_fill, y_fill - 1);
        }
    }
}

static void normalize_direction(int times, uint8_t vals[])
{
    for (int i = 0; i < times; i++)
    {
        uint8_t temp = vals[0];
        vals[0] = vals[1];
        vals[1] = vals[2];
        vals[2] = vals[3];
        vals[3] = temp;
    }
}

uint8_t next_move(bool backtracking, MazeCell mz[][16])
{
    uint8_t px = 255, py = 255, nx = 255, ny = 255;
    int count = 0;

    if (x + 1 <= 15 && !mz[x][y].wall_px) { px = mz[x + 1][y].value; count++; }
    if (x - 1 >= 0  && !mz[x][y].wall_nx) { nx = mz[x - 1][y].value; count++; }
    if (y + 1 <= 15 && !mz[x][y].wall_py) { py = mz[x][y + 1].value; count++; }
    if (y - 1 >= 0  && !mz[x][y].wall_ny) { ny = mz[x][y - 1].value; count++; }

    if (count > 1 && top >= 0)
        stack[top].junction = true;

    uint8_t dir = 12;  /* default: forward */
    uint8_t values[4] = {py, px, ny, nx};

    switch (orientiation)
    {
        case 48:  normalize_direction(1, values); break;
        case 12:  normalize_direction(2, values); break;
        case 3:   normalize_direction(3, values); break;
        default:  break;
    }

    if (backtracking)
    {
        uint8_t max_val = values[0];
        if (values[1] > max_val) { max_val = values[1]; dir = 192; }
        if (values[3] > max_val) { max_val = values[3]; dir = 48;  }
        if (values[2] > max_val) { max_val = values[2]; dir = 3;   }
    }
    else
    {
        uint8_t min_val = values[0];
        if (values[1] < min_val) { min_val = values[1]; dir = 192; }
        if (values[3] < min_val) { min_val = values[3]; dir = 48;  }
        if (values[2] < min_val) { min_val = values[2]; dir = 3;   }
    }

    return dir;
}

/* ---- Init & Run ---- */

void Floodfill_Init(void)
{
    x = 0;
    y = 0;
    orientiation = 192;  /* facing North */
    top = -1;
    front = -1;
    bottom = -1;

    /* Clear all walls and set initial flood values */
    for (int i = 0; i < 16; i++)
    {
        for (int j = 0; j < 16; j++)
        {
            maze[i][j].wall_px = false;
            maze[i][j].wall_nx = false;
            maze[i][j].wall_py = false;
            maze[i][j].wall_ny = false;
            maze[i][j].updated = false;
            maze[i][j].value   = 255;
        }
    }

    /* Set center goal */
    maze[7][7].value = 0;
    maze[7][8].value = 0;
    maze[8][7].value = 0;
    maze[8][8].value = 0;

    sync_position();
}

/**
 * @brief  Main solving loop — blocks until center is reached.
 * @param  backtracking  false = solve to center, true = return to start
 */
void Floodfill_Run(bool backtracking)
{
    push(0, 0, 12);
    char buf[100];

    UART_Print("\r\n[DEBUG] Entered Floodfill_Run\r\n");

    while (1)
    {
        /* Block until we get a fresh sensor reading */
        while (!flag_sensors) {}
        
        service_sensors();

        /* Convert sensor distances to wall booleans */
        bool adc[4];
        adc[0] = (mouse.left_side_sensor_mm   < FLOOD_WALL_THRESHOLD);
        adc[1] = (mouse.left_front_sensor_mm  < FLOOD_WALL_THRESHOLD);
        adc[2] = (mouse.right_front_sensor_mm < FLOOD_WALL_THRESHOLD);
        adc[3] = (mouse.right_side_sensor_mm  < FLOOD_WALL_THRESHOLD);

        /* Update walls and recalculate flood */
        update_cell(adc, maze);
        update_maze(backtracking, maze);

        /* Decide next move */
        uint8_t move_dir = next_move(backtracking, maze);

        sprintf(buf, "(%d,%d) ori=%d walls=%d%d%d%d dir=%d val=%d\r\n",
                x, y, orientiation,
                adc[0], adc[1], adc[2], adc[3],
                move_dir, maze[x][y].value);
        UART_Print(buf);

        if (move_dir != 3)
        {
            push(x, y, move_dir);
        }
        else
        {
            XY junction;
            junction.junction = false;
            while (!junction.junction && top >= 0)
                junction = pop();
            if (junction.x >= 0)
                push(junction.x, junction.y, move_dir);
        }

        move(move_dir);

        /* Check if goal reached */
        if (maze[x][y].value == 0)
        {
            Motors_Stop();
            PID_Disable(&motorLeft);
            PID_Disable(&motorRight);
            Mouse_ControllerDisable(&mouse);
            mouse.state = MOUSE_STOP;
            
            sprintf(buf, "Reached goal at (%d,%d)!\r\n", x, y);
            UART_Print(buf);
            return;
        }
    }
}

