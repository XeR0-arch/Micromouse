#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#define MAX_SIZE 256
#define m 16

int x = 0;
int y = 0;
int top = -1;
int front = -1;
int bottom = -1;
// int m = 16;
uint8_t orientiation; // 0b11000000(192)+y 0b00110000(48) +x
                      // 0b00001100(12) -y 0b00000011(3)  -x
// if(PINX == HIGH){
// orientiation = 192;
// }
// else{
// orientiation = 48;
// }

typedef struct cell
{
    bool updated;
    bool wall_px;
    bool wall_nx;
    bool wall_py;
    bool wall_ny;
    uint8_t value;
} MazeCell;

typedef struct coordinate
{
    int x;
    int y;
    bool junction;
    uint8_t turn;
} XY;

XY arr[MAX_SIZE];
XY stack[MAX_SIZE];
MazeCell maze[16][16];
MazeCell maze1[16][16];

// uint16_t maze[16][16]; 5-8 4 bits for walls: 0b0000 (no walls), 0b0001 (wall to the right),
// 0b0010 (wall down), 0b0100 (wall to the left), 0b1000 (wall up) first 4 bits for visited
// first 4 bits for status: 0b1111xxxx.... (visited), 0b0000xxxx.... (not visited);
// end eight for the cell value ie. distance from center(calculated)

void push(int x, int y,uint8_t turn)
{
    if (top < 255)
    {
        ++top;
        stack[top].x = x;
        stack[top].y = y;
        stack[top].turn = turn;
    }
}

XY pop()
{
    XY not_valid;
    not_valid.x = -1;
    not_valid.y = -1;
    if (top >= 0)
    {
        not_valid = stack[top];
        --top;
        return not_valid;
    }
    else
        return not_valid;
}

XY peek()
{
    XY not_valid;
    not_valid.x = -1;
    not_valid.y = -1;
    if (top == -1)
        return not_valid;
    else
        return stack[top];
}

void enque(int x, int y)
{
    if (bottom < 255 && bottom >= -1)
    {
        ++bottom;
        arr[bottom].x = x;
        arr[bottom].y = y;
    }
    // else{
    //     return;
    // }
}

XY dequeue()
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

// int peek_x(){
//     if(front <= bottom){
//         return arr[front].x;
//     }
// }

// int peek_y(){
//     if(front <= bottom){
//         return arr[front].y;
//     }
// }

void reset_queue(bool backtracking)
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

void rotl8_by_2()
{
    uint8_t mask = orientiation & 3;
    mask = mask << 6;
    orientiation = orientiation | (orientiation >> 2);
}

bool turn_right()
{
    // set the motor pins to turn right and return true if the turn is successful
    return true;
}
bool turn_left()
{
    // set the motor pins to turn left and return true if the turn is successful
    return true;
}
bool move_forward()
{
    // set the motor pins to move forward and return true if the turn is successful
    return true;
}
bool turn_around()
{
    // set the motor pins to turn around and return true if the turn is successful
    return true;
}

void move(uint8_t direction)
{
    // 0b11000000 = right, 0b00110000 = left, 0b00001100 = front, 0b00000011 = back
    // if and only if the turn is sucessfull will the orientiation be updated
    // TODO: convert the mess to switch
    // Convert the directions to clock and anticlock and steps to clear the mess below
    if (direction == 192 && turn_right())
    {
        if (orientiation == 192)
        {
            orientiation = 48;
            x += 1;
        }
        else if (orientiation == 48)
        {
            orientiation = 12;
            y -= 1;
        }
        else if (orientiation == 12)
        {
            orientiation = 3;
            x -= 1;
        }
        else if (orientiation == 3)
        {
            orientiation = 192;
            y += 1;
        }
    }
    else if (direction == 48 && turn_left())
    {
        if (orientiation == 48)
        {
            orientiation = 192;
            y += 1;
        }
        else if (orientiation == 192)
        {
            orientiation = 3;
            x -= 1;
        }
        else if (orientiation == 12)
        {
            orientiation = 48;
            x += 1;
        }
        else if (orientiation == 3)
        {
            orientiation = 12;
            y -= 1;
        }
    }
    else if (direction == 12 && move_forward())
    {
        if (orientiation == 12)
        {
            orientiation = 12;
            y -= 1;
        }
        else if (orientiation == 192)
        {
            orientiation = 192;
            y += 1;
        }
        else if (orientiation == 48)
        {
            orientiation = 48;
            x += 1;
        }
        else if (orientiation == 3)
        {
            orientiation = 3;
            x -= 1;
        }
    }
    else if (direction == 3 && turn_around())
    {
        if (orientiation == 3)
        {
            orientiation = 48;
            x += 1;
        }
        else if (orientiation == 12)
        {
            orientiation = 192;
            y += 1;
        }
        else if (orientiation == 192)
        {
            orientiation = 12;
            y -= 1;
        }
        else if (orientiation == 48)
        {
            orientiation = 3;
            x -= 1;
        }
    }
}

bool is_valid(int x, int y, MazeCell maze[][16])
{
    if (x >= 0 && y >= 0 && x <= 15 && y <= 15 && !maze[x][y].updated)
        return true;
    else
        return false;
}

void update_cell(bool array[], MazeCell maze[][16])
{
    // Expected array indices:
    // 0 = left, 1 = left_front, 2 = right_front, 3 = right

    bool wall_left = array[0];
    bool wall_front = array[1] || array[2]; // Combine front sensors
    bool wall_right = array[3];

    // Map logical sensor readings to absolute NSEW cell walls based on orientiation
    if (orientiation == 192)
    { // Facing +y (Up)
        maze[x][y].wall_nx = wall_left;
        maze[x][y].wall_py = wall_front;
        maze[x][y].wall_px = wall_right;
    }
    else if (orientiation == 48)
    { // Facing +x (Right)
        maze[x][y].wall_py = wall_left;
        maze[x][y].wall_px = wall_front;
        maze[x][y].wall_ny = wall_right;
    }
    else if (orientiation == 12)
    { // Facing -y (Down)
        maze[x][y].wall_px = wall_left;
        maze[x][y].wall_ny = wall_front;
        maze[x][y].wall_nx = wall_right;
    }
    else if (orientiation == 3)
    { // Facing -x (Left)
        maze[x][y].wall_ny = wall_left;
        maze[x][y].wall_nx = wall_front;
        maze[x][y].wall_py = wall_right;
    }

    // Border mirror logic (Keep your existing border mirror logic here)
    if (x == 15)
    {
        if (y == 0)
        {
            maze[x - 1][y].wall_px = maze[x][y].wall_nx;
            maze[x][y + 1].wall_ny = maze[x][y].wall_py;
        }
        else if (y == 15)
        {
            maze[x - 1][y].wall_px = maze[x][y].wall_nx;
            maze[x][y - 1].wall_py = maze[x][y].wall_ny;
        }
        else if (y < 15 && y > 0)
        {
            maze[x - 1][y].wall_px = maze[x][y].wall_nx;
            maze[x][y + 1].wall_ny = maze[x][y].wall_py;
            maze[x][y - 1].wall_py = maze[x][y].wall_ny;
        }
    }
    else if (x == 0)
    {
        if (y == 0)
        {
            maze[x + 1][y].wall_nx = maze[x][y].wall_px;
            maze[x][y + 1].wall_ny = maze[x][y].wall_py;
        }
        else if (y == 15)
        {
            maze[x + 1][y].wall_nx = maze[x][y].wall_px;
            maze[x][y - 1].wall_py = maze[x][y].wall_ny;
        }
        else if (y < 15 && y > 0)
        {
            maze[x + 1][y].wall_nx = maze[x][y].wall_px;
            maze[x][y + 1].wall_ny = maze[x][y].wall_py;
            maze[x][y - 1].wall_py = maze[x][y].wall_ny;
        }
    }
    else if (y == 0)
    {
        maze[x + 1][y].wall_nx = maze[x][y].wall_px;
        maze[x][y + 1].wall_ny = maze[x][y].wall_py;
        maze[x - 1][y].wall_px = maze[x][y].wall_nx;
    }
    else if (y == 15)
    {
        maze[x + 1][y].wall_nx = maze[x][y].wall_px;
        maze[x][y - 1].wall_py = maze[x][y].wall_ny;
        maze[x - 1][y].wall_px = maze[x][y].wall_nx;
    }
    else
    {
        maze[x + 1][y].wall_nx = maze[x][y].wall_px;
        maze[x][y - 1].wall_py = maze[x][y].wall_ny;
        maze[x - 1][y].wall_px = maze[x][y].wall_nx;
        maze[x][y + 1].wall_ny = maze[x][y].wall_py;
    }
}

//void update_cell(bool array[], MazeCell maze[][16])
//{
//    // the array should be of ones and zeros not actual
//    // adc values if possible combine left_front and right_front and change index accordingly
//    // scan adc leftmost to rightmost (can skip diagonals for this reading)
//    // adc values stored in a array left_dig, left, left_front, right_front, right, right_dig
//    //                                 0        1       2            3         4        5
//
//    printf("Updating cell");
//    if (orientiation == 192)
//    {
//        maze[x][y].wall_nx = array[1];
//        maze[x][y].wall_py = array[2];
//        maze[x][y].wall_px = array[4];
//    }
//    else if (orientiation == 48)
//    {
//        maze[x][y].wall_py = array[1];
//        maze[x][y].wall_px = array[2];
//        maze[x][y].wall_ny = array[4];
//    }
//    else if (orientiation == 12)
//    {
//        maze[x][y].wall_nx = array[4];
//        maze[x][y].wall_px = array[1];
//        maze[x][y].wall_ny = array[2];
//    }
//    else if (orientiation == 3)
//    {
//        maze[x][y].wall_nx = array[2];
//        maze[x][y].wall_py = array[4];
//        maze[x][y].wall_ny = array[1];
//    }
//
//    if (x == 15)
//    {
//        if (y == 0)
//        {
//            maze[x - 1][y].wall_px = maze[x][y].wall_nx;
//            maze[x][y + 1].wall_ny = maze[x][y].wall_py;
//        }
//        else if (y == 15)
//        {
//            maze[x - 1][y].wall_px = maze[x][y].wall_nx;
//            maze[x][y - 1].wall_py = maze[x][y].wall_ny;
//        }
//        else if (y < 15 && y > 0)
//        {
//            maze[x - 1][y].wall_px = maze[x][y].wall_nx;
//            maze[x][y + 1].wall_ny = maze[x][y].wall_py;
//            maze[x][y - 1].wall_py = maze[x][y].wall_ny;
//        }
//    }
//    else if (x == 0)
//    {
//        if (y == 0)
//        {
//            maze[x + 1][y].wall_nx = maze[x][y].wall_px;
//            maze[x][y + 1].wall_ny = maze[x][y].wall_py;
//        }
//        else if (y == 15)
//        {
//            maze[x + 1][y].wall_nx = maze[x][y].wall_px;
//            maze[x][y - 1].wall_py = maze[x][y].wall_ny;
//        }
//        else if (y < 15 && y > 0)
//        {
//            maze[x + 1][y].wall_nx = maze[x][y].wall_px;
//            maze[x][y + 1].wall_ny = maze[x][y].wall_py;
//            maze[x][y - 1].wall_py = maze[x][y].wall_ny;
//        }
//    }
//    else if (y == 0)
//    {
//        maze[x + 1][y].wall_nx = maze[x][y].wall_px;
//        maze[x][y + 1].wall_ny = maze[x][y].wall_py;
//        maze[x - 1][y].wall_px = maze[x][y].wall_nx;
//    }
//    else if (y == 15)
//    {
//        maze[x + 1][y].wall_nx = maze[x][y].wall_px;
//        maze[x][y - 1].wall_py = maze[x][y].wall_ny;
//        maze[x - 1][y].wall_px = maze[x][y].wall_nx;
//    }
//    else
//    {
//        maze[x + 1][y].wall_nx = maze[x][y].wall_px;
//        maze[x][y - 1].wall_py = maze[x][y].wall_ny;
//        maze[x - 1][y].wall_px = maze[x][y].wall_nx;
//        maze[x][y + 1].wall_ny = maze[x][y].wall_py;
//    }
//}

void mark_un_updated(bool backtracking, MazeCell maze[][16])
{
    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < m; j++)
        {
            maze[i][j].updated = false;
        }
    }
    if (backtracking)
    {
        maze[0][0].value = 0;
        maze[0][0].updated = true;
    }
    else
    {
        maze[7][7].updated = true;
        maze[7][8].updated = true;
        maze[8][7].updated = true;
        maze[8][8].updated = true;
        maze[7][7].value = 0;
        maze[7][8].value = 0;
        maze[8][7].value = 0;
        maze[8][8].value = 0;
    }
}

// bool all_visited()
// {
//     for (int i = 0; i < m; i++)
//     {
//         for (int j = 0; j < m; j++)
//         {
//             if (maze[i][j].updated == false)
//             {
//                 return false;
//             }
//         }
//     }
//     return true;
// }

void update_maze(bool backtracking, MazeCell maze[][16])
{
    reset_queue(backtracking);
    // printf("Looping for bfs\nfront=%d\nBottom=%d",front,bottom);
    mark_un_updated(backtracking, maze);
    // printf("Looping for bfs\nfront=%d\nBottom=%d",front,bottom);
    XY obj;
    int x_fill;
    int y_fill;
    // printf("Looping for bfs\nfront=%d\nBottom=%d",front,bottom);
    while (front < bottom)
    {
        // printf("========LOOP STARTED=========");
        // printf("Looping for bfs\nfront=%d\nBottom=%d",front,bottom);
        obj = dequeue();
        x_fill = obj.x;
        y_fill = obj.y;
        if (is_valid(x_fill + 1, y_fill, maze) && !maze[x_fill][y_fill].wall_px)
        {
            maze[x_fill + 1][y_fill].value = maze[x_fill][y_fill].value + 1;
            maze[x_fill + 1][y_fill].updated = true;
            enque(x_fill + 1, y_fill);
        }
        if (is_valid(x_fill - 1, y_fill, maze) && !maze[x_fill][y_fill].wall_nx)
        {
            maze[x_fill - 1][y_fill].value = maze[x_fill][y_fill].value + 1;
            maze[x_fill - 1][y_fill].updated = true;
            enque(x_fill - 1, y_fill);
        }
        if (is_valid(x_fill, y_fill + 1, maze) && !maze[x_fill][y_fill].wall_py)
        {
            maze[x_fill][y_fill + 1].value = maze[x_fill][y_fill].value + 1;
            maze[x_fill][y_fill + 1].updated = true;
            enque(x_fill, y_fill + 1);
        }
        if (is_valid(x_fill, y_fill - 1, maze) && !maze[x_fill][y_fill].wall_ny)
        {
            maze[x_fill][y_fill - 1].value = maze[x_fill][y_fill].value + 1;
            maze[x_fill][y_fill - 1].updated = true;
            enque(x_fill, y_fill - 1);
        }
    }
}

void normalize_direction(int times, uint8_t arr[])
{
    for (int i = 0; i < times; i++)
    {
        uint8_t temp = arr[0];
        arr[0] = arr[1];
        arr[1] = arr[2];
        arr[2] = arr[3];
        arr[3] = temp;
    }
}

uint8_t next_move(bool backtracking, MazeCell maze[][16])
{
    uint8_t px = 255;
    uint8_t py = 255;
    uint8_t nx = 255;
    uint8_t ny = 255;
    int count = 0;
    // 0b11000000 = right, 0b00110000 = left, 0b00001100 = front, 0b00000011 = back
    if (x + 1 <= 15 && x + 1 >= 0 && y >= 0 && y <= 15 && !maze[x][y].wall_px)
    {
        px = maze[x + 1][y].value;
        count++;
    }
    if (x - 1 <= 15 && x - 1 >= 0 && y >= 0 && y <= 15 && !maze[x][y].wall_nx)
    {
        nx = maze[x - 1][y].value;
        count++;
    }
    if (x <= 15 && x >= 0 && y + 1 >= 0 && y + 1 <= 15 && !maze[x][y].wall_py)
    {
        py = maze[x][y + 1].value;
        count++;
    }
    if (x <= 15 && x >= 0 && y - 1 >= 0 && y - 1 <= 15 && !maze[x][y].wall_ny)
    {
        ny = maze[x][y - 1].value;
        count++;
    }

    if (count > 1)
    {
        stack[top].junction = true;
    }

    uint8_t min_val;
    uint8_t max_val;
    uint8_t dir = 12;
    uint8_t values[4] = {py, px, ny, nx}; // Default is py,px,ny,nx for orientiation == 192
    switch (orientiation)
    {
    case 48:
        normalize_direction(1, values);
        break;
    case 12:
        normalize_direction(2, values);
        break;
    case 3:
        normalize_direction(3, values);
        break;
    default:
        break;
    }
    if (backtracking)
    {
        max_val = values[0];
        if (values[1] > max_val)
        {
            max_val = values[1];
            dir = 192;
        }
        if (values[3] > max_val)
        {
            max_val = values[3];
            dir = 48;
        }
        if (values[2] > max_val)
        {
            max_val = values[2];
            dir = 3;
        }
    }
    else
    {

        min_val = values[0];
        if (values[1] < min_val)
        {
            min_val = values[1];
            dir = 192;
        }
        if (values[3] < min_val)
        {
            min_val = values[3];
            dir = 48;
        }
        if (values[2] < min_val)
        {
            min_val = values[2];
            dir = 3;
        }
    }
    return dir;
}

void clear_walls(MazeCell maze[][16])
{
    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < m; j++)
        {
            maze[i][j].wall_px = false;
            maze[i][j].wall_nx = false;
            maze[i][j].wall_py = false;
            maze[i][j].wall_ny = false;
        }
    }
}

void copy_walls(MazeCell maze[0][16], MazeCell maze1[][16])
{
    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < m; j++)
        {
            maze1[i][j].wall_nx = maze[i][j].wall_nx;
            maze1[i][j].wall_ny = maze[i][j].wall_ny;
            maze1[i][j].wall_px = maze[i][j].wall_px;
            maze1[i][j].wall_py = maze[i][j].wall_py;
        }
    }
}

#ifndef __arm__
int main()
{
    clear_walls(maze);

    // initialize center goal
    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < m; j++)
        {
            maze[i][j].value = 255;
        }
    }

    maze[7][7].value = 0;
    maze[7][8].value = 0;
    maze[8][7].value = 0;
    maze[8][8].value = 0;

    orientiation = 192; // start facing "up" (your convention)
    x = 0;
    y = 0;
    push(0, 0, 12);

    while (1)
    {
        printf("\nCurrent position: (%d, %d)\n", x, y);

        // Input wall data
        bool adc[4];
        printf("Enter 6 sensor values (0 or 1): ");
        for (int i = 0; i < 4; i++)
        {
            scanf("%d", &adc[i]);
        }

        update_cell(adc, maze);
        update_maze(false, maze);

        // Print maze values (debug)
        printf("\nMaze values:\n");
        for (int i = 15; i >= 0; i--)
        {
            for (int j = 0; j < m; j++)
            {
                printf("%3d ", maze[j][i].value);
            }
            printf("\n");
        }

        XY junction;
        junction.junction = false;
        int count = 0;
        uint8_t move_dir = next_move(false, maze);
        printf("Chosen move: %d\n", move_dir);
        if (move_dir != 3)
        {
            if (count == 0)
                push(x, y,move_dir);
            else
                count--;
        }
        else
        {
            while (!junction.junction)
            {
                junction = pop();
                count++;
                printf("sus while %d top = %d\n", junction.junction, top);
            }
            push(junction.x, junction.y,move_dir);
        }

        move(move_dir);
        if (maze[x][y].value == 0)
        {
            printf("Reached center!\n");
            break;
        }
    }
    clear_walls(maze1);
    copy_walls(maze, maze1);

    // while (1)
    // {
    //     printf("\nCurrent position: (%d, %d)", x, y);

    //     bool adc[6];
    //     printf("Enter 6 sensor values (0 or 1): ");
    //     for (int i = 0; i < 6; i++)
    //     {
    //         scanf("%d", &adc[i]);
    //     }

    //     update_cell(adc, maze);
    //     update_maze(false, maze);

    //     for (int i = 0; i < 16; i++)
    //     {
    //         for (int j = 0; j < 16; j++)
    //         {
    //             printf("%3d", maze[i][j].value);
    //         }
    //         printf("\n");
    //     }
    // }

    while (1)
    {
        printf("\nCurrent position: (%d, %d)\n", x, y);

        // Input wall data
        bool adc[4];
        printf("Enter 6 sensor values (0 or 1): ");
        for (int i = 0; i < 4; i++)
        {
            scanf("%d", &adc[i]);
        }

        update_cell(adc, maze);
        update_maze(false, maze);

        // Print maze values (debug)
        printf("\nMaze values:\n");
        for (int i = 15; i >= 0; i--)
        {
            for (int j = 0; j < m; j++)
            {
                printf("%3d ", maze[j][i].value);
            }
            printf("\n");
        }

        uint8_t move_dir = next_move(true, maze);
        printf("Chosen move: %d\n", move_dir);

        move(move_dir);
        if (x == 0 && y == 0)
        {
            printf("Start Reached!\n");
            break;
        }
    }

    for (int i = 0; i <= top; i++)
    {
        printf("%d, %d\n", stack[i].x, stack[i].y);
    }
    return 0;
    // clear_walls(maze);
    // for (int i = 0; i < m / 2; i++)
    // {
    //     for (int j = 0; j < m; j++)
    //     {
    //         if (i < m / 2 && j < m / 2)
    //         {
    //             maze[i][j].value = m - 2 - i - j;
    //             maze[i][m - 1 - j].value = maze[i][j].value;
    //         }
    //         maze[m - 1 - i][j].value = maze[i][j].value;
    //     }
    // }

    // while (maze[x][y].value != 0)
    // {
    //     // read adc and convert array to bool array
    //     bool adc[6];
    //     update_cell(adc, maze);
    //     update_maze(false, maze);
    //     uint8_t Move = next_move(maze);
    //     move(Move);
    // }

    // // correct the orientiation physically align using wall

    // clear_walls(maze1);
    // //initialize maze[x][y] so that the loop is garunteed to run
    // while (maze1[x][y].value != 0)
    // {
    //     bool adc[6];
    //     update_cell(adc, maze1);
    //     update_maze(true, maze1);
    //     uint8_t Move = next_move(maze1);
    //     move(Move);
    // }
    // // for (int i = 0; i < m; i++) {
    // //     for (int j = 0; j < m; j++) {
    // //         if(maze[i][j].value < 10) {
    // //             printf("%d ",maze[i][j].value);
    // //         }
    // //         else {
    // //             printf("%d ",maze[i][j].value);
    // //         }
    // //     }
    // //     printf("\n");
    // // }
    // return 0;
}
#endif
