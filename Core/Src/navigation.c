#include "navigation.h"
#include "floodfill.h"
#include "motor_control.h"
#include "mpu6050.h"
#include "ir_distance.h"
#include "status.h"
#include <stdlib.h>
#include <math.h>

// Side wall IR stability history buffers
#define IR_HISTORY_SIZE 5
static int16_t left_ir_history[IR_HISTORY_SIZE] = {0};
static int16_t right_ir_history[IR_HISTORY_SIZE] = {0};
static uint8_t ir_hist_idx = 0;
static uint8_t ir_hist_count = 0;

static void reset_ir_history(void) {
    ir_hist_idx = 0;
    ir_hist_count = 0;
}

void Navigation_Init(void) {
    // 1. Initialize UI and status blinking
    Status_Init();

    // 2. Initialize Gyro/Accelerometer
    if (MPU6050_Init()) {
        Status_SetMessage(STATUS_MPU_DETECTED, 0);
    } else {
        Status_SetMessage(ERROR_I2C, 1);
        while (1); // Trap if MPU6050 initialization fails
    }

    // 3. Calibrate Gyro (Mouse MUST be kept completely still!)
    Status_SetModeLEDs(1, 0, 1); // LED visual cue for calibration
    HAL_Delay(500);
    MPU6050_Calibrate();
    Status_SetMessage(STATUS_MPU_CALIBRATED, 0);
    Status_SetModeLEDs(0, 0, 0);

    // 4. Initialize IR sensors
    IR_Distance_Init();

    // 5. Initialize motor drivers and encoder registers
    Motor_Control_Init();

    reset_ir_history();
}

float Navigation_GetCardinalAngle(uint8_t orientation) {
    switch (orientation) {
        case 192: // North (+y)
            return 0.0f;
        case 48:  // East (+x)
            return -90.0f;
        case 12:  // South (-y)
            return 180.0f;
        case 3:   // West (-x)
            return 90.0f;
        default:
            return 0.0f;
    }
}

void Navigation_DelayOrUpdate(uint32_t ms) {
    uint32_t start = HAL_GetTick();
    static uint32_t last_50hz_tick = 0;

    while (HAL_GetTick() - start < ms) {
        uint32_t now = HAL_GetTick();
        
        // 50Hz Rate Limit (20ms interval)
        if (now - last_50hz_tick >= 20) {
            last_50hz_tick = now;

            // 1. Integrate Gyro calculations
            MPU6050_Update(0.02f);

            // 2. Trigger differential IR sensing scans
            IR_Distance_Read();

            // 3. Continuous alignment check if driving forward/stationary
            if (!Motor_IsTurning()) {
                // Populate circular history buffers
                left_ir_history[ir_hist_idx] = ir_data.value[1];
                right_ir_history[ir_hist_idx] = ir_data.value[4];
                ir_hist_idx = (ir_hist_idx + 1) % IR_HISTORY_SIZE;
                if (ir_hist_count < IR_HISTORY_SIZE) ir_hist_count++;

                // If history buffer is fully populated, analyze stability
                if (ir_hist_count == IR_HISTORY_SIZE) {
                    int16_t left_min = 4095, left_max = 0;
                    int16_t right_min = 4095, right_max = 0;

                    for (int i = 0; i < IR_HISTORY_SIZE; i++) {
                        if (left_ir_history[i] < left_min) left_min = left_ir_history[i];
                        if (left_ir_history[i] > left_max) left_max = left_ir_history[i];
                        if (right_ir_history[i] < right_min) right_min = right_ir_history[i];
                        if (right_ir_history[i] > right_max) right_max = right_ir_history[i];
                    }

                    #define STABILITY_THRESHOLD 20
                    uint8_t left_stable = (left_max - left_min) < STABILITY_THRESHOLD;
                    uint8_t right_stable = (right_max - right_min) < STABILITY_THRESHOLD;

                    uint8_t left_wall = ir_data.value[1] > LEFT_WALL_THRESHOLD;
                    uint8_t right_wall = ir_data.value[4] > RIGHT_WALL_THRESHOLD;

                    uint8_t is_aligned = 0;
                    if (left_wall && right_wall && left_stable && right_stable) {
                        int16_t diff = ir_data.value[1] - ir_data.value[4];
                        if (abs(diff) < SIDE_ALIGN_TOLERANCE) {
                            is_aligned = 1;
                        }
                    } else if (left_wall && !right_wall && left_stable) {
                        is_aligned = 1;
                    } else if (right_wall && !left_wall && right_stable) {
                        is_aligned = 1;
                    }

                    // Reset Integrated Gyro Yaw if we are verified to be dead-straight
                    if (is_aligned) {
                        float target_angle = Navigation_GetCardinalAngle(orientiation);
                        mpu.yaw_angle = target_angle;
                    }
                }
            } else {
                // Clear side-sensor readings history during active turns to prevent stale integration corrections
                reset_ir_history();
            }
        }
        HAL_Delay(1);
    }
}

void Navigation_AlignWithWalls(void) {
    // 1. Fetch target cardinal angle corresponding to floodfill orientation
    float target_angle = Navigation_GetCardinalAngle(orientiation);

    // 2. Read physical IR sensors
    Navigation_DelayOrUpdate(20);

    // Case A: Front wall is present (both front sensors see a wall)
    if (ir_data.value[2] > FRONT_WALL_THRESHOLD && ir_data.value[3] > FRONT_WALL_THRESHOLD) {
        // Stop motors first
        Motor_Control_Stop();
        Navigation_DelayOrUpdate(100);

        uint32_t start_time = HAL_GetTick();
        
        // Active Proportional Alignment Loop (up to 800ms)
        while (HAL_GetTick() - start_time < 800) {
            Navigation_DelayOrUpdate(20); // 50Hz sensor updates
            
            // Difference: left_front - right_front
            int16_t diff = ir_data.value[2] - ir_data.value[3];
            
            if (abs(diff) < FRONT_ALIGN_TOLERANCE) {
                break;
            }

            // Simple P-regulator for alignment
            int32_t align_pwm = (int32_t)(diff * 2.5f);
            
            // Limit alignment speed
            if (align_pwm > 1500) align_pwm = 1500;
            if (align_pwm < -1500) align_pwm = -1500;

            Motor_SetPWM_Right(align_pwm);
            Motor_SetPWM_Left(-align_pwm);
        }

        // Stop motors and settle
        Motor_Control_Stop();
        Navigation_DelayOrUpdate(100);

        // Gyro is now aligned with the front wall. Reset it to target cardinal angle!
        mpu.yaw_angle = target_angle;
        
        // Clear history to avoid mixing in pre-aligned values
        reset_ir_history();
    }
}

void Navigation_MoveToCell(uint8_t relative_dir) {
    float current_cardinal = Navigation_GetCardinalAngle(orientiation);
    float target_heading = current_cardinal;

    // 1. Calculate new heading based on relative move direction
    if (relative_dir == 192) {        // Turn Right
        target_heading -= 90.0f;
    } else if (relative_dir == 48) {  // Turn Left
        target_heading += 90.0f;
    } else if (relative_dir == 3) {   // Turn Around
        target_heading += 180.0f;
    }

    // Normalize target heading to [-180, 180)
    while (target_heading >= 180.0f) target_heading -= 360.0f;
    while (target_heading < -180.0f) target_heading += 360.0f;

    // Clear history buffer before turning
    reset_ir_history();

    // 2. Perform turn if needed
    if (relative_dir != 12) {
        Motor_Turn_To_Heading(target_heading);
        // Wait until turn completes
        while (!Motor_IsMovementComplete()) {
            Navigation_DelayOrUpdate(5);
        }
        Navigation_DelayOrUpdate(50); // Settle time
    }

    // 3. Move forward 18.0cm (standard cell size) while maintaining heading
    Motor_Move_Cm_Gyro(18.0f, target_heading);
    // Wait until forward movement completes
    while (!Motor_IsMovementComplete()) {
        Navigation_DelayOrUpdate(5);
    }
    Navigation_DelayOrUpdate(50); // Settle time

    // 4. Update the floodfill logic coordinates
    move(relative_dir);

    // 5. Run active alignment to clear gyro drift
    Navigation_AlignWithWalls();
}

void Navigation_LoopStep(void) {
    // 1. Refresh sensors
    Navigation_DelayOrUpdate(20);

    bool adc[6] = {false};
    adc[1] = ir_data.value[1] > LEFT_WALL_THRESHOLD;
    adc[2] = (ir_data.value[2] > FRONT_WALL_THRESHOLD) || (ir_data.value[3] > FRONT_WALL_THRESHOLD);
    adc[4] = ir_data.value[4] > RIGHT_WALL_THRESHOLD;

    // 2. Update cell wall states and compute floodfill distances
    update_cell(adc, maze);
    update_maze(false, maze);

    // 3. Determine the next move direction
    uint8_t move_dir = next_move(false, maze);

    // 4. Execute physical and logical movement to the cell
    Navigation_MoveToCell(move_dir);
}
