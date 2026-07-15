#ifndef __NAVIGATION_H
#define __NAVIGATION_H

#include "main.h"

// IR Thresholds and Center Points for Wall Alignment & Balance
#define FRONT_WALL_THRESHOLD  700   // Front wall present if sensors > this
#define LEFT_WALL_THRESHOLD   500   // Left wall present if sensor > this
#define RIGHT_WALL_THRESHOLD  500   // Right wall present if sensor > this

#define LEFT_NOMINAL_CENTER   800   // Reading when centered next to left wall
#define RIGHT_NOMINAL_CENTER  800   // Reading when centered next to right wall
#define KP_IR_BALANCER        1.5f  // Gain for IR straight steering correction
#define KP_FRONT_BALANCER     2.0f  // Gain for front wall alignment steering correction

#define FRONT_ALIGN_TOLERANCE 20    // Target diff between front sensors
#define SIDE_ALIGN_TOLERANCE  100   // Target diff between side sensors when centered

// Initialize navigation
void Navigation_Init(void);

// Fetch target gyro heading from floodfill orientation
float Navigation_GetCardinalAngle(uint8_t orientation);

// Align the mouse using IR distance sensors and reset the gyro yaw to the target cardinal heading
void Navigation_AlignWithWalls(void);

// Execute a step/delay while maintaining the 50Hz sensor & gyro integration updates
void Navigation_DelayOrUpdate(uint32_t ms);

// Execute movement to target cell (turn and move)
void Navigation_MoveToCell(uint8_t relative_dir);

// Solve step
void Navigation_LoopStep(void);

#endif /* __NAVIGATION_H */
