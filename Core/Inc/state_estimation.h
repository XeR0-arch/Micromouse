#ifndef __STATE_ESTIMATION_H
#define __STATE_ESTIMATION_H

#include <stdint.h>

// State vector structure for the Kalman Filter
typedef struct {
    float x;      // X position in cm (Absolute map coordinates)
    float y;      // Y position in cm (Absolute map coordinates)
    float theta;  // Heading in degrees (0 = North, 90 = East, 180 = South, -90 = West)
    
    // Covariance matrix diagonal (uncertainty)
    float p_x;
    float p_y;
    float p_theta;
} StateVector;

// Initialize the state estimator (reset to 0,0,0)
void StateEstimation_Init(void);

// Prediction Step: Call this at a fixed rate (e.g. 50Hz, dt=0.02)
// Uses encoder deltas and gyro yaw rate to predict the next pose.
void StateEstimation_PredictStep(float dt);

// Update Step: Call this when new IR distance measurements are available
// Uses side walls to correct Y drift (or X drift depending on heading) and Theta drift.
void StateEstimation_UpdateIR(void);

// Collision Detection: Monitors Accelerometer for sudden spikes
// Returns 1 if a collision is detected, 0 otherwise.
uint8_t StateEstimation_CheckCollision(void);

// Accessors
void StateEstimation_GetPose(float *x, float *y, float *theta);
void StateEstimation_SetPose(float x, float y, float theta);

#endif /* __STATE_ESTIMATION_H */
