#include "state_estimation.h"
#include "motor_control.h"
#include "mpu6050.h"
#include "ir_distance.h"
#include <math.h>

#define PI 3.1415926535f
#define DEG2RAD(x) ((x) * PI / 180.0f)
#define RAD2DEG(x) ((x) * 180.0f / PI)

// Wheel dimensions
#define WHEELBASE_CM 7.8f
#define WHEEL_CIRCUMFERENCE_CM 9.927f
#define COUNTS_PER_CM 94.904f

extern int32_t get_encoder_left(void);
extern int32_t get_encoder_right(void);

static StateVector state;
static int32_t last_enc_l = 0;
static int32_t last_enc_r = 0;

void StateEstimation_Init(void) {
    state.x = 0.0f;
    state.y = 0.0f;
    state.theta = 0.0f; 
    
    state.p_x = 0.1f;
    state.p_y = 0.1f;
    state.p_theta = 0.1f;

    last_enc_l = get_encoder_left();
    last_enc_r = get_encoder_right();
}

void StateEstimation_PredictStep(float dt) {
    // 1. Read Encoders
    int32_t enc_l = get_encoder_left();
    int32_t enc_r = get_encoder_right();
    
    float dist_l = (enc_l - last_enc_l) / COUNTS_PER_CM;
    float dist_r = (enc_r - last_enc_r) / COUNTS_PER_CM;
    
    last_enc_l = enc_l;
    last_enc_r = enc_r;
    
    float d_dist = (dist_l + dist_r) / 2.0f;
    
    // 2. Read Gyro for Theta
    // MPU updates mpu.gz_vel in degrees per second
    float d_theta_gyro = mpu.gz_vel * dt;
    
    // Update Theta
    state.theta += d_theta_gyro;
    
    // Wrap Theta to [-180, 180]
    while (state.theta > 180.0f) state.theta -= 360.0f;
    while (state.theta < -180.0f) state.theta += 360.0f;
    
    // Update X, Y
    // Assuming 0 is North (+y), East is +x
    float theta_rad = DEG2RAD(state.theta);
    state.x += d_dist * sinf(theta_rad);  
    state.y += d_dist * cosf(theta_rad);  
    
    // Increase uncertainty (Process Noise)
    state.p_x += 0.05f;
    state.p_y += 0.05f;
    state.p_theta += 0.01f;
}

void StateEstimation_UpdateIR(void) {
    // Phase 1 stub: Kalman Filter Update step using IR walls.
    // The exact IR distance mapping equation (ADC to mm) is required to properly
    // update the Kalman gain (K) and state error. 
    // This will be integrated into the navigation loop after basic movement is verified.
}

uint8_t StateEstimation_CheckCollision(void) {
    // mpu.ax and mpu.ay are accelerations in 'g'
    // A sudden deceleration spike -> collision
    float accel_mag = sqrtf(mpu.ax * mpu.ax + mpu.ay * mpu.ay);
    
    // If magnitude > 1.5g (excluding gravity which is on Z axis ideally)
    // Note: If sensor is mounted flat, Z is ~1.0g, X/Y are ~0.0g
    if (accel_mag > 1.5f) {
        return 1; // Collision detected
    }
    return 0;
}

void StateEstimation_GetPose(float *x, float *y, float *theta) {
    if (x) *x = state.x;
    if (y) *y = state.y;
    if (theta) *theta = state.theta;
}

void StateEstimation_SetPose(float x, float y, float theta) {
    state.x = x;
    state.y = y;
    state.theta = theta;
    state.p_x = 0.1f;
    state.p_y = 0.1f;
    state.p_theta = 0.1f;
}
