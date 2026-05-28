#include "ir_distance.h"
#include "adc.h"
#include "tim.h" 

// NOTE: Ensure your ir_distance.h has `uint8_t binary_walls[6];` inside `IR_Data_t`
IR_Data_t ir_data = {0};

// Threshold to determine if a wall is present for the floodfill binary array
// You will need to tune this based on your actual ADC values when a wall is present
#define WALL_THRESHOLD 1500 

// Map logical sensor indices (0 = Left-most ... 5 = Right-most)
static const uint32_t ADC_CHANNELS[IR_SENSOR_COUNT] = {
    ADC_CHANNEL_2, // PA2 - Left-most
    ADC_CHANNEL_3, // PA3
    ADC_CHANNEL_4, // PA4
    ADC_CHANNEL_5, // PA5
    ADC_CHANNEL_6, // PA6
    ADC_CHANNEL_7  // PA7 - Right-most
};

// Map logical sensor indices to Emitter Pins
static const uint16_t EMITTER_PINS[IR_SENSOR_COUNT] = {
    GPIO_PIN_2,  // PB2 - Left-most
    GPIO_PIN_10, // PB10
    GPIO_PIN_12, // PB12
    GPIO_PIN_13, // PB13
    GPIO_PIN_14, // PB14
    GPIO_PIN_15  // PB15 - Right-most
};

static void delay_us(uint32_t us) {
    uint32_t start = __HAL_TIM_GET_COUNTER(&htim5);
    while ((__HAL_TIM_GET_COUNTER(&htim5) - start) < us);
}

// Helper to read a single ADC channel by reconfiguring the sequencer
static uint16_t read_adc_channel(uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES; 
    
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        return 0;
    }

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint16_t val = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return val;
}

void IR_Distance_Init(void) {
    // Ensure all emitters start off
    for(int i=0; i<IR_SENSOR_COUNT; i++) {
        HAL_GPIO_WritePin(GPIOB, EMITTER_PINS[i], GPIO_PIN_RESET);
    }
}

void IR_Distance_Read(void) {
    // Read sequentially from left-most (0) to right-most (5) in a clockwise logical direction
    for(int i=0; i<IR_SENSOR_COUNT; i++) {
        
        // 1. Read Ambient Light (Emitter OFF)
        HAL_GPIO_WritePin(GPIOB, EMITTER_PINS[i], GPIO_PIN_RESET);
        delay_us(100); // Photodiode settling time
        ir_data.ambient[i] = read_adc_channel(ADC_CHANNELS[i]);

        // 2. Read Active Light (Emitter ON)
        HAL_GPIO_WritePin(GPIOB, EMITTER_PINS[i], GPIO_PIN_SET);
        delay_us(250); // Give IR LED time to turn on and photodiode to react
        ir_data.active[i] = read_adc_channel(ADC_CHANNELS[i]);

        // 3. Immediately turn OFF to save power and prevent cross-talk with the next sensor
        HAL_GPIO_WritePin(GPIOB, EMITTER_PINS[i], GPIO_PIN_RESET);

        // 4. Calculate actual value (clamp to 0 if ambient > active due to noise)
        ir_data.value[i] = (int16_t)ir_data.active[i] - (int16_t)ir_data.ambient[i];
        if (ir_data.value[i] < 0) {
            ir_data.value[i] = 0; 
        }

        // 5. Convert to Binary array for Floodfill algorithm
        if (ir_data.value[i] > WALL_THRESHOLD) {
            ir_data.binary_walls[i] = 1; // Wall present
        } else {
            ir_data.binary_walls[i] = 0; // No wall
        }
    }
}