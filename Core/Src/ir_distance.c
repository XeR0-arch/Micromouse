#include "ir_distance.h"
#include "adc.h"
#include "tim.h" // For microsecond delay if we use TIM5
#include "motor_control.h"
#include "stdbool.h"
#include "math.h"

IR_Data_t ir_data = { 0 };

// Map logical sensor indices to ADC Channels
static const uint32_t ADC_CHANNELS[IR_SENSOR_COUNT] = {
ADC_CHANNEL_2, // PA2
		ADC_CHANNEL_3, // PA3
		ADC_CHANNEL_4, // PA4
		ADC_CHANNEL_5, // PA5
//    ADC_CHANNEL_6, // PA6
//    ADC_CHANNEL_7  // PA7
		};

// Map logical sensor indices to Emitter Pins
static const uint16_t EMITTER_PINS[IR_SENSOR_COUNT] = {
GPIO_PIN_2,  // PB2
//    GPIO_PIN_10, // PB10
//    GPIO_PIN_12, // PB12
//    GPIO_PIN_13, // PB13
//    GPIO_PIN_14, // PB14
//    GPIO_PIN_15  // PB15
		};

// Helper to set all emitters
static void set_emitters(uint8_t state) {
	GPIO_PinState pin_state = state ? GPIO_PIN_SET : GPIO_PIN_RESET;
	for (int i = 0; i < IR_SENSOR_COUNT; i++) {
		HAL_GPIO_WritePin(GPIOB, EMITTER_PINS[i], pin_state);
	}
}

// Helper to read a single ADC channel by reconfiguring the sequencer
static uint16_t read_adc_channel(uint32_t channel) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	sConfig.Channel = channel;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;

	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		return 0;
	}
	uint16_t val = 0;
	HAL_ADC_Start(&hadc1);
	if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
		val = HAL_ADC_GetValue(&hadc1);
	} else {
		__HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR);
	}

	HAL_ADC_Stop(&hadc1);
	return val;
}

void IR_Distance_Init(void) {
	set_emitters(0); // Ensure emitters are off
	HAL_Delay(1);
}

// ============ NON-BLOCKING STATE MACHINE ============
// Called at 40Hz from TIM10 ISR. Two states, 25ms apart:
//
//   State 0: Emitters have been OFF for 25ms (plenty of settle time)
//            -> Read ambient on all channels
//            -> Turn ON emitters (they charge up over the next 25ms)
//
//   State 1: Emitters have been ON for 25ms (plenty of charge time)
//            -> Read active on all channels
//            -> Compute ambient-rejected values
//            -> Turn OFF emitters
//            -> Compute distances + steering error
//
// Effective distance update rate: 20Hz (one full cycle every 2 ticks)
// ISR duration: ~200us (just 4 ADC reads), NOT 10ms like before
//
static uint8_t ir_state = 0;

void IR_Tick(void) {
	if (ir_state == 0) {
		// --- Emitters have been OFF for 25ms, read AMBIENT ---
		for (int i = 0; i < IR_SENSOR_COUNT; i++) {
			ir_data.ambient[i] = read_adc_channel(ADC_CHANNELS[i]);
		}
		// Turn ON emitters — they'll charge for 25ms until next tick
		set_emitters(1);
		ir_state = 1;

	} else {
		// --- Emitters have been ON for 25ms, read ACTIVE ---
		for (int i = 0; i < IR_SENSOR_COUNT; i++) {
			ir_data.active[i] = read_adc_channel(ADC_CHANNELS[i]);

			// Ambient rejection: ambient - active
			ir_data.value[i] = (int16_t)ir_data.ambient[i]
					- (int16_t)ir_data.active[i];
			if (ir_data.value[i] < 0) {
				ir_data.value[i] = 0;
			}
		}
		// Turn OFF emitters — they'll settle for 25ms until next tick
		set_emitters(0);

		// Fresh data ready — compute distances
		IR_Distance();

		ir_state = 0;
	}
}

// Global steering error for the motor PID to consume
// Positive = closer to left wall, negative = closer to right wall
int32_t ir_steering_error = 0;

void IR_Distance(void) {
	int16_t x;
	for (int i = 0; i < IR_SENSOR_COUNT; i++) {
		if (ir_data.value[i] <= 0) {
			ir_data.distance[i] = 999; // No reading, assume far away
			continue;
		}
		x = log10(ir_data.value[i]);
		switch (i) {
		case 0:
			ir_data.distance[i] = 157.5391 - 83.42516 * x + 11.29101 * x * x;
			break;
		case 1:
			ir_data.distance[i] = 146.2968 - 73.18066 * x + 9.34274 * x * x;
			break;
		case 2:
			ir_data.distance[i] = 182.5472 - 97.43766 * x + 13.28011 * x * x;
			break;
		case 3:
			ir_data.distance[i] = 157.5391 - 83.42516 * x + 11.29101 * x * x;
			break;
		default:
			break;
		}
	}

	// Compute steering error: left sensor(0) vs right sensor(3)
	// Positive error = closer to left wall = need to steer right
	ir_steering_error = ir_data.distance[0] - ir_data.distance[3];
}
void GET_Walls(void) {
	if (ir_data.distance[3] > 16) {
		ir_data.walls[5] = false;
		ir_data.walls[4] = false;
	}
	if (ir_data.distance[2] > 16 && ir_data.distance[1] > 16) {
		ir_data.walls[3] = false;
		ir_data.walls[2] = false;
	}
	if (ir_data.distance[0] > 16) {
		ir_data.walls[1] = false;
		ir_data.walls[0] = false;
	}
}
