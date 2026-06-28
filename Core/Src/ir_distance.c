#include "ir_distance.h"
#include "adc.h"
#include "tim.h" // For microsecond delay if we use TIM5
#include "stdbool.h"

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

static void delay_us(uint32_t us) {
	uint32_t start = __HAL_TIM_GET_COUNTER(&htim5);
	while ((__HAL_TIM_GET_COUNTER(&htim5) - start) < us)
		;
}

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
	// VERY IMPORTANT: To pulse the LED quickly, we use a faster sampling time.
	// 84 cycles is around 3.5 microseconds at 24MHz ADC clock.
	sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;

	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		return 0;
	}
	uint16_t val;
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
	// If we need to calibrate anything or set up filters, do it here
}

void IR_Read(void) {
	// 1. Read Ambient Light (Emitters OFF)
	set_emitters(0);
	// Let photodiode settle
	delay_us(100);

	for (int i = 0; i < IR_SENSOR_COUNT; i++) {
		ir_data.ambient[i] = read_adc_channel(ADC_CHANNELS[i]);
	}

	// 2. Read Active Light (Emitters ON)
	set_emitters(1);
	// High impedance divider + photodiode might take several hundred microseconds to fully respond.
	// Adjust this delay based on testing (100us - 500us is typical).
	delay_us(10000);

	for (int i = 0; i < IR_SENSOR_COUNT; i++) {
		ir_data.active[i] = read_adc_channel(ADC_CHANNELS[i]);

		// 3. Subtract for Rejected value
		// Using int16_t directly in case ambient > active due to noise
		ir_data.value[i] = (int16_t) ir_data.ambient[i]
				- (int16_t) ir_data.active[i];
		if (ir_data.value[i] < 0) {
			ir_data.value[i] = 0; // Clamp to 0
		}

	}

	// 4. Turn off Emitters
	set_emitters(0);
}

void IR_Distance(void) {
	int16_t x;
	for (int i = 0; i < IR_SENSOR_COUNT; i++) {
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
