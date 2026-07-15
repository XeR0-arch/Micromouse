/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdio.h"
#include "string.h"
#include "ir_distance.h"
#include "motor_control.h"
#include "mpu6050.h"
#include "status.h"
#include "uart.h"
#include "navigation.h"
#include "floodfill.h"

// === MPU6050 I2C HELPERS ===
extern I2C_HandleTypeDef hi2c1;   // or hi2c2, hi2c3 — use whatever CubeMX generated

#define MPU6050_ADDR    (0x68 << 1)   // 0xD0 for I2C write, 0xD1 for read

uint8_t Gyro_ReadReg(uint8_t reg)
{
    uint8_t val = 0;
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, reg, 1, &val, 1, 100);
    return val;
}

void Gyro_WriteReg(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg, 1, &data, 1, 100);
}
// ===========================
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM4)
	{
		Motor_Control_Update();
	}
	else if (htim->Instance == TIM10)
	{
		/* TIM10 only sets a flag. I2C runs in MPU6050_Service() in main. */
		MPU6050_ScheduleUpdate();
	}
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Motion_Delay(uint32_t ms);
static void Wait_Motion_Complete(uint32_t timeout_ms);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Keep foreground alive so scheduled MPU samples get serviced. */
static void Motion_Delay(uint32_t ms)
{
	uint32_t start = HAL_GetTick();
	while ((HAL_GetTick() - start) < ms)
	{
		MPU6050_Service();
		HAL_Delay(1);
	}
}

/* Wait for encoder move/turn to finish. Timeout stops a stuck PID. */
static void Wait_Motion_Complete(uint32_t timeout_ms)
{
	uint32_t start = HAL_GetTick();
	while (!Motor_IsMovementComplete())
	{
		MPU6050_Service();
		HAL_Delay(1);
		if ((HAL_GetTick() - start) > timeout_ms)
		{
			Motor_Control_Stop();
			break;
		}
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/
	HAL_Init();

	/* USER CODE BEGIN Init */
	/* USER CODE END Init */

	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_I2C1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM5_Init();
	MX_USART1_UART_Init();
	MX_TIM4_Init();
	MX_TIM1_Init();
	MX_TIM10_Init();

	/* USER CODE BEGIN 2 */
	Motor_Control_Init();
	IR_Distance_Init();

	/* Init + calibrate MPU BEFORE starting TIM10 scheduler. */
	MPU6050_Init();
	MPU6050_Calibrate();
	HAL_TIM_Base_Start_IT(&htim10); /* ~40 Hz schedule */

	/* Time to set the mouse down after flash/reset. Stay still for calib. */
	Motion_Delay(2000);

	char msg[128];
	sprintf(msg, "ready heading=%.2f\r\n", Motor_Get_Heading());
	UART_Print(msg);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	/*
	 * TURN DIAGNOSTIC
	 * ---------------
	 * One +90° MPU turn per cycle + UART heading report.
	 *
	 * Expect delta ~ +90 each time (heading ~90, 180, -90, 0, ...)
	 * If delta ~ +270 or ~ -90: flip MPU_YAW_SIGN in motor_control.c
	 * If bare encoder turn is already wrong size: retune WHEELBASE_CM /
	 *   ENCODER_COUNTS_PER_MOTOR_REV
	 */
	// Wake up MPU6050
	// Wake up MPU6050
	// Wake up MPU6050
	// In main(), BEFORE while(1):
	// === BEFORE while(1) ===
	if (!MPU6050_Init()) {
	    UART_Print("MPU6050 FAIL\r\n");
	    Error_Handler();
	}

	Motor_Control_Init();

	// Reset heading to 0 at start
	mpu.yaw_angle = 0.0f;

	while (1)
	{
	    Motor_Drive_Square(-40.0f);  // 20cm square
	    HAL_Delay(1000);            // Pause between squares
	}
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 12;
	RCC_OscInitStruct.PLL.PLLN = 96;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	(void)file;
	(void)line;
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
