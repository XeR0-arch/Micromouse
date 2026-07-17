/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define on_board_led_Pin GPIO_PIN_13
#define on_board_led_GPIO_Port GPIOC
#define left_motor_yellow_Pin GPIO_PIN_0
#define left_motor_yellow_GPIO_Port GPIOA
#define left_motor_green_Pin GPIO_PIN_1
#define left_motor_green_GPIO_Port GPIOA
#define ir_in_1_Pin GPIO_PIN_2
#define ir_in_1_GPIO_Port GPIOA
#define ir_in_2_Pin GPIO_PIN_3
#define ir_in_2_GPIO_Port GPIOA
#define ir_in_3_Pin GPIO_PIN_4
#define ir_in_3_GPIO_Port GPIOA
#define ir_in_4_Pin GPIO_PIN_5
#define ir_in_4_GPIO_Port GPIOA
#define For_future_use_Pin GPIO_PIN_6
#define For_future_use_GPIO_Port GPIOA
#define For_future_useA7_Pin GPIO_PIN_7
#define For_future_useA7_GPIO_Port GPIOA
#define left_motor_m2_Pin GPIO_PIN_0
#define left_motor_m2_GPIO_Port GPIOB
#define left_motor_m1_Pin GPIO_PIN_1
#define left_motor_m1_GPIO_Port GPIOB
#define ir_led_1_Pin GPIO_PIN_2
#define ir_led_1_GPIO_Port GPIOB
#define For_Future_Use_Pin GPIO_PIN_10
#define For_Future_Use_GPIO_Port GPIOB
#define For_future_useB12_Pin GPIO_PIN_12
#define For_future_useB12_GPIO_Port GPIOB
#define for_future_use_Pin GPIO_PIN_14
#define for_future_use_GPIO_Port GPIOB
#define for_future_useB15_Pin GPIO_PIN_15
#define for_future_useB15_GPIO_Port GPIOB
#define right_motor_m1_Pin GPIO_PIN_8
#define right_motor_m1_GPIO_Port GPIOA
#define right_motor_m2_Pin GPIO_PIN_11
#define right_motor_m2_GPIO_Port GPIOA
#define led_3_Pin GPIO_PIN_12
#define led_3_GPIO_Port GPIOA
#define right_motor_yellow_Pin GPIO_PIN_15
#define right_motor_yellow_GPIO_Port GPIOA
#define right_motor_green_Pin GPIO_PIN_3
#define right_motor_green_GPIO_Port GPIOB
#define button_1_Pin GPIO_PIN_8
#define button_1_GPIO_Port GPIOB
#define button_2_Pin GPIO_PIN_9
#define button_2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
