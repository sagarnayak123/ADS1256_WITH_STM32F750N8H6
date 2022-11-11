/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f7xx_hal.h"

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
#define ADC2_CSN_Pin GPIO_PIN_4
#define ADC2_CSN_GPIO_Port GPIOE
#define ADC2_RST_Pin GPIO_PIN_3
#define ADC2_RST_GPIO_Port GPIOE
#define BTN_IN_2_Pin GPIO_PIN_11
#define BTN_IN_2_GPIO_Port GPIOG
#define RLY_9_Pin GPIO_PIN_8
#define RLY_9_GPIO_Port GPIOI
#define RLY_5_Pin GPIO_PIN_4
#define RLY_5_GPIO_Port GPIOI
#define BTN_IN_3_Pin GPIO_PIN_12
#define BTN_IN_3_GPIO_Port GPIOG
#define BTN_IN_1_Pin GPIO_PIN_10
#define BTN_IN_1_GPIO_Port GPIOG
#define RLY_4_Pin GPIO_PIN_3
#define RLY_4_GPIO_Port GPIOI
#define RLY_3_Pin GPIO_PIN_2
#define RLY_3_GPIO_Port GPIOI
#define RLY_6_Pin GPIO_PIN_5
#define RLY_6_GPIO_Port GPIOI
#define RLY_8_Pin GPIO_PIN_7
#define RLY_8_GPIO_Port GPIOI
#define RLY_7_Pin GPIO_PIN_6
#define RLY_7_GPIO_Port GPIOI
#define RLY_2_Pin GPIO_PIN_1
#define RLY_2_GPIO_Port GPIOI
#define RLY_10_Pin GPIO_PIN_9
#define RLY_10_GPIO_Port GPIOI
#define RLY_1_Pin GPIO_PIN_0
#define RLY_1_GPIO_Port GPIOI
#define ADC2_DRDY_Pin GPIO_PIN_9
#define ADC2_DRDY_GPIO_Port GPIOC
#define ADC2_DRDY_EXTI_IRQn EXTI9_5_IRQn
#define CSN_Pin GPIO_PIN_12
#define CSN_GPIO_Port GPIOB
#define DRDY_Pin GPIO_PIN_12
#define DRDY_GPIO_Port GPIOD
#define DRDY_EXTI_IRQn EXTI15_10_IRQn
#define RST_Pin GPIO_PIN_11
#define RST_GPIO_Port GPIOD
#define BTN_IN_5_Pin GPIO_PIN_7
#define BTN_IN_5_GPIO_Port GPIOH
#define BTN_IN_7_Pin GPIO_PIN_9
#define BTN_IN_7_GPIO_Port GPIOH
#define BTN_IN_4_Pin GPIO_PIN_6
#define BTN_IN_4_GPIO_Port GPIOH
#define BTN_IN_6_Pin GPIO_PIN_8
#define BTN_IN_6_GPIO_Port GPIOH
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
