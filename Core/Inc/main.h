/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "led.h"
#include "clock.h"
#ifndef USB_VCP
#define USB_VCP
#endif
#ifndef SELF_BOOT_DFU
#define SELF_BOOT_DFU
#endif
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
#define SIN_P_Pin GPIO_PIN_0
#define SIN_P_GPIO_Port GPIOC
#define SIN_N_Pin GPIO_PIN_1
#define SIN_N_GPIO_Port GPIOC
#define COS_P_Pin GPIO_PIN_2
#define COS_P_GPIO_Port GPIOC
#define COS_N_Pin GPIO_PIN_3
#define COS_N_GPIO_Port GPIOC
#define LED_R_Pin GPIO_PIN_0
#define LED_R_GPIO_Port GPIOA
#define LED_G_Pin GPIO_PIN_1
#define LED_G_GPIO_Port GPIOA
#define LED_B_Pin GPIO_PIN_2
#define LED_B_GPIO_Port GPIOA
#define STEER_VGMR_Pin GPIO_PIN_3
#define STEER_VGMR_GPIO_Port GPIOA
#define BSPD_Brake_Analog_Pin GPIO_PIN_4
#define BSPD_Brake_Analog_GPIO_Port GPIOA
#define BSE1_fused_Pin GPIO_PIN_5
#define BSE1_fused_GPIO_Port GPIOA
#define BSE2_fused_Pin GPIO_PIN_6
#define BSE2_fused_GPIO_Port GPIOA
#define APPS1_0_Pin GPIO_PIN_7
#define APPS1_0_GPIO_Port GPIOA
#define APPS2_0_Pin GPIO_PIN_4
#define APPS2_0_GPIO_Port GPIOC
#define BPPS1_0_Pin GPIO_PIN_5
#define BPPS1_0_GPIO_Port GPIOC
#define BPPS2_O_Pin GPIO_PIN_0
#define BPPS2_O_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
