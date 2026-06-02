/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SYS_CRITICAL_SIG_FAULT_Pin GPIO_PIN_13
#define SYS_CRITICAL_SIG_FAULT_GPIO_Port GPIOC
#define RESERVED_FAULT_Pin GPIO_PIN_14
#define RESERVED_FAULT_GPIO_Port GPIOC
#define PACK_OVER_CURRENT_FAULT_Pin GPIO_PIN_15
#define PACK_OVER_CURRENT_FAULT_GPIO_Port GPIOC
#define PRECH_OVER_TEMP_FAULT_Pin GPIO_PIN_0
#define PRECH_OVER_TEMP_FAULT_GPIO_Port GPIOC
#define PRECH_WELD_DISCH_FAULT_Pin GPIO_PIN_1
#define PRECH_WELD_DISCH_FAULT_GPIO_Port GPIOC
#define PRECH_OPEN_CIRCUIT_FAULT_Pin GPIO_PIN_2
#define PRECH_OPEN_CIRCUIT_FAULT_GPIO_Port GPIOC
#define CELL_OVER_TEMP_FAULT_Pin GPIO_PIN_3
#define CELL_OVER_TEMP_FAULT_GPIO_Port GPIOC
#define CHARGER_FAULT_Pin GPIO_PIN_0
#define CHARGER_FAULT_GPIO_Port GPIOA
#define BSPD_300_Pin GPIO_PIN_1
#define BSPD_300_GPIO_Port GPIOA
#define BSPD_30_Pin GPIO_PIN_2
#define BSPD_30_GPIO_Port GPIOA
#define isoSPI_CS_Pin GPIO_PIN_3
#define isoSPI_CS_GPIO_Port GPIOA
#define HV_ADC_NCS_Pin GPIO_PIN_4
#define HV_ADC_NCS_GPIO_Port GPIOA
#define HV_ADC_NINT_Pin GPIO_PIN_4
#define HV_ADC_NINT_GPIO_Port GPIOC
#define MCU_PRECHARGE_Pin GPIO_PIN_5
#define MCU_PRECHARGE_GPIO_Port GPIOC
#define MCU_AIR_N_Pin GPIO_PIN_0
#define MCU_AIR_N_GPIO_Port GPIOB
#define MCU_AIR_P_Pin GPIO_PIN_1
#define MCU_AIR_P_GPIO_Port GPIOB
#define AIR_N_MECH_STATE_Pin GPIO_PIN_6
#define AIR_N_MECH_STATE_GPIO_Port GPIOC
#define AIR_N_MECH_STATE_EXTI_IRQn EXTI9_5_IRQn
#define AIR_P_MECH_STATE_Pin GPIO_PIN_7
#define AIR_P_MECH_STATE_GPIO_Port GPIOC
#define AIR_P_MECH_STATE_EXTI_IRQn EXTI9_5_IRQn
#define LED_GREEN_Pin GPIO_PIN_8
#define LED_GREEN_GPIO_Port GPIOC
#define LED_BLUE_Pin GPIO_PIN_8
#define LED_BLUE_GPIO_Port GPIOA
#define LED_YELLOW_Pin GPIO_PIN_9
#define LED_YELLOW_GPIO_Port GPIOA
#define BMS_Re_MECH_STATE_Pin GPIO_PIN_10
#define BMS_Re_MECH_STATE_GPIO_Port GPIOA
#define BMS_Re_MECH_STATE_EXTI_IRQn EXTI15_10_IRQn
#define FAN_PWM_Pin GPIO_PIN_11
#define FAN_PWM_GPIO_Port GPIOA
#define PRECH_MECH_STATE_Pin GPIO_PIN_12
#define PRECH_MECH_STATE_GPIO_Port GPIOA
#define PRECH_MECH_STATE_EXTI_IRQn EXTI15_10_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
