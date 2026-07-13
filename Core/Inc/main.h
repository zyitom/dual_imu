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
#include "stm32h7xx_hal.h"

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
#define ICM45686_1_ACS_Pin GPIO_PIN_13
#define ICM45686_1_ACS_GPIO_Port GPIOC
#define ICM45686_1_SCK_Pin GPIO_PIN_2
#define ICM45686_1_SCK_GPIO_Port GPIOE
#define Baro_I2C1_SDA_Pin GPIO_PIN_7
#define Baro_I2C1_SDA_GPIO_Port GPIOB
#define EX_SPI3_MISO_Pin GPIO_PIN_4
#define EX_SPI3_MISO_GPIO_Port GPIOB
#define EX_SPI3_SCK_Pin GPIO_PIN_3
#define EX_SPI3_SCK_GPIO_Port GPIOB
#define USB_Detect_Pin GPIO_PIN_15
#define USB_Detect_GPIO_Port GPIOA
#define ICM45686_1_GDR_Pin GPIO_PIN_3
#define ICM45686_1_GDR_GPIO_Port GPIOE
#define ICM45686_1_GDR_EXTI_IRQn EXTI3_IRQn
#define Baro_I2C1_SCL_Pin GPIO_PIN_6
#define Baro_I2C1_SCL_GPIO_Port GPIOB
#define ICM45686_1_ADR_Pin GPIO_PIN_4
#define ICM45686_1_ADR_GPIO_Port GPIOE
#define ICM45686_1_ADR_EXTI_IRQn EXTI4_IRQn
#define Baro_I2C1_DR_Pin GPIO_PIN_5
#define Baro_I2C1_DR_GPIO_Port GPIOB
#define FRAM_SPI2_SCK_Pin GPIO_PIN_3
#define FRAM_SPI2_SCK_GPIO_Port GPIOD
#define ICM45686_1_MISO_Pin GPIO_PIN_5
#define ICM45686_1_MISO_GPIO_Port GPIOE
#define ICM45686_1_GCS_Pin GPIO_PIN_2
#define ICM45686_1_GCS_GPIO_Port GPIOC
#define ICM45686_1_MOSI_Pin GPIO_PIN_6
#define ICM45686_1_MOSI_GPIO_Port GPIOE
#define AUX_PC0_Pin GPIO_PIN_0
#define AUX_PC0_GPIO_Port GPIOC
#define AUX_PC1_Pin GPIO_PIN_1
#define AUX_PC1_GPIO_Port GPIOC
#define FRAM_SPI2_MOSI_Pin GPIO_PIN_3
#define FRAM_SPI2_MOSI_GPIO_Port GPIOC
#define BMI088_0_ADR_Pin GPIO_PIN_0
#define BMI088_0_ADR_GPIO_Port GPIOA
#define BMI088_0_ADR_EXTI_IRQn EXTI0_IRQn
#define AUX_PA4_Pin GPIO_PIN_4
#define AUX_PA4_GPIO_Port GPIOA
#define ADC1_INP4_BATT_V_Pin GPIO_PIN_4
#define ADC1_INP4_BATT_V_GPIO_Port GPIOC
#define EX_SPI3_MOSI_Pin GPIO_PIN_2
#define EX_SPI3_MOSI_GPIO_Port GPIOB
#define BM0_PWM_Pin GPIO_PIN_15
#define BM0_PWM_GPIO_Port GPIOD
#define LED_GREEN_Pin GPIO_PIN_11
#define LED_GREEN_GPIO_Port GPIOD
#define LED_BLUE_Pin GPIO_PIN_15
#define LED_BLUE_GPIO_Port GPIOB
#define BMI088_0_GDR_Pin GPIO_PIN_1
#define BMI088_0_GDR_GPIO_Port GPIOA
#define BMI088_0_GDR_EXTI_IRQn EXTI1_IRQn
#define BMI088_0_SCK_Pin GPIO_PIN_5
#define BMI088_0_SCK_GPIO_Port GPIOA
#define ADC2_INP8_BATT_I_Pin GPIO_PIN_5
#define ADC2_INP8_BATT_I_GPIO_Port GPIOC
#define BM0_PWMD14_Pin GPIO_PIN_14
#define BM0_PWMD14_GPIO_Port GPIOD
#define FRAM_SPI2_MISO_Pin GPIO_PIN_14
#define FRAM_SPI2_MISO_GPIO_Port GPIOB
#define BMI088_0_ACS_Pin GPIO_PIN_2
#define BMI088_0_ACS_GPIO_Port GPIOA
#define BMI088_0_MISO_Pin GPIO_PIN_6
#define BMI088_0_MISO_GPIO_Port GPIOA
#define LED_RED_Pin GPIO_PIN_12
#define LED_RED_GPIO_Port GPIOE
#define EX_I2C4_SDA_Pin GPIO_PIN_13
#define EX_I2C4_SDA_GPIO_Port GPIOD
#define BMI088_0_GCS_Pin GPIO_PIN_3
#define BMI088_0_GCS_GPIO_Port GPIOA
#define BMI088_0_MOSI_Pin GPIO_PIN_7
#define BMI088_0_MOSI_GPIO_Port GPIOA
#define EX_I2C4_SCL_Pin GPIO_PIN_12
#define EX_I2C4_SCL_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
