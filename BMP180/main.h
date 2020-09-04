/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_usart.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */



#define LED_PIN_1                              GPIO_PIN_12
#define LED_PIN_2                              GPIO_PIN_13
#define LED_PIN_3                              GPIO_PIN_14
#define LED_PIN_4                              GPIO_PIN_15
#define LED_GPIO_PORT                          GPIOD
#define LED_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOD_CLK_ENABLE()


typedef enum {
	ADC_Channel_0 = 0x00, /*!< Operate with ADC channel 0 */
	ADC_Channel_1,        /*!< Operate with ADC channel 1 */
	ADC_Channel_2,        /*!< Operate with ADC channel 2 */
	ADC_Channel_3,        /*!< Operate with ADC channel 3 */
	ADC_Channel_4,        /*!< Operate with ADC channel 4 */
	ADC_Channel_5,        /*!< Operate with ADC channel 5 */
	ADC_Channel_6,        /*!< Operate with ADC channel 6 */
	ADC_Channel_7,        /*!< Operate with ADC channel 7 */
	ADC_Channel_8,        /*!< Operate with ADC channel 8 */
	ADC_Channel_9,        /*!< Operate with ADC channel 9 */
	ADC_Channel_10,       /*!< Operate with ADC channel 10 */
	ADC_Channel_11,       /*!< Operate with ADC channel 11 */
	ADC_Channel_12,       /*!< Operate with ADC channel 12 */
	ADC_Channel_13,       /*!< Operate with ADC channel 13 */
	ADC_Channel_14,       /*!< Operate with ADC channel 14 */
	ADC_Channel_15,       /*!< Operate with ADC channel 15 */
	ADC_Channel_16,       /*!< Operate with ADC channel 16 */
	ADC_Channel_17,       /*!< Operate with ADC channel 17 */
	ADC_Channel_18       /*!< Operate with ADC channel 18 */
} ADC_Channel_t;







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
#define USART_STATUS_REG                    SR
#define USART_WAIT(USARTx)                  while (!USART_TXEMPTY(USARTx))
#define USART_TXEMPTY(USARTx)               ((USARTx)->USART_STATUS_REG & USART_FLAG_TXE)
#define USART_WRITE_DATA(USARTx, data)      ((USARTx)->DR = (data))

#define D4_Pin GPIO_PIN_8
#define D4_GPIO_Port GPIOD
#define D5_Pin GPIO_PIN_9
#define D5_GPIO_Port GPIOD
#define D6_Pin GPIO_PIN_10
#define D6_GPIO_Port GPIOD
#define D7_Pin GPIO_PIN_11
#define D7_GPIO_Port GPIOD
#define RS_Pin GPIO_PIN_0
#define RS_GPIO_Port GPIOE
#define E_Pin GPIO_PIN_1
#define E_GPIO_Port GPIOE


/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
