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
#include "stm32f1xx_hal.h"

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
#define HC05_TX_Pin GPIO_PIN_2
#define HC05_TX_GPIO_Port GPIOA
#define HC05_RX_Pin GPIO_PIN_3
#define HC05_RX_GPIO_Port GPIOA
#define DFU_EXTI2_Pin GPIO_PIN_2
#define DFU_EXTI2_GPIO_Port GPIOB
#define MOT1_EN_Pin GPIO_PIN_15
#define MOT1_EN_GPIO_Port GPIOB
#define PWM1a_Pin GPIO_PIN_8
#define PWM1a_GPIO_Port GPIOA
#define PWM1b_Pin GPIO_PIN_9
#define PWM1b_GPIO_Port GPIOA
#define PWM1c_Pin GPIO_PIN_10
#define PWM1c_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define USB_RX_BUFFER_SIZE 64 // A suitable buffer size for your usb commands
#define UART_RX_BUFFER_SIZE 96
extern uint8_t usb_rx_buffer[USB_RX_BUFFER_SIZE];
extern uint32_t usb_rx_len;
extern volatile uint8_t usb_rx_ready;
extern uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
extern uint32_t uart_rx_len;
extern volatile uint8_t uart_rx_ready;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
