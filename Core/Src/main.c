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
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "as5600.h"
#include "app_autotune.h"
#include "app_command.h"
#include "app_telemetry.h"
#include "foc.h"
#include "usbd_cdc_if.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t usb_rx_buffer[USB_RX_BUFFER_SIZE];
uint32_t usb_rx_len = 0;
uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
uint32_t uart_rx_len = 0;
volatile uint8_t foc_flag = 0;
volatile uint8_t usb_rx_ready = 0;
volatile uint8_t uart_rx_ready = 0;
#if HC05_ENABLED
static uint8_t uart2_rx_byte = 0;
#endif
static char uart_line_buffer[UART_RX_BUFFER_SIZE];
static uint32_t uart_line_len = 0;
static uint8_t command_usb_tx_buffer[512];

typedef enum
{
	COMMAND_TRANSPORT_USB = 0,
	COMMAND_TRANSPORT_UART = 1
} CommandTransport_t;

static volatile CommandTransport_t active_command_transport = COMMAND_TRANSPORT_USB;

extern uint8_t USB_CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void BluePill_ForceUsbReenumeration(void);
uint8_t Command_Transmit(uint8_t *Buf, uint16_t Len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void BluePill_ForceUsbReenumeration(void)
{
	GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };

	__HAL_RCC_GPIOA_CLK_ENABLE();

	// Blue Pill boards often need D+ forced low briefly so the host sees a disconnect.
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_Delay(20);

	// Release PA12 so the USB peripheral can take ownership during MX_USB_DEVICE_Init().
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_Delay(20);
}

uint8_t Command_Transmit(uint8_t *Buf, uint16_t Len)
{
	if ((Buf == NULL) || (Len == 0U))
	{
		return USBD_FAIL;
	}

#if HC05_ENABLED
	if (active_command_transport == COMMAND_TRANSPORT_UART)
	{
		return (USART2_Transmit_Blocking(Buf, Len, 100U) == 0U) ? USBD_OK : USBD_FAIL;
	}
#endif

	if (Len > sizeof(command_usb_tx_buffer))
	{
		return USBD_FAIL;
	}

	memcpy(command_usb_tx_buffer, Buf, Len);
	for (uint32_t start = HAL_GetTick(); (HAL_GetTick() - start) < 25U;)
	{
		if (USB_CDC_Transmit_FS(command_usb_tx_buffer, Len) == USBD_OK)
		{
			return USBD_OK;
		}
		HAL_Delay(1);
	}

	return USBD_BUSY;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2)
	{
		foc_flag = 1;
		// FOC_Loop runs in the main loop to keep ISR work short and avoid
		// destabilizing the rest of the firmware.
	}
}

void enter_dfu_mode(void)
{
	// Write the magic value to the last 8 bytes of RAM
	volatile uint64_t *magic = (uint64_t *)(0x20004FF8);
	*magic = 0xDEADBEEFCC00FFEEULL;

	// Trigger a system reset
	NVIC_SystemReset();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
 if (GPIO_Pin == DFU_EXTI2_Pin)
	{
		enter_dfu_mode();
	}
}
void HC05_OnByteReceived(uint8_t ch)
{
	if ((ch == '\r') || (ch == '\n'))
	{
		if ((uart_line_len > 0U) && !uart_rx_ready)
		{
			memcpy(uart_rx_buffer, uart_line_buffer, uart_line_len);
			uart_rx_buffer[uart_line_len] = '\0';
			uart_rx_len = uart_line_len;
			uart_rx_ready = 1U;
			uart_line_len = 0U;
		}
	}
	else if (uart_line_len < (UART_RX_BUFFER_SIZE - 1U))
	{
		uart_line_buffer[uart_line_len++] = (char) ch;
	}
	else
	{
		uart_line_len = 0U;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
#if HC05_ENABLED
	if (huart->Instance == USART2)
	{
		HC05_OnByteReceived(uart2_rx_byte);
		HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1U);
	}
#else
	(void) huart;
#endif
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
#if HC05_ENABLED
	if (huart->Instance == USART2)
	{
		__HAL_UART_CLEAR_OREFLAG(huart);
		uart_line_len = 0U;
		HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1U);
	}
#else
	(void) huart;
#endif
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

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	//__HAL_RCC_AFIO_CLK_ENABLE();
	// AFIO->MAPR &= ~AFIO_MAPR_TIM1_REMAP;
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_I2C1_Init();
	MX_USART2_UART_Init();
	BluePill_ForceUsbReenumeration();
	MX_USB_DEVICE_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_IWDG_Init();
	/* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

	// Start encoder DMA reading immediately (independent of motor state)
	AS5600_StartDMA();

	// Start FOC timer for continuous velocity measurement
	HAL_TIM_Base_Start_IT(&htim2);
#if HC05_ENABLED
	HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1U);
#endif

	control_mode = MODE_VELOCITY;
	target_velocity = 0.0f; // Start with zero velocity - wait for START command

	// Keep autonomous status/telemetry on USB so the web dashboard is not affected
	// by noise or activity on the optional HC-05 link.
	AppTelemetry_SendReady();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */
		HAL_IWDG_Refresh(&hiwdg);
		AS5600_Service();
		if (foc_flag)
		{
			foc_flag = 0;
			FOC_Loop();
		}
		AppTelemetry_UpdateCache();
		if (usb_rx_ready)
		{
			active_command_transport = COMMAND_TRANSPORT_USB;
			AppCommand_Process((const char*) usb_rx_buffer, usb_rx_len);
			usb_rx_ready = 0;															// Clear the flag
			usb_rx_len = 0;																// Reset length
			memset(usb_rx_buffer, 0, USB_RX_BUFFER_SIZE); // Clear buffer for next reception
			active_command_transport = COMMAND_TRANSPORT_USB;
		}
		if (uart_rx_ready)
		{
#if HC05_ENABLED
			active_command_transport = COMMAND_TRANSPORT_UART;
			AppCommand_Process((const char*) uart_rx_buffer, uart_rx_len);
			uart_rx_ready = 0;
			uart_rx_len = 0;
			memset(uart_rx_buffer, 0, UART_RX_BUFFER_SIZE);
			active_command_transport = COMMAND_TRANSPORT_USB;
#else
			uart_rx_ready = 0;
			uart_rx_len = 0;
			memset(uart_rx_buffer, 0, UART_RX_BUFFER_SIZE);
#endif
		}
		AppAutotune_Service();
		// Telemetry is served on-demand via GET_TELEMETRY / GET_DIAG so the dashboard
		// has a deterministic request/response path and does not compete with a
		// free-running USB stream.
		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit =
	{ 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
		 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
