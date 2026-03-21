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
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "foc.h"
#include "motor_control.h"
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
volatile uint8_t foc_flag = 0;
volatile uint8_t usb_rx_ready = 0;
extern uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void process_usb_command(const char *cmd_buf, uint32_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2)
	{

		foc_flag = 1;
		//	FOC_Loop();
	}
}

void start_motor()
{
	HAL_GPIO_WritePin(MOT1_EN_GPIO_Port, MOT1_EN_Pin, GPIO_PIN_SET);

	FOC_Init();
	HAL_TIM_Base_Start_IT(&htim2);
}

void stop_motor()
{
	HAL_GPIO_WritePin(MOT1_EN_GPIO_Port, MOT1_EN_Pin, GPIO_PIN_RESET);
	HAL_TIM_Base_Stop_IT(&htim2);
}

void set_velocity(float velocity)
{
	// Code to set the motor velocity
}

void set_pid(float Kp, float Ki, float Kd)
{
	// Code to set PID parameters
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
	//AFIO->MAPR &= ~AFIO_MAPR_TIM1_REMAP;
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_I2C1_Init();
	MX_USB_DEVICE_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

	control_mode = MODE_VELOCITY;
	target_velocity = 0.4f; // rad/s example
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */
		if (usb_rx_ready)
		{
			process_usb_command((const char*) usb_rx_buffer, usb_rx_len);
			usb_rx_ready = 0; // Clear the flag
			usb_rx_len = 0;   // Reset length
			memset(usb_rx_buffer, 0, USB_RX_BUFFER_SIZE); // Clear buffer for next reception
		}
		if (foc_flag)
		{
			foc_flag = 0;
			FOC_Loop();   // now in thread context, not ISR
		}
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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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

// Implement the command processing function
// Implement the command processing function
void process_usb_command(const char *cmd_buf, uint32_t len)
{
	char command[USB_RX_BUFFER_SIZE];
	uint32_t actual_len = len;

	// Copy to a local buffer to ensure it's mutable and null-terminated
	if (actual_len >= USB_RX_BUFFER_SIZE)
	{
		actual_len = USB_RX_BUFFER_SIZE - 1;
	}
	strncpy(command, cmd_buf, actual_len);
	command[actual_len] = '\0'; // Ensure null termination

	// Trim any trailing newline or carriage return characters
	for (int i = actual_len - 1; i >= 0; i--)
	{
		if (command[i] == '\r' || command[i] == '\n')
		{
			command[i] = '\0';
		}
		else
		{
			break; // Stop trimming once non-newline char is found
		}
	}

	// Process commands
	if (strcmp(command, "START") == 0)
	{
		// Start the motor
		start_motor(); // Implement this function as needed
		CDC_Transmit_FS((uint8_t*) "Motor started.\r\n", strlen("Motor started.\r\n"));
	}
	else if (strcmp(command, "STOP") == 0)
	{
		// Stop the motor
		stop_motor(); // Implement this function as needed
		CDC_Transmit_FS((uint8_t*) "Motor stopped.\r\n", strlen("Motor stopped.\r\n"));
	}
	else if (strncmp(command, "SET_VELOCITY:", 14) == 0)
	{
		// Extract the velocity value from the command
		float velocity = atof(command + 14); // Convert string to float
		set_velocity(velocity); // Implement this function as needed
		CDC_Transmit_FS((uint8_t*) "Velocity set.\r\n", strlen("Velocity set.\r\n"));
	}
	else if (strncmp(command, "SET_PID:", 8) == 0)
	{
		// Extract PID parameters from the command
		float Kp, Ki, Kd;
		sscanf(command + 8, "%f,%f,%f", &Kp, &Ki, &Kd); // Parse Kp, Ki, Kd
		set_pid(Kp, Ki, Kd); // Implement this function as needed
		CDC_Transmit_FS((uint8_t*) "PID parameters set.\r\n", strlen("PID parameters set.\r\n"));
	}

	else
	{
		char unknown_cmd_msg[USB_RX_BUFFER_SIZE + 20];
		snprintf(unknown_cmd_msg, sizeof(unknown_cmd_msg), "Unknown command: '%s'\r\n", command);
		CDC_Transmit_FS((uint8_t*) unknown_cmd_msg, strlen(unknown_cmd_msg));
	}
}

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
