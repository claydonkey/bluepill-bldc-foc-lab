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
#include "as5600.h"
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

// Telemetry variables for sending velocity data
#define TELEMETRY_INTERVAL 50 // Send velocity every 50ms (20 Hz)
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
		//	FOC_Loop();  // Don't call from ISR - breaks CDC
	}
}

void start_motor()
{
	// Enable motor driver (active HIGH)
	HAL_GPIO_WritePin(MOT1_EN_GPIO_Port, MOT1_EN_Pin, GPIO_PIN_SET);
	
	// Send status message about motor driver enable
	extern uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);
	char status_msg[64];
	int len = snprintf(status_msg, sizeof(status_msg),
		"{\"motor_driver\":\"enabled\"}\r\n");
	if (len > 0 && len < (int)sizeof(status_msg)) {
		CDC_Transmit_FS((uint8_t*)status_msg, len);
	}
	
	// Perform encoder alignment (applies alignment pulse, takes ~500ms)
	FOC_Init();
	
	// FOC timer is already running for velocity measurement
	// HAL_TIM_Base_Start_IT(&htim2);  // Already started in main()
	
	// Set motor running flag
	extern volatile uint8_t motor_running;
	motor_running = 1;
	
	// Send final status
	len = snprintf(status_msg, sizeof(status_msg),
		"{\"motor_status\":\"running\"}\r\n");
	if (len > 0 && len < (int)sizeof(status_msg)) {
		CDC_Transmit_FS((uint8_t*)status_msg, len);
	}
}

void stop_motor()
{
	// Set motor running flag to 0 first
	extern volatile uint8_t motor_running;
	motor_running = 0;
	
	// Keep FOC timer running for velocity measurement
	// HAL_TIM_Base_Stop_IT(&htim2);  // Don't stop timer, keep velocity updates
		// Reset PID to prevent integral windup
	FOC_ResetPID();
		// Disable motor driver (active HIGH, so set low to disable)
	HAL_GPIO_WritePin(MOT1_EN_GPIO_Port, MOT1_EN_Pin, GPIO_PIN_RESET);
	
	// Reset PWM outputs to zero
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	
	// Reset target velocity
	target_velocity = 0.0f;
}

void set_velocity(float velocity)
{
	// Set the target velocity for the motor
	target_velocity = velocity;
	
	// Reset PID when changing target to prevent windup
	FOC_ResetPID();
	
	// Send confirmation with current target
	char vel_msg[64];
	int len = snprintf(vel_msg, sizeof(vel_msg),
		"{\"target_vel\":%.2f}\r\n", target_velocity);
	if (len > 0 && len < (int)sizeof(vel_msg))
	{
		CDC_Transmit_FS((uint8_t*)vel_msg, len);
	}
}

void set_pid(float Kp, float Ki, float Kd)
{
	// Set PID parameters for velocity control
	FOC_SetPID(Kp, Ki, Kd);
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

	// Start encoder DMA reading immediately (independent of motor state)
	AS5600_StartDMA();

	// Start FOC timer for continuous velocity measurement
	HAL_TIM_Base_Start_IT(&htim2);

	control_mode = MODE_VELOCITY;
	target_velocity = 0.0f; // Start with zero velocity - wait for START command
	
	// Send startup confirmation
	CDC_Transmit_FS((uint8_t*)"{\"status\":\"ready\"}\r\n", strlen("{\"status\":\"ready\"}\r\n"));
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
			FOC_Loop();   // Call from main loop, not ISR
		}
		// Send telemetry less frequently to avoid overwhelming USB
		// Only send every 500ms to prevent buffer overflow
		static uint32_t last_telemetry_time = 0;
		uint32_t current_time = HAL_GetTick();
		if ((current_time - last_telemetry_time) >= 500) {
			float current_velocity = FOC_GetVelocity();
			char telemetry[96];
			int len = snprintf(telemetry, sizeof(telemetry),
				"{\"vel\":%.2f,\"target\":%.2f}\r\n", 
				current_velocity, target_velocity);
			if (len > 0 && len < (int)sizeof(telemetry)) {
				uint8_t result = CDC_Transmit_FS((uint8_t*)telemetry, len);
				if (result == USBD_OK) {
					last_telemetry_time = current_time;
				}
			}
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

	if (strcmp(command, "START") == 0)
	{
		// Start the motor
		start_motor();
	}
	else if (strcmp(command, "STOP") == 0)
	{
		// Stop the motor
		stop_motor();
	}
	else if (strncmp(command, "SET_VELOCITY:", 13) == 0)
	{
		// Extract the velocity value from the command
		float velocity = atof(command + 13); // Convert string to float
		set_velocity(velocity);
		
		// Send immediate confirmation with updated target
		char confirm_msg[64];
		int len = snprintf(confirm_msg, sizeof(confirm_msg),
			"{\"vel\":%.3f,\"target\":%.3f}\r\n", FOC_GetVelocity(), velocity);
		if (len > 0 && len < (int)sizeof(confirm_msg))
		{
			CDC_Transmit_FS((uint8_t*)confirm_msg, len);
		}
	}
	else if (strcmp(command, "GET_VELOCITY") == 0)
	{
		// Report current velocity and target immediately
		float current_velocity = FOC_GetVelocity();
		char vel_msg[64];
		int len = snprintf(vel_msg, sizeof(vel_msg),
			"{\"vel\":%.3f,\"target\":%.3f}\r\n", current_velocity, target_velocity);
		if (len > 0 && len < (int)sizeof(vel_msg))
		{
			CDC_Transmit_FS((uint8_t*)vel_msg, len);
		}
	}
	else if (strncmp(command, "GET_PID", 7) == 0)
	{
		// Get current PID parameters
		float Kp, Ki, Kd;
		FOC_GetPID(&Kp, &Ki, &Kd);
		char pid_msg[128];
		int len = snprintf(pid_msg, sizeof(pid_msg),
			"{\"pid\":{\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f}}\r\n", Kp, Ki, Kd);
		if (len > 0 && len < (int)sizeof(pid_msg))
		{
			CDC_Transmit_FS((uint8_t*)pid_msg, len);
		}
	}
	else if (strncmp(command, "OPEN_LOOP:", 10) == 0)
	{
		// Open-loop test: "OPEN_LOOP:0.1" for 10% duty cycle
		float duty = atof(command + 10);
		if (duty >= 0.0f && duty <= 1.0f) {
			extern ControlMode_t control_mode;
			control_mode = MODE_OPEN_LOOP;
			FOC_OpenLoopTest(duty);
			char test_msg[64];
			int len = snprintf(test_msg, sizeof(test_msg),
				"{\"open_loop\":\"%.2f\"}\r\n", duty);
			if (len > 0 && len < (int)sizeof(test_msg)) {
				CDC_Transmit_FS((uint8_t*)test_msg, len);
			}
		}
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
