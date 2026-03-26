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
#include "foc.h"
#include "motor_control.h"
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
static uint8_t uart2_rx_byte = 0;
static char uart_line_buffer[UART_RX_BUFFER_SIZE];
static uint32_t uart_line_len = 0;
static uint8_t command_usb_tx_buffer[512];
static char usb_ready_msg[] = "{\"status\":\"ready\"}\r\n";
static char foc_telemetry_tx_buffer[448];
static char foc_diag_tx_buffer[192];
static FOC_Telemetry_t foc_telemetry_cache = { 0 };
static uint32_t foc_telemetry_cache_ms = 0;

typedef enum
{
	COMMAND_TRANSPORT_USB = 0,
	COMMAND_TRANSPORT_UART = 1
} CommandTransport_t;

static volatile CommandTransport_t active_command_transport = COMMAND_TRANSPORT_USB;

extern uint8_t USB_CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);

// Telemetry variables for sending velocity data
#define TELEMETRY_INTERVAL 50 // Send velocity every 50ms (20 Hz)
#define AUTOTUNE_SAMPLE_PERIOD_MS 25U
#define AUTOTUNE_STEP_DURATION_MS 1800U
#define AUTOTUNE_SETTLE_DURATION_MS 250U
#define AUTOTUNE_COAST_DURATION_MS 250U
#define AUTOTUNE_MAX_SAMPLES 80U
#define AUTOTUNE_LOAD_HOLD_KI_MULTIPLIER 2.5f
#define POSITION_AUTOTUNE_SAMPLE_PERIOD_MS 25U
#define POSITION_AUTOTUNE_DURATION_MS 1400U
#define POSITION_AUTOTUNE_STEP_RAD 0.35f
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void process_usb_command(const char *cmd_buf, uint32_t len);
static void BluePill_ForceUsbReenumeration(void);
uint8_t Command_Transmit(uint8_t *Buf, uint16_t Len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef enum
{
	AUTOTUNE_IDLE = 0, AUTOTUNE_SETTLE, AUTOTUNE_STEP, AUTOTUNE_COAST
} AutotunePhase_t;

typedef struct
{
	uint8_t active;
	AutotunePhase_t phase;
	uint32_t phase_start_ms;
	uint32_t last_sample_ms;
	float step_uq;
	float sample_velocity[AUTOTUNE_MAX_SAMPLES];
	uint32_t sample_time_ms[AUTOTUNE_MAX_SAMPLES];
	uint32_t sample_count;
} AutotuneState_t;

static AutotuneState_t autotune =
{ 0 };

static void autotune_send_status(const char *status);
static void autotune_finish(void);
static void autotune_service(void);
static void autotune_start(void);
static void position_autotune_send_status(const char *status);
static void position_autotune_finish(void);
static void position_autotune_service(void);
static void position_autotune_start(void);
static void send_foc_telemetry_snapshot(void);
static void send_diag_snapshot(void);
static float json_safe_float(float value);

typedef struct
{
	uint8_t active;
	uint32_t start_ms;
	uint32_t last_sample_ms;
	float origin;
	float target;
	float initial_step;
	float max_overshoot;
	float first_cross_error_abs;
	float first_reach_ms;
	uint8_t crossed_target;
} PositionAutotuneState_t;

static PositionAutotuneState_t position_autotune =
{ 0 };

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

void start_motor()
{
	if (!AS5600_WaitForHealthy(250))
	{
		char fault_msg[96];
		int fault_len = snprintf(fault_msg, sizeof(fault_msg), "{\"encoder_fault\":\"offline\",\"cb\":%lu,\"err\":%lu,\"start\":%lu}\r\n", AS5600_dma_callbacks, AS5600_dma_errors, AS5600_dma_starts);
		if (fault_len > 0 && fault_len < (int) sizeof(fault_msg))
		{
			CDC_Transmit_FS((uint8_t*) fault_msg, fault_len);
		}
		return;
	}

	control_mode = MODE_VELOCITY;

	// Enable motor driver (active HIGH)
	HAL_GPIO_WritePin(MOT1_EN_GPIO_Port, MOT1_EN_Pin, GPIO_PIN_SET);

	// Send status message about motor driver enable
	char status_msg[64];
	int len = snprintf(status_msg, sizeof(status_msg), "{\"motor_driver\":\"enabled\"}\r\n");
	if (len > 0 && len < (int) sizeof(status_msg))
	{
		CDC_Transmit_FS((uint8_t*) status_msg, len);
	}

	// Perform encoder alignment (applies alignment pulse, takes ~500ms)
	FOC_Init();

	// FOC timer is already running for velocity measurement
	// HAL_TIM_Base_Start_IT(&htim2);  // Already started in main()

	// Set motor running flag
	extern volatile uint8_t motor_running;
	motor_running = 1;

	// Send final status
	len = snprintf(status_msg, sizeof(status_msg), "{\"motor_status\":\"running\"}\r\n");
	if (len > 0 && len < (int) sizeof(status_msg))
	{
		CDC_Transmit_FS((uint8_t*) status_msg, len);
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
	uint8_t mode_changed = (control_mode != MODE_VELOCITY);
	float previous_target = target_velocity;

	// Set the target velocity for the motor
	control_mode = MODE_VELOCITY;
	target_velocity = velocity;

	// Only clear controller history when switching into velocity mode.
	// Staying in velocity mode should preserve the ongoing response so
	// repeated or nearby setpoints do not create a kick.
	if (mode_changed)
	{
		FOC_ResetPIDPreserveRamp();
	}
	else if (fabsf(previous_target - velocity) < 0.001f)
	{
		// No effective target change, keep the controller completely untouched.
	}

	// Send confirmation with current target
	char vel_msg[64];
	int len = snprintf(vel_msg, sizeof(vel_msg), "{\"target_vel\":%.2f}\r\n", target_velocity);
	if (len > 0 && len < (int) sizeof(vel_msg))
	{
		CDC_Transmit_FS((uint8_t*) vel_msg, len);
	}
}

void set_pid(float Kp, float Ki, float Kd)
{
	// Set PID parameters for velocity control
	FOC_SetPID(Kp, Ki, Kd);
}

void set_position_pid(float Kp, float Ki, float Kd)
{
	FOC_SetPositionPID(Kp, Ki, Kd);
}

void set_voltage_limit(float uq_limit)
{
	FOC_SetVoltageLimit(uq_limit);
}

void set_velocity_ramp(float accel_limit)
{
	FOC_SetVelocityRamp(accel_limit);
}

void set_position_velocity_limit(float velocity_limit)
{
	FOC_SetPositionVelocityLimit(velocity_limit);
}

void set_position_accel_limit(float accel_limit)
{
	FOC_SetPositionAccelLimit(accel_limit);
}

void set_position_decel_limit(float decel_limit)
{
	FOC_SetPositionDecelLimit(decel_limit);
}

void set_low_speed_feedforward(float voltage, float fade_speed)
{
	FOC_SetLowSpeedFeedforward(voltage, fade_speed);
}

void set_low_speed_bias(float voltage, float fade_speed)
{
	FOC_SetLowSpeedBias(voltage, fade_speed);
}

void set_position_torque_assist(float voltage)
{
	FOC_SetPositionTorqueAssist(voltage);
}

static void autotune_send_status(const char *status)
{
	char msg[96];
	int len = snprintf(msg, sizeof(msg), "{\"autotune\":{\"status\":\"%s\"}}\r\n", status);
	if (len > 0 && len < (int) sizeof(msg))
	{
		CDC_Transmit_FS((uint8_t*) msg, len);
	}
}

void set_position(float position)
{
	const float two_pi = 2.0f * (float) M_PI;
	float current_position = FOC_GetMechanicalPosition();

	control_mode = MODE_POSITION;

	// Treat single-turn commands as "go to the nearest equivalent angle"
	// while still allowing explicit multi-turn absolute targets.
	if (fabsf(position) <= (two_pi + 0.001f))
	{
		float turn_offset = roundf((current_position - position) / two_pi);
		target_position = position + (turn_offset * two_pi);
	}
	else
	{
		target_position = position;
	}

	FOC_ResetPID();

	char pos_msg[64];
	int len = snprintf(pos_msg, sizeof(pos_msg), "{\"target_pos\":%.3f}\r\n", target_position);
	if (len > 0 && len < (int) sizeof(pos_msg))
	{
		CDC_Transmit_FS((uint8_t*) pos_msg, len);
	}
}

static void autotune_finish(void)
{
	float final_velocity = 0.0f;
	float tau_s = 0.25f;
	float kp = 0.1f;
	float ki = 0.1f;
	float kd = 0.0f;
	uint32_t tau_ms = 250U;
	uint32_t final_window_start = (autotune.sample_count > 8U) ? (autotune.sample_count - 8U) : 0U;
	uint32_t final_count = 0U;

	for (uint32_t i = final_window_start; i < autotune.sample_count; i++)
	{
		final_velocity += autotune.sample_velocity[i];
		final_count++;
	}
	if (final_count > 0U)
	{
		final_velocity /= (float) final_count;
	}

	if ((fabsf(final_velocity) > 0.05f) && (fabsf(autotune.step_uq) > 0.05f))
	{
		float target63 = final_velocity * 0.632f;
		for (uint32_t i = 0; i < autotune.sample_count; i++)
		{
			float sample = autotune.sample_velocity[i];
			if (((final_velocity >= 0.0f) && (sample >= target63)) || ((final_velocity < 0.0f) && (sample <= target63)))
			{
				tau_ms = autotune.sample_time_ms[i];
				break;
			}
		}

		tau_s = (float) tau_ms / 1000.0f;
		if (tau_s < 0.05f)
		{
			tau_s = 0.05f;
			tau_ms = 50U;
		}

		float plant_gain = final_velocity / autotune.step_uq;
		float lambda = fmaxf(0.10f, tau_s * 0.75f);

		if (fabsf(plant_gain) > 0.02f)
		{
			kp = tau_s / (fabsf(plant_gain) * lambda);
			ki = 1.0f / (fabsf(plant_gain) * lambda);
		}
	}

	ki *= AUTOTUNE_LOAD_HOLD_KI_MULTIPLIER;
	if (ki > 10.0f)
	{
		ki = 10.0f;
	}

	FOC_SetPID(kp, ki, kd);
	control_mode = MODE_VELOCITY;
	target_velocity = 0.0f;
	stop_motor();

	char msg[224];
	int len = snprintf(msg, sizeof(msg), "{\"autotune\":{\"status\":\"done\",\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f,\"step_uq\":%.2f,\"final_vel\":%.3f,\"tau_ms\":%lu}}\r\n", kp, ki, kd,
			autotune.step_uq, final_velocity, tau_ms);
	if (len > 0 && len < (int) sizeof(msg))
	{
		CDC_Transmit_FS((uint8_t*) msg, len);
	}

	memset(&autotune, 0, sizeof(autotune));
}

static void autotune_service(void)
{
	if (!autotune.active)
	{
		return;
	}

	uint32_t now = HAL_GetTick();

	switch (autotune.phase)
	{
	case AUTOTUNE_SETTLE:
		if ((now - autotune.phase_start_ms) >= AUTOTUNE_SETTLE_DURATION_MS)
		{
			autotune.phase = AUTOTUNE_STEP;
			autotune.phase_start_ms = now;
			autotune.last_sample_ms = now;
			FOC_StartTorque(autotune.step_uq);
			autotune_send_status("step");
		}
		break;

	case AUTOTUNE_STEP:
		if ((now - autotune.last_sample_ms) >= AUTOTUNE_SAMPLE_PERIOD_MS && autotune.sample_count < AUTOTUNE_MAX_SAMPLES)
		{
			autotune.sample_velocity[autotune.sample_count] = FOC_GetVelocity();
			autotune.sample_time_ms[autotune.sample_count] = now - autotune.phase_start_ms;
			autotune.sample_count++;
			autotune.last_sample_ms = now;
		}

		if ((now - autotune.phase_start_ms) >= AUTOTUNE_STEP_DURATION_MS)
		{
			FOC_StartTorque(0.0f);
			autotune.phase = AUTOTUNE_COAST;
			autotune.phase_start_ms = now;
			autotune_send_status("coast");
		}
		break;

	case AUTOTUNE_COAST:
		if ((now - autotune.phase_start_ms) >= AUTOTUNE_COAST_DURATION_MS)
		{
			autotune_finish();
		}
		break;

	default:
		break;
	}
}

static void autotune_start(void)
{
	memset(&autotune, 0, sizeof(autotune));
	autotune.active = 1U;
	autotune.phase = AUTOTUNE_SETTLE;
	autotune.phase_start_ms = HAL_GetTick();
	autotune.last_sample_ms = autotune.phase_start_ms;
	autotune.step_uq = fminf(1.5f, FOC_GetVoltageLimit() * 0.5f);
	FOC_StartTorque(0.0f);
	autotune_send_status("settle");
}

static float json_safe_float(float value)
{
	return isfinite(value) ? value : 0.0f;
}

static int32_t telemetry_scale_100(float value)
{
	value = json_safe_float(value);
	if (value > 21474836.0f)
	{
		value = 21474836.0f;
	}
	else if (value < -21474836.0f)
	{
		value = -21474836.0f;
	}
	return (int32_t) lrintf(value * 100.0f);
}

static int32_t telemetry_scale_1000(float value)
{
	value = json_safe_float(value);
	if (value > 2147483.0f)
	{
		value = 2147483.0f;
	}
	else if (value < -2147483.0f)
	{
		value = -2147483.0f;
	}
	return (int32_t) lrintf(value * 1000.0f);
}

static void send_foc_telemetry_snapshot(void)
{
	int len = snprintf(foc_telemetry_tx_buffer, sizeof(foc_telemetry_tx_buffer),
			"{\"foc\":{\"vi\":%ld,\"mechi\":%ld,\"ti\":%ld,\"tri\":%ld,\"tpi\":%ld,\"erri\":%ld,\"uqi\":%ld,\"vlimi\":%ld,\"vrampi\":%ld,\"pvlimi\":%ld,\"pacci\":%ld,\"pdeci\":%ld,\"pboosti\":%ld,\"usat\":%u,\"mode\":%u,\"mod\":%u,\"pm\":%u,\"adiri\":%ld,\"aligni\":%ld,\"lc\":%lu,\"r\":%u,\"pwm\":[%lu,%lu,%lu],\"per\":%lu,\"cb\":%lu,\"derr\":%lu,\"start\":%lu,\"run\":%u}}\r\n",
			(long) telemetry_scale_100(json_safe_float(foc_telemetry_cache.velocity)),
			(long) telemetry_scale_1000(json_safe_float(foc_telemetry_cache.mechanical_angle)),
			(long) telemetry_scale_100(json_safe_float(foc_telemetry_cache.target_velocity)),
			(long) telemetry_scale_100(json_safe_float(foc_telemetry_cache.ramped_velocity_target)),
			(long) telemetry_scale_1000(json_safe_float(foc_telemetry_cache.target_position)),
			(long) telemetry_scale_100(json_safe_float(foc_telemetry_cache.velocity_error)),
			(long) telemetry_scale_100(json_safe_float(foc_telemetry_cache.uq_voltage)),
			(long) telemetry_scale_100(json_safe_float(foc_telemetry_cache.voltage_limit)),
			(long) telemetry_scale_100(json_safe_float(foc_telemetry_cache.velocity_ramp_rate)),
			(long) telemetry_scale_100(json_safe_float(foc_telemetry_cache.position_velocity_limit)),
			(long) telemetry_scale_100(json_safe_float(foc_telemetry_cache.position_accel_limit)),
			(long) telemetry_scale_100(json_safe_float(foc_telemetry_cache.position_decel_limit)),
			(long) telemetry_scale_100(json_safe_float(foc_telemetry_cache.position_torque_assist)), foc_telemetry_cache.uq_saturated, foc_telemetry_cache.control_mode,
			foc_telemetry_cache.modulation_mode, foc_telemetry_cache.phase_map,
			(long) telemetry_scale_1000(json_safe_float(foc_telemetry_cache.sensor_direction)),
			(long) telemetry_scale_1000(json_safe_float(foc_telemetry_cache.alignment_offset)), foc_telemetry_cache.loop_count, foc_telemetry_cache.raw_angle,
			foc_telemetry_cache.pwm1, foc_telemetry_cache.pwm2, foc_telemetry_cache.pwm3, foc_telemetry_cache.pwm_period, foc_telemetry_cache.dma_callbacks, foc_telemetry_cache.dma_errors, foc_telemetry_cache.dma_starts,
			foc_telemetry_cache.motor_running);

	if (len > 0 && len < (int) sizeof(foc_telemetry_tx_buffer))
	{
		CDC_Transmit_FS((uint8_t*) foc_telemetry_tx_buffer, len);
	}
}

static void send_diag_snapshot(void)
{
	int diag_len = snprintf(foc_diag_tx_buffer, sizeof(foc_diag_tx_buffer), "{\"diag\":{\"lc\":%lu,\"sent\":%lu,\"failed\":%lu,\"enc\":%u,\"cb\":%lu,\"err\":%lu,\"start\":%lu,\"run\":%u}}\r\n", foc_telemetry_cache.loop_count,
			foc_telemetry_cache.messages_sent, foc_telemetry_cache.messages_failed, foc_telemetry_cache.raw_angle, foc_telemetry_cache.dma_callbacks, foc_telemetry_cache.dma_errors, foc_telemetry_cache.dma_starts,
			foc_telemetry_cache.motor_running);

	if (diag_len > 0 && diag_len < (int) sizeof(foc_diag_tx_buffer))
	{
		CDC_Transmit_FS((uint8_t*) foc_diag_tx_buffer, diag_len);
	}
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
	USB_CDC_Transmit_FS((uint8_t*) usb_ready_msg, (uint16_t) strlen(usb_ready_msg));
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
		if ((HAL_GetTick() - foc_telemetry_cache_ms) >= 50U)
		{
			FOC_GetTelemetry(&foc_telemetry_cache);
			foc_telemetry_cache_ms = HAL_GetTick();
		}
		if (usb_rx_ready)
		{
			active_command_transport = COMMAND_TRANSPORT_USB;
			process_usb_command((const char*) usb_rx_buffer, usb_rx_len);
			usb_rx_ready = 0;															// Clear the flag
			usb_rx_len = 0;																// Reset length
			memset(usb_rx_buffer, 0, USB_RX_BUFFER_SIZE); // Clear buffer for next reception
			active_command_transport = COMMAND_TRANSPORT_USB;
		}
		if (uart_rx_ready)
		{
#if HC05_ENABLED
			active_command_transport = COMMAND_TRANSPORT_UART;
			process_usb_command((const char*) uart_rx_buffer, uart_rx_len);
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
		autotune_service();
		position_autotune_service();
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

// Implement the command processing function
void process_usb_command(const char *cmd_buf, uint32_t len)
{
	char command[UART_RX_BUFFER_SIZE];
	uint32_t actual_len = len;

	// Copy to a local buffer to ensure it's mutable and null-terminated
	if (actual_len >= UART_RX_BUFFER_SIZE)
	{
		actual_len = UART_RX_BUFFER_SIZE - 1;
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

	if (strcmp(command, "PING") == 0)
	{
		CDC_Transmit_FS((uint8_t*) "{\"ping\":\"pong\"}\r\n", strlen("{\"ping\":\"pong\"}\r\n"));
	}
	else if (strcmp(command, "GET_STATUS") == 0)
	{
		char status_msg[160];
		int status_len = snprintf(status_msg, sizeof(status_msg),
				"{\"status\":{\"running\":%u,\"mode\":%u,\"target_velocity\":%.3f,\"target_position\":%.3f}}\r\n",
				motor_running, (unsigned int) control_mode, target_velocity, target_position);
		if (status_len > 0 && status_len < (int) sizeof(status_msg))
		{
			CDC_Transmit_FS((uint8_t*) status_msg, status_len);
		}
	}
	else if (strcmp(command, "GET_TELEMETRY") == 0)
	{
		send_foc_telemetry_snapshot();
	}
	else if (strcmp(command, "GET_DIAG") == 0)
	{
		send_diag_snapshot();
	}
	else if (strcmp(command, "START") == 0)
	{
		// Start the motor
		start_motor();
	}
	else if (strcmp(command, "START_OPENLOOP") == 0)
	{
		extern volatile uint8_t motor_running;
		FOC_StartOpenLoop(20.0f, 1.0f);
		HAL_GPIO_WritePin(MOT1_EN_GPIO_Port, MOT1_EN_Pin, GPIO_PIN_SET);
		motor_running = 1;

		char mode_msg[96];
		int len = snprintf(mode_msg, sizeof(mode_msg), "{\"open_loop\":{\"elec_vel\":%.2f,\"uq\":%.2f}}\r\n", 20.0f, 1.0f);
		if (len > 0 && len < (int) sizeof(mode_msg))
		{
			CDC_Transmit_FS((uint8_t*) mode_msg, len);
		}
	}
	else if (strcmp(command, "START_AUTOTUNE") == 0)
	{
		if (!motor_running)
		{
			start_motor();
		}

		if (motor_running)
		{
			autotune_start();
		}
	}
	else if (strcmp(command, "START_POSITION_AUTOTUNE") == 0)
	{
		if (!motor_running)
		{
			start_motor();
		}

		if (motor_running)
		{
			position_autotune_start();
		}
	}
	else if (strncmp(command, "TEST_VECTOR:", 12) == 0)
	{
		int vector_index = atoi(command + 12);
		if (vector_index >= 0 && vector_index <= 5)
		{
			extern volatile uint8_t motor_running;
			FOC_StartVectorTest((uint8_t) vector_index, 1.0f);
			HAL_GPIO_WritePin(MOT1_EN_GPIO_Port, MOT1_EN_Pin, GPIO_PIN_SET);
			motor_running = 1;

			char vector_msg[96];
			int len = snprintf(vector_msg, sizeof(vector_msg), "{\"vector_test\":{\"index\":%d,\"uq\":%.2f}}\r\n", vector_index, 1.0f);
			if (len > 0 && len < (int) sizeof(vector_msg))
			{
				CDC_Transmit_FS((uint8_t*) vector_msg, len);
			}
		}
	}
	else if (strcmp(command, "STOP") == 0)
	{
		// Stop the motor
		stop_motor();
	}
	else if (strncmp(command, "SET_PHASE_MAP:", 14) == 0)
	{
		int map = atoi(command + 14);
		if (map >= 0 && map <= 5)
		{
			FOC_SetPhaseMap((uint8_t) map);
			char phase_msg[64];
			int phase_len = snprintf(phase_msg, sizeof(phase_msg), "{\"phase_map\":%d}\r\n", map);
			if (phase_len > 0 && phase_len < (int) sizeof(phase_msg))
			{
				CDC_Transmit_FS((uint8_t*) phase_msg, phase_len);
			}
		}
	}
	else if (strncmp(command, "SET_MODULATION:", 15) == 0)
	{
		const char *mode = command + 15;
		ModulationMode_t new_mode;
		if (strcmp(mode, "SVPWM") == 0)
		{
			new_mode = MODULATION_SVPWM;
		}
		else if (strcmp(mode, "SINE") == 0)
		{
			new_mode = MODULATION_SINE;
		}
		else
		{
			return;
		}

		FOC_SetModulationMode(new_mode);
		char mod_msg[64];
		int mod_len = snprintf(mod_msg, sizeof(mod_msg), "{\"modulation\":\"%s\"}\r\n", (FOC_GetModulationMode() == MODULATION_SINE) ? "SINE" : "SVPWM");
		if (mod_len > 0 && mod_len < (int) sizeof(mod_msg))
		{
			CDC_Transmit_FS((uint8_t*) mod_msg, mod_len);
		}
	}
	else if (strcmp(command, "GET_MODULATION") == 0)
	{
		char mod_msg[64];
		int mod_len = snprintf(mod_msg, sizeof(mod_msg), "{\"modulation\":\"%s\"}\r\n", (FOC_GetModulationMode() == MODULATION_SINE) ? "SINE" : "SVPWM");
		if (mod_len > 0 && mod_len < (int) sizeof(mod_msg))
		{
			CDC_Transmit_FS((uint8_t*) mod_msg, mod_len);
		}
	}
	else if (strncmp(command, "SET_VELOCITY:", 13) == 0)
	{
		// Extract the velocity value from the command
		float velocity = atof(command + 13); // Convert string to float
		set_velocity(velocity);

		// Send immediate confirmation with updated target
		char confirm_msg[64];
		int len = snprintf(confirm_msg, sizeof(confirm_msg), "{\"vel\":%.3f,\"target\":%.3f}\r\n", FOC_GetVelocity(), velocity);
		if (len > 0 && len < (int) sizeof(confirm_msg))
		{
			CDC_Transmit_FS((uint8_t*) confirm_msg, len);
		}
	}
	else if (strncmp(command, "SET_POSITION:", 13) == 0)
	{
		float position = atof(command + 13);
		set_position(position);
	}
	else if (strcmp(command, "GET_VELOCITY") == 0)
	{
		// Report current velocity and target immediately
		float current_velocity = FOC_GetVelocity();
		char vel_msg[64];
		int len = snprintf(vel_msg, sizeof(vel_msg), "{\"vel\":%.3f,\"target\":%.3f}\r\n", current_velocity, target_velocity);
		if (len > 0 && len < (int) sizeof(vel_msg))
		{
			CDC_Transmit_FS((uint8_t*) vel_msg, len);
		}
	}
	else if (strcmp(command, "GET_POSITION") == 0)
	{
		char pos_msg[96];
		int len = snprintf(pos_msg, sizeof(pos_msg), "{\"pos\":%.3f,\"target_pos\":%.3f}\r\n", FOC_GetMechanicalPosition(), target_position);
		if (len > 0 && len < (int) sizeof(pos_msg))
		{
			CDC_Transmit_FS((uint8_t*) pos_msg, len);
		}
	}
	else if (strncmp(command, "GET_PID", 7) == 0)
	{
		// Get current PID parameters
		float Kp, Ki, Kd;
		FOC_GetPID(&Kp, &Ki, &Kd);
		char pid_msg[128];
		int len = snprintf(pid_msg, sizeof(pid_msg), "{\"pid\":{\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f}}\r\n", Kp, Ki, Kd);
		if (len > 0 && len < (int) sizeof(pid_msg))
		{
			CDC_Transmit_FS((uint8_t*) pid_msg, len);
		}
	}
	else if (strncmp(command, "GET_POSITION_PID", 16) == 0)
	{
		float Kp, Ki, Kd;
		FOC_GetPositionPID(&Kp, &Ki, &Kd);
		char pid_msg[144];
		int len = snprintf(pid_msg, sizeof(pid_msg), "{\"position_pid\":{\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f}}\r\n", Kp, Ki, Kd);
		if (len > 0 && len < (int) sizeof(pid_msg))
		{
			CDC_Transmit_FS((uint8_t*) pid_msg, len);
		}
	}
	else if (strncmp(command, "GET_VOLTAGE_LIMIT", 17) == 0)
	{
		char limit_msg[64];
		int len = snprintf(limit_msg, sizeof(limit_msg), "{\"voltage_limit\":%.2f}\r\n", FOC_GetVoltageLimit());
		if (len > 0 && len < (int) sizeof(limit_msg))
		{
			CDC_Transmit_FS((uint8_t*) limit_msg, len);
		}
	}
	else if (strncmp(command, "GET_VELOCITY_RAMP", 17) == 0)
	{
		char ramp_msg[64];
		int len = snprintf(ramp_msg, sizeof(ramp_msg), "{\"velocity_ramp\":%.2f}\r\n", FOC_GetVelocityRamp());
		if (len > 0 && len < (int) sizeof(ramp_msg))
		{
			CDC_Transmit_FS((uint8_t*) ramp_msg, len);
		}
	}
	else if (strncmp(command, "SET_VELOCITY_RAMP:", 18) == 0)
	{
		float accel_limit = atof(command + 18);
		set_velocity_ramp(accel_limit);

		char ramp_msg[64];
		int len = snprintf(ramp_msg, sizeof(ramp_msg), "{\"velocity_ramp\":%.2f}\r\n", FOC_GetVelocityRamp());
		if (len > 0 && len < (int) sizeof(ramp_msg))
		{
			CDC_Transmit_FS((uint8_t*) ramp_msg, len);
		}
	}
	else if (strncmp(command, "GET_POSITION_VELOCITY_LIMIT", 27) == 0)
	{
		char limit_msg[80];
		int len = snprintf(limit_msg, sizeof(limit_msg), "{\"position_velocity_limit\":%.2f}\r\n", FOC_GetPositionVelocityLimit());
		if (len > 0 && len < (int) sizeof(limit_msg))
		{
			CDC_Transmit_FS((uint8_t*) limit_msg, len);
		}
	}
	else if (strncmp(command, "SET_POSITION_VELOCITY_LIMIT:", 28) == 0)
	{
		float velocity_limit = atof(command + 28);
		set_position_velocity_limit(velocity_limit);

		char limit_msg[80];
		int len = snprintf(limit_msg, sizeof(limit_msg), "{\"position_velocity_limit\":%.2f}\r\n", FOC_GetPositionVelocityLimit());
		if (len > 0 && len < (int) sizeof(limit_msg))
		{
			CDC_Transmit_FS((uint8_t*) limit_msg, len);
		}
	}
	else if (strncmp(command, "GET_POSITION_ACCEL_LIMIT", 24) == 0)
	{
		char accel_msg[80];
		int len = snprintf(accel_msg, sizeof(accel_msg), "{\"position_accel_limit\":%.2f}\r\n", FOC_GetPositionAccelLimit());
		if (len > 0 && len < (int) sizeof(accel_msg))
		{
			CDC_Transmit_FS((uint8_t*) accel_msg, len);
		}
	}
	else if (strncmp(command, "GET_POSITION_DECEL_LIMIT", 24) == 0)
	{
		char decel_msg[80];
		int len = snprintf(decel_msg, sizeof(decel_msg), "{\"position_decel_limit\":%.2f}\r\n", FOC_GetPositionDecelLimit());
		if (len > 0 && len < (int) sizeof(decel_msg))
		{
			CDC_Transmit_FS((uint8_t*) decel_msg, len);
		}
	}
	else if (strncmp(command, "SET_POSITION_ACCEL_LIMIT:", 25) == 0)
	{
		float accel_limit = atof(command + 25);
		set_position_accel_limit(accel_limit);

		char accel_msg[80];
		int len = snprintf(accel_msg, sizeof(accel_msg), "{\"position_accel_limit\":%.2f}\r\n", FOC_GetPositionAccelLimit());
		if (len > 0 && len < (int) sizeof(accel_msg))
		{
			CDC_Transmit_FS((uint8_t*) accel_msg, len);
		}
	}
	else if (strncmp(command, "SET_POSITION_DECEL_LIMIT:", 25) == 0)
	{
		float decel_limit = atof(command + 25);
		set_position_decel_limit(decel_limit);

		char decel_msg[80];
		int len = snprintf(decel_msg, sizeof(decel_msg), "{\"position_decel_limit\":%.2f}\r\n", FOC_GetPositionDecelLimit());
		if (len > 0 && len < (int) sizeof(decel_msg))
		{
			CDC_Transmit_FS((uint8_t*) decel_msg, len);
		}
	}
	else if (strncmp(command, "SET_VOLTAGE_LIMIT:", 18) == 0)
	{
		float uq_limit = atof(command + 18);
		set_voltage_limit(uq_limit);

		char limit_msg[64];
		int len = snprintf(limit_msg, sizeof(limit_msg), "{\"voltage_limit\":%.2f}\r\n", FOC_GetVoltageLimit());
		if (len > 0 && len < (int) sizeof(limit_msg))
		{
			CDC_Transmit_FS((uint8_t*) limit_msg, len);
		}
	}
	else if (strncmp(command, "GET_FEEDFORWARD", 15) == 0)
	{
		float ff_voltage;
		float ff_fade_speed;
		FOC_GetLowSpeedFeedforward(&ff_voltage, &ff_fade_speed);

		char ff_msg[96];
		int len = snprintf(ff_msg, sizeof(ff_msg), "{\"feedforward\":{\"voltage\":%.2f,\"fade_speed\":%.2f}}\r\n", ff_voltage, ff_fade_speed);
		if (len > 0 && len < (int) sizeof(ff_msg))
		{
			CDC_Transmit_FS((uint8_t*) ff_msg, len);
		}
	}
	else if (strncmp(command, "SET_FEEDFORWARD:", 16) == 0)
	{
		float ff_voltage, ff_fade_speed;
		if (sscanf(command + 16, "%f,%f", &ff_voltage, &ff_fade_speed) == 2)
		{
			set_low_speed_feedforward(ff_voltage, ff_fade_speed);

			char ff_msg[96];
			int len = snprintf(ff_msg, sizeof(ff_msg), "{\"feedforward\":{\"voltage\":%.2f,\"fade_speed\":%.2f}}\r\n", ff_voltage, ff_fade_speed);
			if (len > 0 && len < (int) sizeof(ff_msg))
			{
				CDC_Transmit_FS((uint8_t*) ff_msg, len);
			}
		}
	}
	else if (strncmp(command, "GET_LOW_SPEED_BIAS", 18) == 0)
	{
		float bias_voltage;
		float bias_fade_speed;
		FOC_GetLowSpeedBias(&bias_voltage, &bias_fade_speed);

		char bias_msg[96];
		int len = snprintf(bias_msg, sizeof(bias_msg), "{\"low_speed_bias\":{\"voltage\":%.2f,\"fade_speed\":%.2f}}\r\n", bias_voltage, bias_fade_speed);
		if (len > 0 && len < (int) sizeof(bias_msg))
		{
			CDC_Transmit_FS((uint8_t*) bias_msg, len);
		}
	}
	else if (strncmp(command, "SET_LOW_SPEED_BIAS:", 19) == 0)
	{
		float bias_voltage, bias_fade_speed;
		if (sscanf(command + 19, "%f,%f", &bias_voltage, &bias_fade_speed) == 2)
		{
			set_low_speed_bias(bias_voltage, bias_fade_speed);

			char bias_msg[96];
			int len = snprintf(bias_msg, sizeof(bias_msg), "{\"low_speed_bias\":{\"voltage\":%.2f,\"fade_speed\":%.2f}}\r\n", bias_voltage, bias_fade_speed);
			if (len > 0 && len < (int) sizeof(bias_msg))
			{
				CDC_Transmit_FS((uint8_t*) bias_msg, len);
			}
		}
	}
	else if (strncmp(command, "GET_POSITION_TORQUE_ASSIST", 26) == 0)
	{
		char assist_msg[80];
		int len = snprintf(assist_msg, sizeof(assist_msg), "{\"position_torque_assist\":%.2f}\r\n", FOC_GetPositionTorqueAssist());
		if (len > 0 && len < (int) sizeof(assist_msg))
		{
			CDC_Transmit_FS((uint8_t*) assist_msg, len);
		}
	}
	else if (strncmp(command, "SET_POSITION_TORQUE_ASSIST:", 27) == 0)
	{
		float assist_voltage = atof(command + 27);
		set_position_torque_assist(assist_voltage);

		char assist_msg[80];
		int len = snprintf(assist_msg, sizeof(assist_msg), "{\"position_torque_assist\":%.2f}\r\n", FOC_GetPositionTorqueAssist());
		if (len > 0 && len < (int) sizeof(assist_msg))
		{
			CDC_Transmit_FS((uint8_t*) assist_msg, len);
		}
	}
	else if (strncmp(command, "SET_PID:", 8) == 0)
	{
		float Kp, Ki, Kd;
		if (sscanf(command + 8, "%f,%f,%f", &Kp, &Ki, &Kd) == 3)
		{
			set_pid(Kp, Ki, Kd);

			char pid_msg[128];
			int len = snprintf(pid_msg, sizeof(pid_msg), "{\"pid\":{\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f}}\r\n", Kp, Ki, Kd);
			if (len > 0 && len < (int) sizeof(pid_msg))
			{
				CDC_Transmit_FS((uint8_t*) pid_msg, len);
			}
		}
	}
	else if (strncmp(command, "SET_POSITION_PID:", 17) == 0)
	{
		float Kp, Ki, Kd;
		if (sscanf(command + 17, "%f,%f,%f", &Kp, &Ki, &Kd) == 3)
		{
			set_position_pid(Kp, Ki, Kd);

			char pid_msg[144];
			int len = snprintf(pid_msg, sizeof(pid_msg), "{\"position_pid\":{\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f}}\r\n", Kp, Ki, Kd);
			if (len > 0 && len < (int) sizeof(pid_msg))
			{
				CDC_Transmit_FS((uint8_t*) pid_msg, len);
			}
		}
	}
}

static void position_autotune_send_status(const char *status)
{
	char msg[112];
	int len = snprintf(msg, sizeof(msg), "{\"position_autotune\":{\"status\":\"%s\"}}\r\n", status);
	if (len > 0 && len < (int) sizeof(msg))
	{
		CDC_Transmit_FS((uint8_t*) msg, len);
	}
}

static void position_autotune_finish(void)
{
	float kp = 1.5f;
	float ki = 0.0f;
	float kd = 0.05f;
	float reach_ms = position_autotune.first_reach_ms;
	float overshoot = position_autotune.max_overshoot;

	if ((reach_ms <= 0.0f) || (reach_ms > 1000.0f))
	{
		kp = 2.4f;
		kd = 0.06f;
	}
	else if (reach_ms > 650.0f)
	{
		kp = 2.0f;
		kd = 0.06f;
	}
	else if (reach_ms > 350.0f)
	{
		kp = 1.6f;
		kd = 0.05f;
	}
	else
	{
		kp = 1.2f;
		kd = 0.04f;
	}

	if (overshoot > 0.14f)
	{
		kp *= 0.65f;
		kd += 0.05f;
	}
	else if (overshoot > 0.08f)
	{
		kp *= 0.8f;
		kd += 0.03f;
	}
	else if (overshoot < 0.02f && reach_ms > 0.0f && reach_ms < 260.0f)
	{
		kp *= 1.1f;
	}

	if (kp < 0.5f)
	{
		kp = 0.5f;
	}
	else if (kp > 6.0f)
	{
		kp = 6.0f;
	}

	if (kd < 0.0f)
	{
		kd = 0.0f;
	}
	else if (kd > 0.25f)
	{
		kd = 0.25f;
	}

	FOC_SetPositionPID(kp, ki, kd);

	char msg[224];
	int len = snprintf(msg, sizeof(msg), "{\"position_autotune\":{\"status\":\"done\",\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f,\"reach_ms\":%.0f,\"overshoot\":%.4f}}\r\n", kp, ki, kd, reach_ms, overshoot);
	if (len > 0 && len < (int) sizeof(msg))
	{
		CDC_Transmit_FS((uint8_t*) msg, len);
	}

	memset(&position_autotune, 0, sizeof(position_autotune));
}

static void position_autotune_service(void)
{
	if (!position_autotune.active)
	{
		return;
	}

	uint32_t now = HAL_GetTick();
	if ((now - position_autotune.last_sample_ms) >= POSITION_AUTOTUNE_SAMPLE_PERIOD_MS)
	{
		float error = target_position - FOC_GetMechanicalPosition();

		if (!position_autotune.crossed_target)
		{
			if (fabsf(error) <= (position_autotune.initial_step * 0.10f) && position_autotune.first_reach_ms <= 0.0f)
			{
				position_autotune.first_reach_ms = (float) (now - position_autotune.start_ms);
			}

			if (error <= 0.0f)
			{
				position_autotune.crossed_target = 1U;
				position_autotune.first_cross_error_abs = fabsf(error);
				position_autotune.max_overshoot = fabsf(error);
			}
		}
		else
		{
			float abs_error = fabsf(error);
			if (abs_error > position_autotune.max_overshoot)
			{
				position_autotune.max_overshoot = abs_error;
			}
		}

		position_autotune.last_sample_ms = now;
	}

	if ((now - position_autotune.start_ms) >= POSITION_AUTOTUNE_DURATION_MS)
	{
		position_autotune_finish();
	}
}

static void position_autotune_start(void)
{
	memset(&position_autotune, 0, sizeof(position_autotune));

	position_autotune.active = 1U;
	position_autotune.start_ms = HAL_GetTick();
	position_autotune.last_sample_ms = position_autotune.start_ms;
	position_autotune.origin = FOC_GetMechanicalPosition();
	position_autotune.initial_step = POSITION_AUTOTUNE_STEP_RAD;
	position_autotune.target = position_autotune.origin + POSITION_AUTOTUNE_STEP_RAD;

	FOC_SetPositionPID(1.2f, 0.0f, 0.04f);
	set_position(position_autotune.target);
	position_autotune_send_status("running");
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
