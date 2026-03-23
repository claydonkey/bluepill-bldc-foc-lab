/*
 * as5600.h
 *
 *  Created on: Mar 18, 2026
 *      Author: antho
 */

#ifndef INC_AS5600_H_
#define INC_AS5600_H_

#include "stm32f1xx_hal.h"

void AS5600_StartDMA(void);
void AS5600_Service(void);
float AS5600_GetMechanicalAngle(void);
uint8_t AS5600_IsHealthy(void);
uint8_t AS5600_WaitForHealthy(uint32_t timeout_ms);

// Debugging counters
extern volatile uint32_t AS5600_dma_callbacks;
extern volatile uint32_t AS5600_dma_errors;
extern volatile uint32_t AS5600_dma_starts;

// Current encoder values for FOC control
extern volatile float AS5600_mech_angle;
extern volatile float AS5600_velocity;
extern volatile uint32_t AS5600_last_update_ms;

#endif /* INC_AS5600_H_ */
