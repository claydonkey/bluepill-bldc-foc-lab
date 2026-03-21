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
float AS5600_GetMechanicalAngle(void);

#endif /* INC_AS5600_H_ */
