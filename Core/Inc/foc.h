#ifndef FOC_H
#define FOC_H

#include "stm32f1xx_hal.h"

typedef enum {
    MODE_VELOCITY,
    MODE_POSITION
} ControlMode_t;

extern ControlMode_t control_mode;
extern float target_velocity;
extern float target_position;

void FOC_Init(void);
void FOC_Loop(void);

#endif
