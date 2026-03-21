/*
 * pid.h
 *
 *  Created on: Mar 18, 2026
 *      Author: antho
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct {
    float kp;
    float ki;
    float kd;

    float integral;
    float prev_error;

    float output_limit;
} PID_t;

float PID_compute(PID_t *pid, float error, float dt);

#endif /* INC_PID_H_ */
