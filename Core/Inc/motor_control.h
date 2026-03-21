/*
 * motor_control.h
 *
 *  Created on: Mar 19, 2026
 *      Author: antho
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

void start_motor() ;


void stop_motor() ;

void set_velocity(float velocity);

void set_pid(float Kp, float Ki, float Kd) ;

#endif /* INC_MOTOR_CONTROL_H_ */
