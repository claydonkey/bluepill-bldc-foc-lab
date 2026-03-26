/*
 * svpwm.h
 *
 *  Created on: Mar 18, 2026
 *      Author: antho
 */

#ifndef INC_SVPWM_H_
#define INC_SVPWM_H_
void dq_to_alphabeta(float Ud, float Uq, float theta,
                     float *Ualpha, float *Ubeta);

void svpwm(float Ualpha, float Ubeta,
           float *dA, float *dB, float *dC);


#endif /* INC_SVPWM_H_ */
