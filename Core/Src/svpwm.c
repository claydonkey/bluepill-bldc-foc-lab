#include "svpwm.h"
#include <math.h>

void dq_to_alphabeta(float Ud, float Uq, float theta,
                     float *Ualpha, float *Ubeta) {
    float c = cosf(theta);
    float s = sinf(theta);
    *Ualpha = Ud * c - Uq * s;
    *Ubeta  = Ud * s + Uq * c;
}

void svpwm(float Ualpha, float Ubeta,
           float *dA, float *dB, float *dC) {

    float X = Ualpha;
    float Y = (Ualpha + sqrtf(3.0f) * Ubeta) * 0.5f;
    float Z = (Ualpha - sqrtf(3.0f) * Ubeta) * 0.5f;

    float T1, T2;
    int sector;

    if (Y >= 0 && Z < 0) sector = 1;
    else if (X < 0 && Y >= 0) sector = 2;
    else if (X < 0 && Z >= 0) sector = 3;
    else if (Y < 0 && Z >= 0) sector = 4;
    else if (X >= 0 && Y < 0) sector = 5;
    else sector = 6;

    switch (sector) {
        case 1: T1 = Y;     T2 = X;     break;
        case 2: T1 = -X;    T2 = Y;     break;
        case 3: T1 = -Z;    T2 = -X;    break;
        case 4: T1 = -Y;    T2 = -Z;    break;
        case 5: T1 = X;     T2 = -Y;    break;
        default:T1 = Z;     T2 = X;     break;
    }

    float T0 = 1.0f - T1 - T2;
    float T0_2 = T0 * 0.5f;

    *dA = T0_2 + T1 + T2;
    *dB = T0_2 + T2;
    *dC = T0_2;
}
