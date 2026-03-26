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
    const float sqrt3_over_2 = 0.8660254f;

    // Convert alpha-beta vector into three phase references.
    float Va = Ualpha;
    float Vb = -0.5f * Ualpha + sqrt3_over_2 * Ubeta;
    float Vc = -0.5f * Ualpha - sqrt3_over_2 * Ubeta;

    // Inject common-mode voltage so the largest and smallest phase
    // references are centered in the available PWM window. This is the
    // standard zero-sequence form of SVPWM and avoids sector mapping bugs.
    float vmax = fmaxf(Va, fmaxf(Vb, Vc));
    float vmin = fminf(Va, fminf(Vb, Vc));
    float voffset = -0.5f * (vmax + vmin);

    Va += voffset;
    Vb += voffset;
    Vc += voffset;

    *dA = 0.5f + Va;
    *dB = 0.5f + Vb;
    *dC = 0.5f + Vc;
}
