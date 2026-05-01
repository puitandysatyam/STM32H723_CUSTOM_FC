#include "bf_filters.h"

#define BIQUAD_Q (1.0f / sqrtf(2.0f)) // Quality factor for 2nd order Butterworth

// ---------------------------------------------------------
// PT1 Low Pass Filter
// ---------------------------------------------------------
float pt1FilterGain(float f_cut, float dT) {
    float omega = 2.0f * M_PIf * f_cut * dT;
    return omega / (omega + 1.0f);
}

void pt1FilterInit(pt1Filter_t *filter, float k) {
    filter->state = 0.0f;
    filter->k = k;
}

void pt1FilterUpdateCutoff(pt1Filter_t *filter, float k) {
    filter->k = k;
}

float pt1FilterApply(pt1Filter_t *filter, float input) {
    filter->state = filter->state + filter->k * (input - filter->state);
    return filter->state;
}

// ---------------------------------------------------------
// Biquad Filter
// ---------------------------------------------------------
void biquadFilterInitLPF(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate) {
    biquadFilterInit(filter, filterFreq, refreshRate, BIQUAD_Q, FILTER_LPF, 1.0f);
}

void biquadFilterInit(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType, float weight) {
    biquadFilterUpdate(filter, filterFreq, refreshRate, Q, filterType, weight);
    filter->x1 = filter->x2 = 0;
    filter->y1 = filter->y2 = 0;
}

void biquadFilterUpdate(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType, float weight) {
    const float omega = 2.0f * M_PIf * filterFreq * refreshRate * 0.000001f;
    
    // Using hardware double-precision FPU math (sinf/cosf) instead of Betaflight's 
    // sin_approx/cos_approx macros perfectly leverages the STM32H7's raw computing power!
    const float sn = sinf(omega);
    const float cs = cosf(omega);
    const float alpha = sn / (2.0f * Q);

    switch (filterType) {
    case FILTER_LPF:
        filter->b1 = 1 - cs;
        filter->b0 = filter->b1 * 0.5f;
        filter->b2 = filter->b0;
        filter->a1 = -2 * cs;
        filter->a2 = 1 - alpha;
        break;
    case FILTER_NOTCH:
        filter->b0 = 1;
        filter->b1 = -2 * cs;
        filter->b2 = 1;
        filter->a1 = filter->b1;
        filter->a2 = 1 - alpha;
        break;
    case FILTER_BPF:
        filter->b0 = alpha;
        filter->b1 = 0;
        filter->b2 = -alpha;
        filter->a1 = -2 * cs;
        filter->a2 = 1 - alpha;
        break;
    }

    const float a0 = 1 + alpha;
    filter->b0 /= a0;
    filter->b1 /= a0;
    filter->b2 /= a0;
    filter->a1 /= a0;
    filter->a2 /= a0;
    filter->weight = weight;
}

// Computes Direct Form II (Fast but vulnerable to coefficient shifts mid-flight)
float biquadFilterApply(biquadFilter_t *filter, float input) {
    const float result = filter->b0 * input + filter->x1;
    filter->x1 = filter->b1 * input - filter->a1 * result + filter->x2;
    filter->x2 = filter->b2 * input - filter->a2 * result;
    return result;
}

// Computes Direct Form I (High precision, dynamic updates supported!)
float biquadFilterApplyDF1(biquadFilter_t *filter, float input) {
    const float result = filter->b0 * input + filter->b1 * filter->x1 + filter->b2 * filter->x2 - filter->a1 * filter->y1 - filter->a2 * filter->y2;
    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = result;
    return result;
}
