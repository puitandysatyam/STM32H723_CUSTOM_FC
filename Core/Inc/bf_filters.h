#ifndef BF_FILTERS_H
#define BF_FILTERS_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#ifndef M_PIf
#define M_PIf 3.14159265358979323846f
#endif

#define sq(x) ((x)*(x))

typedef struct {
    float state;
    float k;
} pt1Filter_t;

typedef enum {
    FILTER_LPF,
    FILTER_NOTCH,
    FILTER_BPF,
} biquadFilterType_e;

typedef struct {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
    float weight;
} biquadFilter_t;

// PT1 Filter API
float pt1FilterGain(float f_cut, float dT);
void pt1FilterInit(pt1Filter_t *filter, float k);
void pt1FilterUpdateCutoff(pt1Filter_t *filter, float k);
float pt1FilterApply(pt1Filter_t *filter, float input);

// Biquad Filter API
void biquadFilterInitLPF(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate);
void biquadFilterInit(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType, float weight);
void biquadFilterUpdate(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType, float weight);
float biquadFilterApply(biquadFilter_t *filter, float input);
float biquadFilterApplyDF1(biquadFilter_t *filter, float input);

#endif // BF_FILTERS_H
