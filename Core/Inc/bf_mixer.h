#ifndef BF_MIXER_H
#define BF_MIXER_H

#include "bf_pid.h"

// Quad X Motor output mapping structure
typedef struct {
    float motor[4]; // Scaled 0.0f to 1000.0f (which we will map directly to DShot ranges later)
} motorMixer_t;

void mixerInit(void);
void mixQuadX(pidController_t *pid, float throttle, motorMixer_t *output);

#endif // BF_MIXER_H
