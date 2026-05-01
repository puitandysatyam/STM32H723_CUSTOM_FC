#include "bf_mixer.h"

#define CONSTRAIN(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

void mixerInit(void) {
    // Expansion hook if advanced custom motor mixing mappings are needed later
}

// -------------------------------------------------------------
// Applies the standard QuadX Betaflight Mixing Matrix
// -------------------------------------------------------------
void mixQuadX(pidController_t *pid, float throttle, motorMixer_t *output) {
    // Throttle is expected to be roughly 0.0f to 1000.0f
    
    // Grab the final payload calculation from the PID loop
    float pitchSum = pid->data[PITCH].Sum;
    float rollSum  = pid->data[ROLL].Sum;
    float yawSum   = pid->data[YAW].Sum;

    // Betaflight standard Quad-X Geometry Mappings (Yaw Inverted for Negative Feedback)
    // Motor 1: Rear Right (Counter-Clockwise)
    output->motor[0] = throttle - pitchSum - rollSum - yawSum; 
    
    // Motor 2: Front Right (Clockwise)
    output->motor[1] = throttle + pitchSum - rollSum + yawSum;
    
    // Motor 3: Rear Left (Clockwise)
    output->motor[2] = throttle - pitchSum + rollSum + yawSum;
    
    // Motor 4: Front Left (Counter-Clockwise)
    output->motor[3] = throttle + pitchSum + rollSum - yawSum;

    // AirMode style lower limit protection
    for(int i = 0; i < 4; i++) {
        output->motor[i] = CONSTRAIN(output->motor[i], 0.0f, 1000.0f);
    }
}
