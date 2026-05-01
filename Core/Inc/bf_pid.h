#ifndef BF_PID_H
#define BF_PID_H

#include <stdint.h>
#include "bf_filters.h"

// Standard Betaflight scaling architecture
#define PTERM_SCALE 0.032029f
#define ITERM_SCALE 0.244381f
#define DTERM_SCALE 0.000529f
#define FEEDFORWARD_SCALE 0.013754f

typedef enum {
    ROLL = 0,
    PITCH,
    YAW,
    XYZ_AXIS_COUNT
} axis_e;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float Kf;
} pidGains_t;

typedef struct {
    float P;
    float I;
    float D;
    float F;
    float Sum;
} pidAxisData_t;

// The central context object for the flight control loop
typedef struct {
    float dT; // The delta time step (e.g., 0.000125 for 8kHz)
    float itermLimit;
    float dtermLimit;
    float pidSumLimit;
    
    pidGains_t gains[XYZ_AXIS_COUNT];
    pidAxisData_t data[XYZ_AXIS_COUNT];

    float previousGyroRate[XYZ_AXIS_COUNT];
    float previousSetpoint[XYZ_AXIS_COUNT];
    
    // D-Term Lowpass Filters to destroy high-frequency gyro noise
    pt1Filter_t dtermLpf[XYZ_AXIS_COUNT];
    pt1Filter_t dtermLpf2[XYZ_AXIS_COUNT];
    
} pidController_t;

void pidInit(pidController_t *pid, float dT);
void pidApply(pidController_t *pid, axis_e axis, float gyroRate, float setpoint);
void pidResetIterm(pidController_t *pid);

#endif // BF_PID_H
