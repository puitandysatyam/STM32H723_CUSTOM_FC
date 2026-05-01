#include "bf_pid.h"

#define CONSTRAIN(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

void pidInit(pidController_t *pid, float dT) {
    pid->dT = dT;
    pid->itermLimit = 250.0f;
    pid->dtermLimit = 400.0f;
    pid->pidSumLimit = 500.0f;

    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        pid->data[i].P = 0.0f;
        pid->data[i].I = 0.0f;
        pid->data[i].D = 0.0f;
        pid->data[i].F = 0.0f;
        pid->data[i].Sum = 0.0f;
        
        pid->previousGyroRate[i] = 0.0f;
        pid->previousSetpoint[i] = 0.0f;

        // Initialize D-term Lowpass filters to safer ~50Hz and 80Hz cascade (SITL Tuned)
        pt1FilterInit(&pid->dtermLpf[i], pt1FilterGain(50.0f, dT));
        pt1FilterInit(&pid->dtermLpf2[i], pt1FilterGain(80.0f, dT));
    }
}

// -------------------------------------------------------------
// The legendary Betaflight core logic: Proportional, Integral, 
// Derivative (Measurement based), and smart FeedForward.
// -------------------------------------------------------------
void pidApply(pidController_t *pid, axis_e axis, float gyroRate, float setpoint) {
    const float dT = pid->dT;
    pidGains_t *gains = &pid->gains[axis];
    pidAxisData_t *data = &pid->data[axis];

    // 1. Error calculation
    const float error = setpoint - gyroRate;

    // 2. P-Term
    data->P = error * gains->Kp * PTERM_SCALE;

    // 3. I-Term (with aggressive anti-windup limits)
    data->I += error * gains->Ki * ITERM_SCALE * dT;
    data->I = CONSTRAIN(data->I, -pid->itermLimit, pid->itermLimit);

    // 4. D-Term (Error derivative based strictly on gyro measurement 
    // to prevent aggressive "setpoint kick" when smashing the sticks)
    float deltaGyro = gyroRate - pid->previousGyroRate[axis];
    pid->previousGyroRate[axis] = gyroRate;
    
    float dTermRaw = -(deltaGyro / dT) * gains->Kd * DTERM_SCALE;
    
    // Apply D-Term Lowpass PT1 cascade to kill high-frequency vibration
    dTermRaw = pt1FilterApply(&pid->dtermLpf[axis], dTermRaw);
    dTermRaw = pt1FilterApply(&pid->dtermLpf2[axis], dTermRaw);
    data->D = CONSTRAIN(dTermRaw, -pid->dtermLimit, pid->dtermLimit);

    // 5. Feedforward (F-Term) pushes the quad into the turn *before* error accumulates!
    float deltaSetpoint = setpoint - pid->previousSetpoint[axis];
    pid->previousSetpoint[axis] = setpoint;
    
    data->F = (deltaSetpoint / dT) * gains->Kf * FEEDFORWARD_SCALE;

    // 6. Final Mixer Payload
    data->Sum = data->P + data->I + data->D + data->F;
    data->Sum = CONSTRAIN(data->Sum, -pid->pidSumLimit, pid->pidSumLimit);
}

void pidResetIterm(pidController_t *pid) {
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        pid->data[i].I = 0.0f;
    }
}
