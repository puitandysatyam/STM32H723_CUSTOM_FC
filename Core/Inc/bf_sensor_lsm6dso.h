#ifndef BF_SENSOR_LSM6DSO_H
#define BF_SENSOR_LSM6DSO_H

#include <stdint.h>
#include <stdbool.h>

// Standard 3D spatial data structure
typedef struct {
    float x; // Mapped to Roll
    float y; // Mapped to Pitch
    float z; // Mapped to Yaw
} sensorData_t;

// The unified packet sent to the Betaflight filters
typedef struct {
    sensorData_t gyro;  // Real-world Degrees per Second (dps)
    sensorData_t accel; // Real-world G-Forces (1.0 = 9.81 m/s^2)
} imuData_t;

bool lsm6dso_Init(void);
bool lsm6dso_Read(imuData_t *data);

#endif // BF_SENSOR_LSM6DSO_H
