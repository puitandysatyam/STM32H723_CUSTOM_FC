#ifndef BF_RC_LINK_H
#define BF_RC_LINK_H

#include <stdint.h>
#include <stdbool.h>

// Struct representing the live 1000-2000 channel sticks from the ESP32
typedef struct {
    uint16_t throttle;
    uint16_t roll;
    uint16_t pitch;
    uint16_t yaw;
    uint16_t aux1;
    uint16_t aux2;
    uint32_t packetCount;  // RC Packet Counter
    uint32_t rawByteCount; // NEW: Raw DMA Byte Tracker
    uint32_t sensorCount;  // HITL Sensor Frame Counter
    bool isNewDataReady;
} rcData_t;

// Simulated Sensor Data for HITL
typedef struct {
    int16_t gyro[3];   // X, Y, Z (Signed)
    int16_t accel[3];  // X, Y, Z
} sensorInjected_t;

// Telemetry payload to push back to the ground
typedef struct {
    uint16_t altitude;
    uint16_t battery;
} telemetryData_t;

void rcLink_Init(void);
void rcLink_Update(void);
rcData_t* rcLink_GetData(void);
sensorInjected_t* rcLink_GetInjectedSensors(void);

// Consolidated HITL Feedback
void rcLink_SendHITL(uint16_t *motors, uint16_t altitude, uint16_t battery);

#endif // BF_RC_LINK_H
