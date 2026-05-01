#include "bf_hitl.h"
#include <string.h>

// Extern the raw STMCube USB transmit function
extern uint8_t CDC_Transmit_HS(uint8_t* Buf, uint16_t Len);

static imuData_t simulatedImu;
static bool hasNewHitlData = false;

// ---------------------------------------------------------
// Called automatically from deep inside the USB CDC Interrupt
// ---------------------------------------------------------
void hitl_rx_callback(uint8_t *Buf, uint32_t Len) {
    // The Python bridge sends 12 bytes (3 floats: GyroX, GyroY, GyroZ)
    if (Len >= 12) {
        // Blistering fast memory copy straight into the physics struct
        memcpy(&simulatedImu.gyro.x, Buf, 12); 
        hasNewHitlData = true;
    }
}

// ---------------------------------------------------------
// Replaces the physical SPI read function in your main.c loop!
// ---------------------------------------------------------
bool hitl_read_imu(imuData_t *data) {
    if (hasNewHitlData) {
        data->gyro = simulatedImu.gyro;
        hasNewHitlData = false;
        return true;
    }
    return false;
}

// ---------------------------------------------------------
// Pushes the calculated motor speeds back to the 3D Physics engine
// ---------------------------------------------------------
void hitl_tx_motors(motorMixer_t *mix) {
    // mix->motor[0] is array of 4 floats. (4 * 4 bytes = 16 bytes payload)
    CDC_Transmit_HS((uint8_t*)&mix->motor[0], 16);
}
