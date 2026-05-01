#ifndef BF_HITL_H
#define BF_HITL_H

#include <stdint.h>
#include <stdbool.h>
#include "bf_sensor_lsm6dso.h" // Gives us imuData_t
#include "bf_mixer.h"          // Gives us motorMixer_t

// Intercepts raw USB bytes and places them in an intermediate buffer
void hitl_rx_callback(uint8_t *Buf, uint32_t Len);

// Reads the intermediate buffer (returns true if new data arrived from PC)
bool hitl_read_imu(imuData_t *data);

// Transmits the final 4 motor speeds back over the USB to the PC simulator
void hitl_tx_motors(motorMixer_t *mix);

#endif // BF_HITL_H
