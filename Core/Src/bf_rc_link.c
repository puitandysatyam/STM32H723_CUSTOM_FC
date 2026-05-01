#include "bf_rc_link.h"
#include "usart.h"

// Define a large RX buffer for STM32 Circular DMA
#define RX_BUFFER_SIZE 64

static uint8_t rxBuffer[RX_BUFFER_SIZE];

// Init with safe centered un-armed values
static rcData_t currentRcData = {1000, 1500, 1500, 1500, 1000, 1000, false};

// Serial State Machine 
typedef enum { STATE_WAIT_H1, STATE_WAIT_H2, STATE_PAYLOAD, STATE_CHECKSUM } parserState_e;

static sensorInjected_t currentInjectedSensors = {0};
static uint8_t activeHeader = 0;

// ---------------------------------------------------------
// Fires up the Circular DMA ring buffer
// ---------------------------------------------------------
void rcLink_Init(void) {
    // Reset Counters
    currentRcData.packetCount = 0;
    currentRcData.sensorCount = 0;
    currentRcData.rawByteCount = 0;
    currentRcData.isNewDataReady = false;
    
    // Set Safe Defaults (Throttle 1000, others 1500)
    currentRcData.throttle = 1000;
    currentRcData.roll = 1500;
    currentRcData.pitch = 1500;
    currentRcData.yaw = 1500;
    currentRcData.aux1 = 1000;

    // Start DMA receiver in Circular Mode
    HAL_UART_Receive_DMA(&huart1, rxBuffer, RX_BUFFER_SIZE);
}

// ---------------------------------------------------------
// Scans the DMA ring buffer asynchronously for 0xAA 0x55
// ---------------------------------------------------------
void rcLink_Update(void) {
    static uint16_t readIdx = 0;
    static parserState_e state = STATE_WAIT_H1;
    static uint8_t payload[12];
    static uint8_t payloadIdx = 0;
    static uint8_t calculatedChecksum = 0;

    // Calculate where the hardware DMA engine is currently resting in memory
    uint16_t writeIdx = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);

    // Drain and process the delta
    while (readIdx != writeIdx) {
        uint8_t byte = rxBuffer[readIdx];
        currentRcData.rawByteCount++; // TRACK EVERY BYTE ARRIVING IN HARDWARE DMA
        
        switch (state) {
            case STATE_WAIT_H1:
                if (byte == 0xAA) {
                    state = STATE_WAIT_H2;
                }
                break;
            case STATE_WAIT_H2:
                if (byte == 0xCC || byte == 0xAA) {
                    activeHeader = byte;
                    state = STATE_PAYLOAD;
                    payloadIdx = 0;
                    calculatedChecksum = 0;
                } else {
                    state = STATE_WAIT_H1;
                }
                break;
            case STATE_PAYLOAD:
                payload[payloadIdx++] = byte;
                calculatedChecksum ^= byte;
                if (payloadIdx == 12) state = STATE_CHECKSUM;
                break;
            case STATE_CHECKSUM:
                if (byte == calculatedChecksum) {
                    if (activeHeader == 0xAA) {
                        // Integrity passed! Lock in the RC parameters (1000-2000 mapping)
                        currentRcData.throttle = (payload[0] << 8) | payload[1];
                        currentRcData.roll     = (payload[2] << 8) | payload[3];
                        currentRcData.pitch    = (payload[4] << 8) | payload[5];
                        currentRcData.yaw      = (payload[6] << 8) | payload[7];
                        currentRcData.aux1     = (payload[8] << 8) | payload[9];
                        currentRcData.aux2     = (payload[10] << 8) | payload[11];
                        currentRcData.packetCount++; 
                        currentRcData.isNewDataReady = true;
                    } else if (activeHeader == 0xCC) {
                        // HITL Sensor Injection
                        currentInjectedSensors.gyro[0] = (int16_t)((payload[0] << 8) | payload[1]);
                        currentInjectedSensors.gyro[1] = (int16_t)((payload[2] << 8) | payload[3]);
                        currentInjectedSensors.gyro[2] = (int16_t)((payload[4] << 8) | payload[5]);
                        currentInjectedSensors.accel[0] = (int16_t)((payload[6] << 8) | payload[7]);
                        currentInjectedSensors.accel[1] = (int16_t)((payload[8] << 8) | payload[9]);
                        currentInjectedSensors.accel[2] = (int16_t)((payload[10] << 8) | payload[11]);
                        currentRcData.sensorCount++;
                    }
                }
                state = STATE_WAIT_H1; // Restart scan
                break;
        }
        readIdx = (readIdx + 1) % RX_BUFFER_SIZE;
    }
}

rcData_t* rcLink_GetData(void) {
    return &currentRcData;
}

sensorInjected_t* rcLink_GetInjectedSensors(void) {
    return &currentInjectedSensors;
}

// -------------------------------------------------------------
// Combined HITL feedback to prevent DMA collisions
// 0xFF (Header), 0x55, 16 Bytes Payload, 1 Byte Checksum = 19 Bytes
// -------------------------------------------------------------
void rcLink_SendHITL(uint16_t *motors, uint16_t altitude, uint16_t battery) {
    if (huart1.gState != HAL_UART_STATE_READY) return;

    static uint8_t hitlTx[19];
    hitlTx[0] = 0xFF;
    hitlTx[1] = 0x55;
    
    // Motors (8 bytes)
    for(int i=0; i<4; i++) {
        hitlTx[2 + i*2] = (motors[i] >> 8) & 0xFF;
        hitlTx[3 + i*2] = motors[i] & 0xFF;
    }
    
    // Telemetry (4 bytes)
    hitlTx[10] = (altitude >> 8) & 0xFF; hitlTx[11] = altitude & 0xFF;
    hitlTx[12] = (battery >> 8) & 0xFF;  hitlTx[13] = battery & 0xFF;
    
    // Padding/Reserved (4 bytes)
    hitlTx[14] = 0; hitlTx[15] = 0; hitlTx[16] = 0; hitlTx[17] = 0;
    
    // Checksum
    uint8_t cs = 0;
    for(int i=2; i<18; i++) cs ^= hitlTx[i];
    hitlTx[18] = cs;

    SCB_CleanDCache_by_Addr((uint32_t*)hitlTx, 19);
    HAL_UART_Transmit_DMA(&huart1, hitlTx, 19);
}
