#include "bf_dshot.h"
#include "tim.h" // Gives us access to `htim4`

#define MOTOR_COUNT 4

// The 4 memory arrays that the STM32 DMA engine will automatically push into the Timer's Capture-Compare Registers
static uint32_t dmaBuffer[MOTOR_COUNT][DSHOT_DMA_BUFFER_SIZE];

// ---------------------------------------------------------
// Calculates the standard 4-bit DShot CRC
// ---------------------------------------------------------
static uint8_t dshot_crc(uint16_t data) {
    uint8_t crc = (data ^ (data >> 4) ^ (data >> 8)) & 0x0F;
    return crc;
}

// ---------------------------------------------------------
// Translates the 16-bit payload into raw PWM Duty Cycles
// ---------------------------------------------------------
static void dshot_prepare_dma_buffer(uint32_t arr, uint16_t packet, uint32_t *buffer) {
    // DShot Digital 0 is geometrically ~37.5% duty cycle
    uint32_t bit0 = (arr * 3) >> 3;
    
    // DShot Digital 1 is geometrically ~75% duty cycle
    uint32_t bit1 = (arr * 6) >> 3;

    for (int i = 0; i < 16; i++) {
        // DShot transmits the Most Significant Bit (MSB) first
        if (packet & (0x8000 >> i)) {
            buffer[i] = bit1;
        } else {
            buffer[i] = bit0;
        }
    }
    
    // Append 2 distinct zeros to separate DShot packets gracefully
    buffer[16] = 0;
    buffer[17] = 0;
}

// ---------------------------------------------------------
// Fires up the 4 Advanced Timer Hardware DMA Streams
// ---------------------------------------------------------
void dshot_Init(void) {
    // These channel mappings match the PD12, PD13, PD14, PD15 pins from your re-routed schematic!
    HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_1, dmaBuffer[0], DSHOT_DMA_BUFFER_SIZE);
    HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_2, dmaBuffer[1], DSHOT_DMA_BUFFER_SIZE);
    HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_3, dmaBuffer[2], DSHOT_DMA_BUFFER_SIZE);
    HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_4, dmaBuffer[3], DSHOT_DMA_BUFFER_SIZE);
}

// ---------------------------------------------------------
// Mixes, formats, and executes the mathematical payload
// ---------------------------------------------------------
void dshot_Write(motorMixer_t *mixerOutputs) {
    // Dynamically grab the Auto-Reload-Register (ARR) assigned by CubeMX 
    // This allows the clock speed to shift without breaking the DShot math!
    uint32_t arr = htim4.Instance->ARR;

    for (int i = 0; i < MOTOR_COUNT; i++) {
        uint16_t throttle = 0;
        
        // Safety guard against negative motor output anomalies
        if (mixerOutputs->motor[i] > 0.0f) {
            // Our mixer outputs 0-1000. DShot expects 48-2047 (0-47 are reserved command integers)
            throttle = 48 + (uint16_t)(mixerOutputs->motor[i] * 1.999f); 
            if (throttle > 2047) throttle = 2047;
        }

        // Construct raw 16-bit payload: [11-bit Throttle] + [1-bit Telemetry Req(0)]
        uint16_t packet = (throttle << 1) | 0; 
        
        // Inject 4-bit mathematical integrity Checksum
        uint8_t crc = dshot_crc(packet);
        packet = (packet << 4) | crc;

        // Push to memory array (The DMA hardware takes over automatically!)
        dshot_prepare_dma_buffer(arr, packet, dmaBuffer[i]);
    }
}
