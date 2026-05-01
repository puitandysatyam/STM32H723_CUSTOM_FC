#include "bf_sensor_lsm6dso.h"
#include "spi.h"
#include "gpio.h"

extern SPI_HandleTypeDef hspi1;

// Core LSM6DSO Hardware Registers
#define LSM6DSO_WHO_AM_I         0x0F
#define LSM6DSO_CTRL1_XL         0x10
#define LSM6DSO_CTRL2_G          0x11
#define LSM6DSO_OUTX_L_G         0x22

// ---------------------------------------------------------
// Low-Level SPI Wrapper Functions
// ---------------------------------------------------------
static void spiWriteReg(uint8_t reg, uint8_t data) {
    uint8_t tx[2] = {reg & 0x7F, data};
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, tx, 2, 10);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
}

static void spiReadRegs(uint8_t reg, uint8_t *buffer, uint16_t length) {
    uint8_t tx = reg | 0x80; // Set the Read bit (MSB = 1)
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &tx, 1, 10);
    HAL_SPI_Receive(&hspi1, buffer, length, 10);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
}

// ---------------------------------------------------------
// Initialization & Configuration
// ---------------------------------------------------------
bool lsm6dso_Init(void) {
    uint8_t whoAmI = 0;
    spiReadRegs(LSM6DSO_WHO_AM_I, &whoAmI, 1);
    
    // Validate we are actually talking to an LSM6DSO
    if (whoAmI != 0x6C) {
        return false; 
    }
    
    // Configure Gyroscope: 6.66kHz Sample Rate, 2000 dps full scale
    spiWriteReg(LSM6DSO_CTRL2_G, 0xAC); 
    
    // Configure Accelerometer: 6.66kHz Sample Rate, 16G full scale
    spiWriteReg(LSM6DSO_CTRL1_XL, 0xA4);
    
    return true;
}

// ---------------------------------------------------------
// High-Speed Data Acquisition & Formatting
// ---------------------------------------------------------
bool lsm6dso_Read(imuData_t *data) {
    uint8_t buffer[12];
    
    // Burst read 12 registers sequentially (6 Gyro bytes + 6 Accel bytes) starting from OUTX_L_G
    spiReadRegs(LSM6DSO_OUTX_L_G, buffer, 12);

    // Reconstruct the 16-bit 2's complement integers from raw LSB buffers
    int16_t gyroX = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t gyroY = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t gyroZ = (int16_t)((buffer[5] << 8) | buffer[4]);

    int16_t accelX = (int16_t)((buffer[7] << 8) | buffer[6]);
    int16_t accelY = (int16_t)((buffer[9] << 8) | buffer[8]);
    int16_t accelZ = (int16_t)((buffer[11] << 8) | buffer[10]);

    // Scale Gyro to Degrees Per Second (2000 dps scale = 70 mdps/LSB = 0.07 dps multiplier)
    data->gyro.x = (float)gyroX * 0.07f;
    data->gyro.y = (float)gyroY * 0.07f;
    data->gyro.z = (float)gyroZ * 0.07f;

    // Scale Accel to standard G-Force (16g scale = 0.488 mg/LSB = 0.000488 g multiplier)
    data->accel.x = (float)accelX * 0.000488f;
    data->accel.y = (float)accelY * 0.000488f;
    data->accel.z = (float)accelZ * 0.000488f;

    return true;
}
