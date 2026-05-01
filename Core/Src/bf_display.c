#include "bf_display.h"
#include "bf_pid.h"
#include "bf_rc_link.h"
#include <stdio.h>
#include <string.h>

extern SPI_HandleTypeDef hspi4;
extern pidController_t pid_engine;

#include <stdbool.h>
extern bool isImuReady;
extern HAL_StatusTypeDef baro_ready;
extern HAL_StatusTypeDef mag_ready;

// Board Pins matching STMCubeMX Guide
#define BL_PIN    GPIO_PIN_10
#define RST_PIN   GPIO_PIN_9
#define CS_PIN    GPIO_PIN_11
#define DC_PIN    GPIO_PIN_13
#define GPIO_PORT GPIOE

#include "font8x8_basic.h" // Full ASCII 8x8 array

// ------------------------------------
// Low Level Driver
// ------------------------------------
static void LCD_WriteCmd(uint8_t cmd) {
    HAL_GPIO_WritePin(GPIO_PORT, DC_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_PORT, CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi4, &cmd, 1, 10);
    HAL_GPIO_WritePin(GPIO_PORT, CS_PIN, GPIO_PIN_SET);
}

static void LCD_WriteData(uint8_t data) {
    HAL_GPIO_WritePin(GPIO_PORT, DC_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_PORT, CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi4, &data, 1, 10);
    HAL_GPIO_WritePin(GPIO_PORT, CS_PIN, GPIO_PIN_SET);
}

static void LCD_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    uint8_t d[4];
    uint16_t ox = 0, oy = 26; // USER VERIFIED OFFSET
    x0 += ox; x1 += ox; y0 += oy; y1 += oy;
    LCD_WriteCmd(0x2A); d[0]=x0>>8; d[1]=x0&0xFF; d[2]=x1>>8; d[3]=x1&0xFF;
    HAL_GPIO_WritePin(GPIO_PORT, DC_PIN, GPIO_PIN_SET); HAL_GPIO_WritePin(GPIO_PORT, CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi4, d, 4, 10); HAL_GPIO_WritePin(GPIO_PORT, CS_PIN, GPIO_PIN_SET);
    
    LCD_WriteCmd(0x2B); d[0]=y0>>8; d[1]=y0&0xFF; d[2]=y1>>8; d[3]=y1&0xFF;
    HAL_GPIO_WritePin(GPIO_PORT, DC_PIN, GPIO_PIN_SET); HAL_GPIO_WritePin(GPIO_PORT, CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi4, d, 4, 10); HAL_GPIO_WritePin(GPIO_PORT, CS_PIN, GPIO_PIN_SET);
    
    LCD_WriteCmd(0x2C);
}

void LCD_Fill(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    if(x >= 160 || y >= 80) return;
    if(x + w > 160) w = 160 - x;
    if(y + h > 80) h = 80 - y;
    LCD_SetWindow(x, y, x+w-1, y+h-1);
    
    uint8_t p[2] = {color >> 8, color & 0xFF};
    uint8_t line_buffer[320]; // 160px * 2 bytes
    for(int i=0; i<160; i++) {
        line_buffer[i*2] = p[0];
        line_buffer[i*2+1] = p[1];
    }
    
    HAL_GPIO_WritePin(GPIO_PORT, DC_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_PORT, CS_PIN, GPIO_PIN_RESET);
    for(int i=0; i < h; i++) {
        HAL_SPI_Transmit(&hspi4, line_buffer, w*2, 10);
    }
    HAL_GPIO_WritePin(GPIO_PORT, CS_PIN, GPIO_PIN_SET);
}

void LCD_DrawChar8x8(int16_t x, int16_t y, char c, uint16_t color, uint16_t bg) {
    if(x + 8 < 0 || x > 160 || y + 8 < 0 || y > 80) return;
    if (c < 0 || c > 127) c = '?';
    
    uint8_t* glyph = (uint8_t*)font8x8_basic[(uint8_t)c];
    uint16_t char_buf[8 * 8];
    
    for(int i=0; i<8; i++) {
        uint8_t row = glyph[i];
        for(int j=0; j<8; j++) {
            uint16_t p_color = ((row >> j) & 0x01) ? color : bg;
            char_buf[i*8 + j] = (p_color >> 8) | (p_color << 8); // Endian swap
        }
    }
    
    LCD_SetWindow(x, y, x+7, y+7);
    HAL_GPIO_WritePin(GPIO_PORT, DC_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_PORT, CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi4, (uint8_t*)char_buf, 8 * 8 * 2, 50);
    HAL_GPIO_WritePin(GPIO_PORT, CS_PIN, GPIO_PIN_SET);
}

void LCD_Print(int16_t x, int16_t y, const char* str, uint16_t color) {
    int16_t cur_x = x;
    while(*str) {
        LCD_DrawChar8x8(cur_x, y, *str++, color, 0x0000);
        cur_x += 8; // 8px char width
    }
}

// ------------------------------------
// Public API
// ------------------------------------
static volatile uint8_t currentPage = 0;

void displayInit(void) {
    // 0. HARDWARE EDGE ENFORCEMENT
    // Even though CubeMX initializes the pins, we force `SPEED_VERY_HIGH` here to ensure
    // the 10MHz+ SPI signals don't suffer from rounded edges (slew rate limits).
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = RST_PIN | BL_PIN | CS_PIN | DC_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; 
    HAL_GPIO_Init(GPIO_PORT, &GPIO_InitStruct);

    // 0.6. FORCE SPI4 TO EXACT MODE-0 AND PRESCALER 64
    // We disable the peripheral first to ensure the new register values take hold.
    __HAL_SPI_DISABLE(&hspi4);
    hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64; 
    hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
    HAL_SPI_Init(&hspi4);
    __HAL_SPI_ENABLE(&hspi4);
    
    // 0.7. Robust Hardware Reset Sequence (Active-Low Support)
    HAL_GPIO_WritePin(GPIO_PORT, CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_PORT, DC_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_PORT, BL_PIN, GPIO_PIN_SET); // Backlight OFF (Active-Low)
    
    // Hard toggle of the RST line
    HAL_GPIO_WritePin(GPIO_PORT, RST_PIN, GPIO_PIN_RESET); HAL_Delay(200);
    HAL_GPIO_WritePin(GPIO_PORT, RST_PIN, GPIO_PIN_SET);   HAL_Delay(200);
    
    // ST7789 Software Init (Standard Seq)
    LCD_WriteCmd(0x01); HAL_Delay(150); // SW Reset
    LCD_WriteCmd(0x11); HAL_Delay(150); // Sleep Out
    LCD_WriteCmd(0x3A); LCD_WriteData(0x05); // 16-bit colors
    LCD_WriteCmd(0x36); LCD_WriteData(0xA8); // Landscape (Flipped per User Guide)
    LCD_WriteCmd(0x21); // Invert ON
    LCD_WriteCmd(0x29); HAL_Delay(100); // Display ON
    
    HAL_GPIO_WritePin(GPIO_PORT, BL_PIN, GPIO_PIN_RESET); // Backlight ON (Active-Low)
    
    // Fill with CYAN success indicator
    LCD_Fill(0, 0, 160, 80, 0x07FF); 
    HAL_Delay(500);
    LCD_Fill(0, 0, 160, 80, 0x0000); // Back to black
}

void displayNextPage(void) {
    currentPage = (currentPage + 1) % 3;
    LCD_Fill(0, 0, 160, 80, 0x0000); // Clear screen on page swap (Corrected Dimensions)
}

void displayUpdate(void) {
    char buf[32];
    
    // HUD: Current Page Indicator (Top Right)
    sprintf(buf, "P%d", currentPage + 1);
    LCD_Print(140, 5, buf, 0x7BEF); // Grey/Blue HUD
    
    if (currentPage == 0) {
        // DIAGNOSTICS PAGE
        LCD_Print(2,  5, "--- SYSTEM STATUS ---", 0x07E0); // Green
        
        if (isImuReady) LCD_Print(10, 25, "IMU  : READY", 0xFFFF);
        else           LCD_Print(10, 25, "IMU  : OFFLINE", 0xF800);
        
        if (baro_ready == HAL_OK) LCD_Print(10, 45, "BARO : READY", 0xFFFF);
        else                      LCD_Print(10, 45, "BARO : OFFLINE", 0xF800);
        
        if (mag_ready == HAL_OK)  LCD_Print(10, 65, "MAG  : READY", 0xFFFF);
        else                      LCD_Print(10, 65, "MAG  : OFFLINE", 0xF800);
    } 
    else if (currentPage == 1) {
        // PID GYRO TELEMETRY PAGE
        LCD_Print(2,  5, "--- GYRO VECTORS ---", 0x07FF); // Cyan
        sprintf(buf, "ROLL : %4d d/s", (int)pid_engine.previousGyroRate[0]);
        LCD_Print(10, 25, buf, 0xFFFF);
        sprintf(buf, "PITCH: %4d d/s", (int)pid_engine.previousGyroRate[1]);
        LCD_Print(10, 45, buf, 0xFFFF);
        sprintf(buf, "YAW  : %4d d/s", (int)pid_engine.previousGyroRate[2]);
        LCD_Print(10, 65, buf, 0xFFFF);
    }
    else if (currentPage == 2) {
        // RECEIVER PAGE (Page 3)
        LCD_Print(2,  5, "--- ESP32 LINK ---", 0xFFE0); // Yellow
        
        rcData_t *rc = rcLink_GetData();
        bool armed = (rc->aux1 > 1500);
        if (armed) {
            LCD_Print(10, 25, "STATE: ARMED    ", 0xF800); // Red
        } else {
            LCD_Print(10, 25, "STATE: DISARMED ", 0x07E0); // Green
        }
        LCD_Print(10, 40, "MODE : UDP WI-TL", 0xFFFF);
        
        sprintf(buf, "RC   : %lu PKTS", rc->packetCount);
        LCD_Print(10, 55, buf, 0xFFFF);
        sprintf(buf, "DMA  : %lu BYTES", rc->rawByteCount); 
        LCD_Print(10, 68, buf, 0x07FF); // Cyan
    }
}
