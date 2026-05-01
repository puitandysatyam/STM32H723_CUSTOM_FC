/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bf_sensor_lsm6dso.h"
#include "bf_pid.h"
#include "bf_mixer.h"
#include "bf_rc_link.h"
#include "bf_dshot.h"
#include "bf_display.h"
#include <stdio.h>
#include <string.h>
// #include "usbd_cdc_if.h" // Uncomment after generating USB via CubeMX (Option A)
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
pidController_t pid_engine;
imuData_t imuData;
motorMixer_t motorOutputs;
bool isImuReady = false;

uint32_t lastDisplayUpdate = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Debug variables to capture sensor responses
volatile uint8_t imu_who_am_i = 0;
volatile HAL_StatusTypeDef baro_ready = HAL_ERROR;
volatile HAL_StatusTypeDef mag_ready = HAL_ERROR;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi4;
extern UART_HandleTypeDef huart1;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  // (Left intentionally blank after purging USB)
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI4_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  // 0. FORCIBLY CLAIM `PE3` AS OUPUT FOR VISUAL HEARTBEAT DIAGNOSTICS!
  __HAL_RCC_GPIOE_CLK_ENABLE();
  GPIO_InitTypeDef led_gpio = {0};
  led_gpio.Pin = GPIO_PIN_3;
  led_gpio.Mode = GPIO_MODE_OUTPUT_PP;
  led_gpio.Pull = GPIO_NOPULL;
  led_gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &led_gpio);

  // 0.5. PERMANENT K1 BUTTON BINDING (PC13 Active-High)
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef k1_gpio = {0};
  k1_gpio.Pin = GPIO_PIN_13;
  k1_gpio.Mode = GPIO_MODE_INPUT;
  k1_gpio.Pull = GPIO_PULLDOWN; // Active-High logic detected!
  HAL_GPIO_Init(GPIOC, &k1_gpio);

  // 1. Initialize Onboard TFT Display (FIRST priority)
  displayInit();

  // 2. BOOT SEQUENCE & PRE-ARM CHECKS
  // Ensure physical sensors and RC link are established before proceeding
  LCD_Fill(0, 0, 160, 80, 0x0000);
  LCD_Print(10, 10, "BOOTING UP...", 0xFFE0); // Yellow
  
  // Wait for IMU to initialize
  while (!lsm6dso_Init()) {
      LCD_Print(10, 30, "IMU: OFFLINE", 0xF800); // Red
      HAL_Delay(500);
  }
  isImuReady = true;
  LCD_Print(10, 30, "IMU: OK     ", 0x07E0); // Green

  /*
  uint8_t spi_tx[2] = {0x0F | 0x80, 0x00}; 
  uint8_t spi_rx[2] = {0};
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, spi_tx, spi_rx, 2, 100);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  imu_who_am_i = spi_rx[1]; 
  */
  
  // 3. Initialize Betaflight Core Engine (dT = 0.000125 for 8kHz pseudo-loop)
  pidInit(&pid_engine, 0.000125f);
  
  // Set "Soft" PID Gains for HITL Simulation Stability (Restored Ki!)
  for(int i=0; i<3; i++) {
      pid_engine.gains[i].Kp = 4.0f;
      pid_engine.gains[i].Ki = 2.0f; 
      pid_engine.gains[i].Kd = 0.0f;
      pid_engine.gains[i].Kf = 10.0f;
  }
  
  mixerInit();
  
  // 4. Mount Sensors 
  // (IMU successfully initialized in Boot Sequence)
  
  // 5. Start RC Link Control Stream (High-Speed DMA UART)
  rcLink_Init();
  
  // 6. Start the DShot600 QuadX TIM1 DMA Esc Output Streams
  dshot_Init();

  // 7. Wait for Remote Controller (RC Link)
  rcData_t *boot_rc = rcLink_GetData();
  while(boot_rc->packetCount == 0) {
      rcLink_Update();
      LCD_Print(10, 50, "RC : WAITING", 0xF800);
      HAL_Delay(100);
  }
  LCD_Print(10, 50, "RC : OK     ", 0x07E0);
  HAL_Delay(1000); // Hold success screen
  LCD_Fill(0, 0, 160, 80, 0x0000); // Clear screen for main loop
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // 1. Process incoming ESP32 RC Stick packets via DMA Ring Buffer asynchronously
    rcLink_Update();
    rcData_t *rc = rcLink_GetData();
    bool isArmed = (rc->aux1 > 1500); // REMOTE ARMING FLAG

    // 2. HARDWARE PID LOOP EXECUTION
    if (isImuReady) {
        // --- 2.1. IMU SENSOR SYNC ---
        // Read physical hardware sensors
        lsm6dso_Read(&imuData);

        // --- 2.2. PID CALCULATIONS & MOTOR MIXING ---
        if (isArmed) {
            // Standard RC Range (1000 - 2000)
            float rollSetpoint  = (rc->roll - 1500) * 0.5f;
            float pitchSetpoint = (rc->pitch - 1500) * 0.5f;
            float yawSetpoint   = -(rc->yaw - 1500) * 0.5f;
            float throttleInput = (rc->throttle - 1000); // 0.0 to 1000.0
            
            pidApply(&pid_engine, ROLL,  imuData.gyro.x, rollSetpoint);
            pidApply(&pid_engine, PITCH, imuData.gyro.y, pitchSetpoint);
            pidApply(&pid_engine, YAW,   imuData.gyro.z, yawSetpoint);
            
            mixQuadX(&pid_engine, throttleInput, &motorOutputs); 
        } else {
            // FORCE IDLE IF DISARMED
            for(int i=0; i<4; i++) motorOutputs.motor[i] = 0;
        }
        
        // dshot_Write(&motorOutputs); // ESC output is blocked for safe bench-testing
    }
    
    // 3. LCD TELEMETRY TICK (10Hz) & HEARTBEAT (1Hz)
    uint32_t ticks = HAL_GetTick();
    static uint32_t lastDisplayUpdate = 0;
    static uint32_t lastHeartbeat = 0;

    // 3.1. K1 BUTTON POLLING (PC13 Active-High)
    static uint32_t lastButtonPress = 0;
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) { // 1 = Pressed!
        if (ticks - lastButtonPress > 200) { // Debounce
            displayNextPage();
            lastButtonPress = ticks;
        }
    }

    // 3.2. Update Display & Telemetry at 10Hz
    if (ticks - lastDisplayUpdate > 100) {
        // Broadcast Consolidated HITL Data (Motors + Telemetry)
        uint16_t motors[4] = {
            (uint16_t)(motorOutputs.motor[0] + 1000), 
            (uint16_t)(motorOutputs.motor[1] + 1000), 
            (uint16_t)(motorOutputs.motor[2] + 1000), 
            (uint16_t)(motorOutputs.motor[3] + 1000)
        };
        rcLink_SendHITL(motors, 1000, 1260); // Motors, Altitude, Battery
        
        displayUpdate();
        lastDisplayUpdate = ticks;
    }
    
    // Heartbeat Pulse
    if (ticks - lastHeartbeat > 500) {
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3);
        lastHeartbeat = ticks;
        
        // --- USB CDC TELEMETRY OUTPUT ---
        // Uncomment the code below AFTER you have generated the USB code via CubeMX.
        /*
        char usbBuf[128];
        sprintf(usbBuf, "IMU[%.2f, %.2f, %.2f] | RC_PKTS: %lu | ARMED: %d\r\n", 
                imuData.gyro.x, imuData.gyro.y, imuData.gyro.z,
                rc->packetCount, isArmed);
        CDC_Transmit_HS((uint8_t*)usbBuf, strlen(usbBuf));
        */
    }
    
    // (HAL_Delay is removed to allow max-frequency PID processing for HITL!) 
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 44;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_0) { // K1 Key (PA0)
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3); // Visual confirmation of IRQ firing!
        displayNextPage();
    }
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  
  // Force PE3 to flash angrily if the board suffers a deadly fault!
  __HAL_RCC_GPIOE_CLK_ENABLE();
  GPIO_InitTypeDef err_led = {0};
  err_led.Pin = GPIO_PIN_3;
  err_led.Mode = GPIO_MODE_OUTPUT_PP;
  err_led.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &err_led);
  
  while (1)
  {
      HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3);
      for(volatile int i=0; i<300000; i++); // Fast strobe block
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
