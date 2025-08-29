/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include "stm32f1xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Telemetry data structure
typedef struct {
    float altitude;
    float pressure;
    float accel_x;
    float accel_y;
    float accel_z;
    float angle_x;
    float angle_y;
    float angle_z;
} TelemetryData_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TELEMETRY_HEADER 0xAB
#define FOOTER_1 0x0D
#define FOOTER_2 0x0A
#define PACKET_SIZE 36

// New status message format
#define STATUS_HEADER 0xAA
#define STATUS_FOOTER_1 0x0D
#define STATUS_FOOTER_2 0x0A
#define STATUS_MESSAGE_SIZE 6

// Status bit definitions (sequential activation from LSB to MSB)
#define STATUS_ROCKET_FIRED_BIT      (1 << 0)  // Bit 0 - FIRST
#define STATUS_WAITED_5SN_BIT        (1 << 1)  // Bit 1 - SECOND
#define STATUS_MIN_ALTITUDE_BIT      (1 << 2)  // Bit 2 - THIRD
#define STATUS_ANGLE_EXCEEDED_BIT    (1 << 3)  // Bit 3 - FOURTH
#define STATUS_ALTITUDE_DECREASING_BIT (1 << 4) // Bit 4 - FIFTH
#define STATUS_FIRST_PARACHUTE_BIT   (1 << 5)  // Bit 5 - SIXTH (moved from bit 6)
#define STATUS_ALTITUDE_550_BIT      (1 << 6)  // Bit 6 - SEVENTH (moved from bit 5)
#define STATUS_SECOND_PARACHUTE_BIT  (1 << 7)  // Bit 7 - LAST
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
TelemetryData_t telemetry_data = {0};
uint8_t rx_buffer[PACKET_SIZE];
uint8_t packet_received = 0;

// Simple buffer for incoming data
uint8_t incoming_buffer[PACKET_SIZE * 4] = {0}; // Increased to 4x packet size for better buffering
uint16_t buffer_index = 0;
uint8_t uart_busy = 0; // Flag to prevent conflicts

// Timer variables
TIM_HandleTypeDef htim2;
uint32_t timer_tick = 0;
uint32_t packets_received_count = 0; // Counter for received packets

// Status tracking variables
uint8_t current_status_byte = 0x00;
float previous_altitude = 0.0f;
uint32_t rocket_fired_timestamp = 0;
uint8_t status_sent = 0; // Flag to track if status was already sent

// Filtering variables for telemetry data
#define FILTER_SIZE 5
float altitude_filter[FILTER_SIZE] = {0};
float accel_z_filter[FILTER_SIZE] = {0};
float angle_x_filter[FILTER_SIZE] = {0};
float angle_y_filter[FILTER_SIZE] = {0};
uint8_t filter_index = 0;
uint8_t filter_filled = 0; // Flag to indicate if filter is filled
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void ParseTelemetryPacket(uint8_t* packet);
void PrintTelemetryData(void);
void ProcessIncomingData(uint8_t new_byte);
static void MX_TIM2_Init(void);
void TimerCallback(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void UpdateStatusFromTelemetry(void);
void SendStatusMessage(void);
uint8_t CalculateChecksum(uint8_t* data, uint8_t length);
void ApplyFiltering(void);
float GetFilteredValue(float* filter_array);
uint8_t status_message[STATUS_MESSAGE_SIZE];
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  // Enable global interrupts
  __enable_irq();
  
  // Start interrupt-driven UART reception on UART1
  HAL_UART_Receive_IT(&huart1, &incoming_buffer[buffer_index], 1);

  // Configure NVIC for TIM2 interrupt
  HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  
  // Configure NVIC for UART1 interrupt
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  
  // Start timer interrupt (every 5ms for better performance)
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Check if we have received data
    if (packet_received) {
      packet_received = 0;

      // Process the received packet
      ParseTelemetryPacket(rx_buffer);
      // Apply filtering to telemetry values
      ApplyFiltering();
      // Update status based on telemetry data
      UpdateStatusFromTelemetry();
      // Send status message instead of telemetry data
      SendStatusMessage();


    }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */
  // UART1 configured for both TX and RX (115200 baud, 8N1)
  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief Parse telemetry packet from UART
 * @param packet: Pointer to 36-byte packet
 */
void ParseTelemetryPacket(uint8_t* packet)
{
    // Check header and footer
    if (packet[0] != TELEMETRY_HEADER || 
        packet[34] != FOOTER_1 || 
        packet[35] != FOOTER_2) {
        return;
    }
    
    // Parse big-endian float values - need to reverse byte order for ARM (little-endian)
    // Altitude (bytes 2-5) - reverse order: 4,3,2,1
    uint8_t alt_bytes[4] = {packet[4], packet[3], packet[2], packet[1]};
    memcpy(&telemetry_data.altitude, alt_bytes, 4);
    
    // Pressure (bytes 6-9) - reverse order: 8,7,6,5
    uint8_t press_bytes[4] = {packet[8], packet[7], packet[6], packet[5]};
    memcpy(&telemetry_data.pressure, press_bytes, 4);
    
    // Acceleration X (bytes 10-13) - reverse order: 12,11,10,9
    uint8_t accel_x_bytes[4] = {packet[12], packet[11], packet[10], packet[9]};
    memcpy(&telemetry_data.accel_x, accel_x_bytes, 4);
    
    // Acceleration Y (bytes 14-17) - reverse order: 16,15,14,13
    uint8_t accel_y_bytes[4] = {packet[16], packet[15], packet[14], packet[13]};
    memcpy(&telemetry_data.accel_y, accel_y_bytes, 4);
    
    // Acceleration Z (bytes 18-21) - reverse order: 20,19,18,17
    uint8_t accel_z_bytes[4] = {packet[20], packet[19], packet[18], packet[17]};
    memcpy(&telemetry_data.accel_z, accel_z_bytes, 4);
    
    // Angle X (bytes 22-25) - reverse order: 24,23,22,21
    uint8_t angle_x_bytes[4] = {packet[24], packet[23], packet[22], packet[21]};
    memcpy(&telemetry_data.angle_x, angle_x_bytes, 4);
    
    // Angle Y (bytes 26-29) - reverse order: 28,27,26,25
    uint8_t angle_y_bytes[4] = {packet[28], packet[27], packet[26], packet[25]};
    memcpy(&telemetry_data.angle_y, angle_y_bytes, 4);
    
    // Angle Z (bytes 30-33) - reverse order: 32,31,30,29
    uint8_t angle_z_bytes[4] = {packet[32], packet[31], packet[30], packet[29]};
    memcpy(&telemetry_data.angle_z, angle_z_bytes, 4);
}

/**
 * @brief Print parsed telemetry data via UART
 */
/**
 * @brief Process incoming byte and look for complete packets
 * @param new_byte: New byte received
 */
void ProcessIncomingData(uint8_t new_byte)
{
    // Store byte in buffer
    incoming_buffer[buffer_index] = new_byte;

    // Simple and fast packet detection
    static uint8_t packet_state = 0;
    static uint8_t byte_count = 0;

    switch (packet_state) {
        case 0: // Looking for header
            if (new_byte == TELEMETRY_HEADER) {
                packet_state = 1;
                byte_count = 1;
            }
            break;

        case 1: // Collecting packet data
            byte_count++;
            if (byte_count >= PACKET_SIZE) {
                // Check footer
                if (incoming_buffer[(buffer_index - PACKET_SIZE + 1 + PACKET_SIZE - 2) % (PACKET_SIZE * 4)] == FOOTER_1 &&
                    incoming_buffer[(buffer_index - PACKET_SIZE + 1 + PACKET_SIZE - 1) % (PACKET_SIZE * 4)] == FOOTER_2) {

                    // Copy complete packet to rx_buffer
                    uint16_t start_idx = (buffer_index - PACKET_SIZE + 1) % (PACKET_SIZE * 4);
                    for (uint8_t i = 0; i < PACKET_SIZE; i++) {
                        rx_buffer[i] = incoming_buffer[(start_idx + i) % (PACKET_SIZE * 4)];
                    }

                    // Set flag for main loop to process
                    packet_received = 1;
                    packets_received_count++; // Increment packet counter
                }

                // Reset for next packet
                packet_state = 0;
                byte_count = 0;
            }
            break;
    }
}

/**
 * @brief UART receive complete callback
 * @param huart: UART handle
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    // Process the received byte immediately
    ProcessIncomingData(incoming_buffer[buffer_index]);

    // Move to next buffer position
    buffer_index = (buffer_index + 1) % (PACKET_SIZE * 4);

    // Continue receiving next byte immediately - no delays
    HAL_UART_Receive_IT(&huart1, &incoming_buffer[buffer_index], 1);
  }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        // USART1 ile gönderim tamamlandı
        uart_busy = 0; // Örn: tekrar gönderime izin ver
    }
    else if (huart->Instance == USART2) {
        // USART2 için yapılacak işlemler
    }
    else if (huart->Instance == USART3) {
        // USART3 için yapılacak işlemler
    }
}

/**
 * @brief Timer2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71999; // 72MHz / 72000 = 1kHz
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4; // 1kHz / 5 = 200Hz (5ms period) - faster for better responsiveness
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief Timer callback function - called every 5ms
 */
void TimerCallback(void)
{
  timer_tick++;

  // Timer callback - minimal processing
}

/**
 * @brief Timer period elapsed callback
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) {
    TimerCallback();
  }
}

/**
 * @brief Update status bits based on telemetry data
 */
void UpdateStatusFromTelemetry(void)
{
    uint8_t new_status = current_status_byte; // Start with current status (preserve existing bits)
    uint32_t current_time = HAL_GetTick();

    // Get filtered values
    float filtered_altitude = GetFilteredValue(altitude_filter);
    float filtered_accel_z = GetFilteredValue(accel_z_filter);
    float filtered_angle_x = GetFilteredValue(angle_x_filter);
    float filtered_angle_y = GetFilteredValue(angle_y_filter);

    // Check rocket fired (accel z > 30) - FIRST BIT (Bit 0)
    // Once activated, this bit stays on permanently
    if (filtered_accel_z > 25.0f) {
        new_status |= STATUS_ROCKET_FIRED_BIT;
        if (!(current_status_byte & STATUS_ROCKET_FIRED_BIT)) {
            // Rocket just fired, record timestamp
            rocket_fired_timestamp = current_time;
        }
    }

    // Check waited 5 seconds after rocket fired - SECOND BIT (Bit 1)
    // Only if rocket fired bit is active, and once activated stays on
    if ((new_status & STATUS_ROCKET_FIRED_BIT) &&
        (current_time - rocket_fired_timestamp >= 5000)) {
        new_status |= STATUS_WAITED_5SN_BIT;
    }

    // Check minimum altitude (>= 1500) - THIRD BIT (Bit 2)
    // Only if waited 5s bit is active, and once activated stays on
    if ((new_status & STATUS_WAITED_5SN_BIT) &&
        filtered_altitude >= 1500.0f) {
        new_status |= STATUS_MIN_ALTITUDE_BIT;
    }

    // Check angle exceeded (x or y > 60) - FOURTH BIT (Bit 3)
    // Only if minimum altitude bit is active, and once activated stays on
    if ((new_status & STATUS_MIN_ALTITUDE_BIT) &&
        (filtered_angle_x > 50.0f || filtered_angle_y > 50.0f)) {
        new_status |= STATUS_ANGLE_EXCEEDED_BIT;
    }

    // Check altitude decreasing - FIFTH BIT (Bit 4)
    // Only if angle exceeded bit is active, and once activated stays on
    if ((new_status & STATUS_ANGLE_EXCEEDED_BIT) &&
        filtered_altitude < previous_altitude) {
        new_status |= STATUS_ALTITUDE_DECREASING_BIT;
        // First parachute deployed at the same time as altitude decreasing - SIXTH BIT (Bit 5)
        new_status |= STATUS_FIRST_PARACHUTE_BIT;
    }

    // Check altitude <= 550 - SEVENTH BIT (Bit 6)
    if ((new_status & STATUS_FIRST_PARACHUTE_BIT) &&
        filtered_altitude <= 600.0f) {
        new_status |= STATUS_ALTITUDE_550_BIT;
    }

    // Check second parachute deployed - EIGHTH BIT (Bit 7)
    if ((new_status & STATUS_ALTITUDE_550_BIT) &&
        filtered_altitude <= 550.0f) {  // örnek eşik
        new_status |= STATUS_SECOND_PARACHUTE_BIT;
    }

    
    // Update status byte (new bits are added, existing bits are preserved)
    current_status_byte = new_status;

    // Store current filtered altitude for next comparison
    previous_altitude = filtered_altitude;
}

/**
 * @brief Calculate checksum for status message
 * @param data: Pointer to data array
 * @param length: Length of data (excluding checksum)
 * @retval Checksum value
 */
uint8_t CalculateChecksum(uint8_t* data, uint8_t length)
{
    uint16_t sum = 0;
    for (uint8_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return (uint8_t)(sum % 256);
}

/**
 * @brief Send status message in the new 6-byte format
 */
void SendStatusMessage(void)
{
    // Build status message
    status_message[0] = STATUS_HEADER;        // 0xAA
    status_message[1] = current_status_byte;  // Status byte
    status_message[2] = 0x00;                // Reserved
    status_message[3] = 0x00;                // Checksum (calculated below)
    status_message[4] = STATUS_FOOTER_1;      // 0x0D
    status_message[5] = STATUS_FOOTER_2;      // 0x0A

    // Calculate checksum (sum of bytes 0, 1, 2, 4, 5, then mod 256)
    uint8_t checksum_data[5] = {status_message[0], status_message[1], status_message[2], status_message[4], status_message[5]};
    status_message[3] = CalculateChecksum(checksum_data, 5);
    // Send status message
    if (!uart_busy) {
        uart_busy = 1;
        HAL_UART_Transmit_IT(&huart1, status_message, STATUS_MESSAGE_SIZE);
    }

}

/**
 * @brief Apply filtering to telemetry values
 */
void ApplyFiltering(void)
{
    // Add new values to filter arrays
    altitude_filter[filter_index] = telemetry_data.altitude;
    accel_z_filter[filter_index] = telemetry_data.accel_z;
    angle_x_filter[filter_index] = telemetry_data.angle_x;
    angle_y_filter[filter_index] = telemetry_data.angle_y;
    
    // Move to next filter position
    filter_index = (filter_index + 1) % FILTER_SIZE;
    
    // Mark filter as filled after first complete cycle
    if (filter_index == 0) {
        filter_filled = 1;
    }
}

/**
 * @brief Get filtered value from filter array
 * @param filter_array: Pointer to filter array
 * @retval Filtered value (average)
 */
float GetFilteredValue(float* filter_array)
{
    if (!filter_filled) {
        // If filter not filled yet, return current value
        return filter_array[0];
    }

    // Calculate moving average
    float sum = 0;
    for (uint8_t i = 0; i < FILTER_SIZE; i++) {
        sum += filter_array[i];
    }
    return sum / FILTER_SIZE;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
