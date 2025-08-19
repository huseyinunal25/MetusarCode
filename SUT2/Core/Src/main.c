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
#include <math.h>
#include <stdint.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Flight event flags structure
typedef struct {
    uint16_t launch_detected : 1;
    uint16_t motor_burnout_delay_completed : 1;
    uint16_t min_altitude_reached : 1;
    uint16_t excessive_tilt_detected : 1;
    uint16_t descent_detected : 1;
    uint16_t drogue_deployment_issued : 1;
    uint16_t altitude_below_main_threshold : 1;
    uint16_t main_parachute_deployment_issued : 1;
    uint16_t reserved : 8;
} FlightStatus_t;

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

// Command types
typedef enum {
    CMD_NONE = 0x00,
    CMD_START_SIT = 0x20,
    CMD_START_SUT = 0x22,
    CMD_STOP_TEST = 0x24
} CommandType_t;

// System state
typedef enum {
    STATE_IDLE = 0,
    STATE_SENSOR_MONITORING,
    STATE_SYNTHETIC_FLIGHT
} SystemState_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Flight parameters
#define LAUNCH_ACCELERATION_THRESHOLD 10.0f      // m/s²
#define MOTOR_BURNOUT_DELAY_MS 5000             // 5 seconds
#define MIN_ALTITUDE_THRESHOLD 1500.0f           // meters
#define EXCESSIVE_TILT_THRESHOLD 60.0f           // degrees
#define MAIN_PARACHUTE_ALTITUDE 550.0f           // meters

// Packet constants
#define TELEMETRY_HEADER 0xAB
#define COMMAND_HEADER 0xAA
#define FOOTER_1 0x0D
#define FOOTER_2 0x0A

// GPIO pins for parachute deployment
#define DROGUE_PARACHUTE_PIN GPIO_PIN_14
#define MAIN_PARACHUTE_PIN GPIO_PIN_15
#define DROGUE_PARACHUTE_PORT GPIOB
#define MAIN_PARACHUTE_PORT GPIOB

// Buffer sizes
#define TELEMETRY_BUFFER_SIZE 36
#define COMMAND_BUFFER_SIZE 5
#define STATUS_BUFFER_SIZE 6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
// Global variables
FlightStatus_t flight_status = {0};
TelemetryData_t current_telemetry = {0};
SystemState_t system_state = STATE_IDLE;

// Timing variables
uint32_t launch_timestamp = 0;
uint32_t last_status_transmission = 0;
uint32_t status_transmission_interval = 1000; // 1 second

// Buffers
uint8_t telemetry_buffer[TELEMETRY_BUFFER_SIZE];
uint8_t command_buffer[COMMAND_BUFFER_SIZE];
uint8_t status_buffer[STATUS_BUFFER_SIZE];

// Moving average filter for sensor data
#define FILTER_SIZE 5
float altitude_filter[FILTER_SIZE] = {0};
float accel_z_filter[FILTER_SIZE] = {0};
uint8_t filter_index = 0;

// Peak altitude tracking
float peak_altitude = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void ProcessTelemetryPacket(uint8_t* packet);
void ProcessCommandPacket(uint8_t* packet);
void UpdateFlightStatus(void);
void DeployDrogueParachute(void);
void DeployMainParachute(void);
void TransmitStatus(void);
uint8_t CalculateChecksum(uint8_t* data, uint8_t length);
void ResetSystem(void);
float ApplyMovingAverageFilter(float* filter, float new_value);
void InitializeGPIO(void);
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  InitializeGPIO();

  // Enable UART1 receive interrupt for commands (5 bytes)
  HAL_UART_Receive_IT(&huart1, command_buffer, COMMAND_BUFFER_SIZE);

  // Also enable interrupt for telemetry data (36 bytes)
  // Note: We'll need to handle both types of packets

  // For testing: Automatically start SUT mode
  // Remove this in production
  system_state = STATE_SYNTHETIC_FLIGHT;
  
  // Send initial status to verify UART is working
  HAL_Delay(1000); // Wait 1 second
  TransmitStatus();
  
  // Send a test message to verify UART transmission
  uint8_t test_msg[] = "ROCKET CONTROL SYSTEM READY\r\n";
  HAL_UART_Transmit(&huart1, test_msg, sizeof(test_msg)-1, 100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Get current time once for this loop iteration
    uint32_t current_time = HAL_GetTick();
    
    // Process incoming telemetry data
    if (system_state != STATE_IDLE) {
      // Check for telemetry data (this would typically come from sensors)
      // For now, we'll simulate telemetry data reception
      
      // Update flight status based on current telemetry
      UpdateFlightStatus();
      
      // Transmit status periodically
      if (current_time - last_status_transmission >= status_transmission_interval) {
        TransmitStatus();
        last_status_transmission = current_time;
      }
    } else {
      // Even in IDLE state, transmit status to show system is alive
      if (current_time - last_status_transmission >= status_transmission_interval) {
        TransmitStatus();
        last_status_transmission = current_time;
      }
    }
    
    // Check if we have received telemetry data via UART1
    // This would typically be done in an interrupt, but for now we'll check here
    if (system_state != STATE_IDLE) {
      // Process any received telemetry data
      // The ProcessTelemetryPacket function should be called from UART interrupt
      // when telemetry data is received
      
      // For testing purposes, let's simulate some telemetry data
      // In a real implementation, this would come from sensors
      static uint32_t last_simulated_telemetry = 0;
      if (current_time - last_simulated_telemetry >= 2000) { // Every 2 seconds
        last_simulated_telemetry = current_time;
        
        // Simulate increasing altitude and acceleration
        if (current_telemetry.altitude < 2000.0f) {
          current_telemetry.altitude += 50.0f;
        }
        if (current_telemetry.accel_z < 20.0f) {
          current_telemetry.accel_z += 2.0f;
        }
        
        // Update flight status with simulated data
        UpdateFlightStatus();
      }
    }

    // Debug: Print current system state every 5 seconds
    static uint32_t last_debug_print = 0;
    if (current_time - last_debug_print >= 5000) {
        last_debug_print = current_time;
        // Send a test status packet to verify UART transmission is working
        TransmitStatus();

        // Also send a simple debug message with current status
        char debug_msg[100];
        snprintf(debug_msg, sizeof(debug_msg),
                "DEBUG: Alt=%.1fm, AccZ=%.1fm/s2, Status=0x%04X\r\n",
                current_telemetry.altitude,
                current_telemetry.accel_z,
                (uint16_t)(flight_status.launch_detected |
                          (flight_status.motor_burnout_delay_completed << 1) |
                          (flight_status.min_altitude_reached << 2) |
                          (flight_status.excessive_tilt_detected << 3) |
                          (flight_status.descent_detected << 4) |
                          (flight_status.drogue_deployment_issued << 5) |
                          (flight_status.altitude_below_main_threshold << 6) |
                          (flight_status.main_parachute_deployment_issued << 7)));
        HAL_UART_Transmit(&huart1, (uint8_t*)debug_msg, strlen(debug_msg), 100);
    }
    
    // Small delay to prevent busy waiting
    HAL_Delay(10);
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

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
 * @brief Initialize GPIO pins for parachute deployment
 */
void InitializeGPIO(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // Enable GPIOB clock if not already enabled
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    // Configure drogue parachute pin (GPIO 14)
    GPIO_InitStruct.Pin = DROGUE_PARACHUTE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DROGUE_PARACHUTE_PORT, &GPIO_InitStruct);
    
    // Configure main parachute pin (GPIO 15)
    GPIO_InitStruct.Pin = MAIN_PARACHUTE_PIN;
    HAL_GPIO_Init(MAIN_PARACHUTE_PORT, &GPIO_InitStruct);
    
    // Set both pins to low initially
    HAL_GPIO_WritePin(DROGUE_PARACHUTE_PORT, DROGUE_PARACHUTE_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MAIN_PARACHUTE_PORT, MAIN_PARACHUTE_PIN, GPIO_PIN_RESET);
}

/**
 * @brief Calculate checksum for packet validation
 * @param data: Pointer to data array
 * @param length: Length of data array
 * @retval Calculated checksum
 */
uint8_t CalculateChecksum(uint8_t* data, uint8_t length)
{
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < length; i++) {
        checksum += data[i];
    }
    return checksum;
}

/**
 * @brief Apply moving average filter to reduce sensor noise
 * @param filter: Pointer to filter array
 * @param new_value: New sensor value
 * @retval Filtered value
 */
float ApplyMovingAverageFilter(float* filter, float new_value)
{
    filter[filter_index] = new_value;
    filter_index = (filter_index + 1) % FILTER_SIZE;
    
    float sum = 0.0f;
    for (uint8_t i = 0; i < FILTER_SIZE; i++) {
        sum += filter[i];
    }
    
    return sum / FILTER_SIZE;
}

/**
 * @brief Process incoming telemetry packet
 * @param packet: Pointer to telemetry packet
 */
void ProcessTelemetryPacket(uint8_t* packet)
{
    // Validate packet header and footer
    if (packet[0] != TELEMETRY_HEADER || 
        packet[34] != FOOTER_1 || 
        packet[35] != FOOTER_2) {
        return; // Invalid packet
    }
    
    // Validate checksum
    uint8_t calculated_checksum = CalculateChecksum(packet, 34);
    if (calculated_checksum != packet[34]) {
        return; // Checksum mismatch
    }
    
    // Extract telemetry data (little-endian IEEE 754 float)
    memcpy(&current_telemetry.altitude, &packet[1], 4);
    memcpy(&current_telemetry.pressure, &packet[5], 4);
    memcpy(&current_telemetry.accel_x, &packet[9], 4);
    memcpy(&current_telemetry.accel_y, &packet[13], 4);
    memcpy(&current_telemetry.accel_z, &packet[17], 4);
    memcpy(&current_telemetry.angle_x, &packet[21], 4);
    memcpy(&current_telemetry.angle_y, &packet[25], 4);
    memcpy(&current_telemetry.angle_z, &packet[29], 4);
    
    // Apply moving average filter to reduce noise
    current_telemetry.altitude = ApplyMovingAverageFilter(altitude_filter, current_telemetry.altitude);
    current_telemetry.accel_z = ApplyMovingAverageFilter(accel_z_filter, current_telemetry.accel_z);
}

/**
 * @brief Process incoming command packet
 * @param packet: Pointer to command packet
 */
void ProcessCommandPacket(uint8_t* packet)
{
    // Validate packet header and footer
    if (packet[0] != COMMAND_HEADER || 
        packet[3] != FOOTER_1 || 
        packet[4] != FOOTER_2) {
        return; // Invalid packet
    }
    
    // Validate checksum
    uint8_t calculated_checksum = CalculateChecksum(packet, 3);
    if (calculated_checksum != packet[3]) {
        return; // Checksum mismatch
    }
    
    CommandType_t command = (CommandType_t)packet[1];
    
    switch (command) {
        case CMD_START_SIT:
            system_state = STATE_SENSOR_MONITORING;
            ResetSystem();
            break;
            
        case CMD_START_SUT:
            system_state = STATE_SYNTHETIC_FLIGHT;
            ResetSystem();
            break;
            
        case CMD_STOP_TEST:
            system_state = STATE_IDLE;
            ResetSystem();
            break;
            
        default:
            break;
    }
}

/**
 * @brief Update flight status based on current telemetry data
 */
void UpdateFlightStatus(void)
{
    uint32_t current_time = HAL_GetTick();
    
    // Launch detection
    if (!flight_status.launch_detected && 
        fabsf(current_telemetry.accel_z) > LAUNCH_ACCELERATION_THRESHOLD) {
        flight_status.launch_detected = 1;
        launch_timestamp = current_time;
    }
    
    // Motor burnout delay (5 seconds after launch)
    if (flight_status.launch_detected && !flight_status.motor_burnout_delay_completed &&
        (current_time - launch_timestamp) >= MOTOR_BURNOUT_DELAY_MS) {
        flight_status.motor_burnout_delay_completed = 1;
    }
    
    // Minimum altitude threshold
    if (!flight_status.min_altitude_reached && 
        current_telemetry.altitude > MIN_ALTITUDE_THRESHOLD) {
        flight_status.min_altitude_reached = 1;
    }
    
    // Excessive tilt detection
    if (!flight_status.excessive_tilt_detected &&
        (fabsf(current_telemetry.angle_x) > EXCESSIVE_TILT_THRESHOLD ||
         fabsf(current_telemetry.angle_y) > EXCESSIVE_TILT_THRESHOLD)) {
        flight_status.excessive_tilt_detected = 1;
    }
    
    // Track peak altitude for apogee detection
    if (current_telemetry.altitude > peak_altitude) {
        peak_altitude = current_telemetry.altitude;
    }
    
    // Descent detection (apogee passed)
    if (!flight_status.descent_detected && 
        flight_status.min_altitude_reached &&
        current_telemetry.altitude < (peak_altitude - 10.0f)) { // 10m tolerance
        flight_status.descent_detected = 1;
    }
    
    // Drogue parachute deployment
    if (!flight_status.drogue_deployment_issued &&
        flight_status.launch_detected &&
        flight_status.motor_burnout_delay_completed &&
        flight_status.min_altitude_reached &&
        flight_status.excessive_tilt_detected &&
        flight_status.descent_detected) {
        DeployDrogueParachute();
    }
    
    // Main parachute altitude threshold
    if (!flight_status.altitude_below_main_threshold &&
        current_telemetry.altitude < MAIN_PARACHUTE_ALTITUDE) {
        flight_status.altitude_below_main_threshold = 1;
    }
    
    // Main parachute deployment
    if (!flight_status.main_parachute_deployment_issued &&
        flight_status.altitude_below_main_threshold) {
        DeployMainParachute();
    }
}

/**
 * @brief Deploy drogue parachute
 */
void DeployDrogueParachute(void)
{
    HAL_GPIO_WritePin(DROGUE_PARACHUTE_PORT, DROGUE_PARACHUTE_PIN, GPIO_PIN_SET);
    flight_status.drogue_deployment_issued = 1;
    
    // Keep pin high for 100ms to ensure deployment
    HAL_Delay(100);
    HAL_GPIO_WritePin(DROGUE_PARACHUTE_PORT, DROGUE_PARACHUTE_PIN, GPIO_PIN_RESET);
}

/**
 * @brief Deploy main parachute
 */
void DeployMainParachute(void)
{
    HAL_GPIO_WritePin(MAIN_PARACHUTE_PORT, MAIN_PARACHUTE_PIN, GPIO_PIN_SET);
    flight_status.main_parachute_deployment_issued = 1;
    
    // Keep pin high for 100ms to ensure deployment
    HAL_Delay(100);
    HAL_GPIO_WritePin(MAIN_PARACHUTE_PORT, MAIN_PARACHUTE_PIN, GPIO_PIN_RESET);
}

/**
 * @brief Transmit current flight status
 */
void TransmitStatus(void)
{
    // Prepare status packet
    status_buffer[0] = COMMAND_HEADER;
    
    // Convert flight status to 16-bit word
    uint16_t status_word = 0;
    status_word |= flight_status.launch_detected << 7;
    status_word |= (flight_status.motor_burnout_delay_completed << 6);
    status_word |= (flight_status.min_altitude_reached << 5);
    status_word |= (flight_status.excessive_tilt_detected << 4);
    status_word |= (flight_status.descent_detected << 3);
    status_word |= (flight_status.drogue_deployment_issued << 2);
    status_word |= (flight_status.altitude_below_main_threshold << 1);
    status_word |= (flight_status.main_parachute_deployment_issued);

    status_buffer[1] = (uint8_t)(status_word & 0xFF);        // Lower byte
    status_buffer[2] = (uint8_t)((status_word >> 8) & 0xFF); // Upper byte
    
    // Calculate and add checksum
    status_buffer[3] = CalculateChecksum(status_buffer, 3);
    
    // Add footer
    status_buffer[4] = FOOTER_1;
    status_buffer[5] = FOOTER_2;
    
    // Transmit status packet via UART1
    HAL_UART_Transmit(&huart1, status_buffer, STATUS_BUFFER_SIZE, 100);
}

/**
 * @brief Reset system to initial state
 */
void ResetSystem(void)
{
    // Reset flight status
    memset(&flight_status, 0, sizeof(FlightStatus_t));
    
    // Reset telemetry data
    memset(&current_telemetry, 0, sizeof(TelemetryData_t));
    
    // Reset filters
    memset(altitude_filter, 0, sizeof(altitude_filter));
    memset(accel_z_filter, 0, sizeof(accel_z_filter));
    filter_index = 0;
    
    // Reset peak altitude
    peak_altitude = 0.0f;
    
    // Reset timing variables
    launch_timestamp = 0;
    last_status_transmission = 0;
    
    // Reset GPIO pins
    HAL_GPIO_WritePin(DROGUE_PARACHUTE_PORT, DROGUE_PARACHUTE_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MAIN_PARACHUTE_PORT, MAIN_PARACHUTE_PIN, GPIO_PIN_RESET);
}

/**
 * @brief UART receive complete callback
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        // Check if this is a command or telemetry packet
        if (command_buffer[0] == COMMAND_HEADER) {
            // This is a command packet (5 bytes)
            ProcessCommandPacket(command_buffer);
            // Re-enable receive interrupt for next packet
            HAL_UART_Receive_IT(&huart1, command_buffer, COMMAND_BUFFER_SIZE);
        } else if (command_buffer[0] == TELEMETRY_HEADER) {
            // This is telemetry data - we need to receive the full 36-byte packet
            // For now, just acknowledge we received telemetry header
            // In a real implementation, you'd receive the full 36-byte packet
            // Re-enable receive interrupt for next packet
            HAL_UART_Receive_IT(&huart1, command_buffer, COMMAND_BUFFER_SIZE);
        } else {
            // Unknown packet type, re-enable receive interrupt
            HAL_UART_Receive_IT(&huart1, command_buffer, COMMAND_BUFFER_SIZE);
        }
    }
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
