/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - Sensor data acquisition and transmission
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
#include "math.h"
#include "BME280_STM32.h"
#include <stdio.h>
#include <string.h>
#include "lwgps/lwgps.h"
/* External Variables --------------------------------------------------------*/
extern int32_t tRaw, pRaw, hRaw;

/* Private defines -----------------------------------------------------------*/
#define PACKET_HEADER_SIZE      3
#define BUFFER_SIZE            150
#define IMU_DATA_SIZE          14
#define MEASUREMENT_DELAY      200

// GPS Debug Configuration - Set to 1 to enable raw NMEA debugging
#define GPS_DEBUG_RAW_NMEA     0

// Packet configuration
#define TARGET_ADDR_HIGH       0x00
#define TARGET_ADDR_LOW        0x02
#define CHANNEL                0x17

// MPU9250 Register addresses
#define MPU9250_ACCEL_XOUT_H   59
#define MPU9250_GYRO_XOUT_H    67

// Add these defines for accelerometer scaling
#define ACCEL_SCALE_FACTOR  16384.0f  // For ±2g range
#define GYRO_SCALE_FACTOR   131.0f    // For ±250°/s range
#define RAD_TO_DEG          57.2958f
#define DEG_TO_RAD          0.0174533f

/* Private typedef -----------------------------------------------------------*/
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} sensor_data_t;

// Structure for orientation angles
typedef struct {
    float roll;     // Rotation around X-axis
    float pitch;    // Rotation around Y-axis
    float yaw;      // Rotation around Z-axis (requires magnetometer for absolute)
} orientation_t;

// Global variables for orientation calculation
static orientation_t orientation = {0};
static uint32_t last_time = 0;

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

// Sensor data variables
float Temperature, Pressure, altitude;
static float previous_altitude = 0;

float gyro_x;
float gyro_y;
float gyro_z;

float accel_x;
float accel_y;
float accel_z;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);

// Custom function prototypes

void mpu9250_read_data(uint8_t reg, uint8_t *data, uint8_t data_length);
void read_accelerometer_data(sensor_data_t *accel_data);
void read_gyroscope_data(sensor_data_t *gyro_data);
void read_bme280_data(void);
void transmit_sensor_packet(int altitude, float accel_x, float accel_y, float accel_z, float gyro_x, float gyro_y, float gyro_z, float latitude, float longitude);
void initialize_sensors(void);
void print_gps_debug_info(void);
void check_uart_errors(void);

void calculate_orientation(sensor_data_t *accel_data, sensor_data_t *gyro_data, orientation_t *orientation);
void calculate_simple_orientation(sensor_data_t *accel_data, float *roll, float *pitch);

orientation_t get_orientation(void);
uint8_t is_system_level(float tolerance);

lwgps_t gps;

uint8_t rx_buffer[128];
uint8_t rx_index = 0;
uint8_t rx_data = 0;
uint32_t gps_data_received_count = 0;

/* Private user code ---------------------------------------------------------*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart2){
		HAL_UART_Receive_IT(&huart2, &rx_data, 1);

		// Check for end of NMEA sentence (either \n or \r\n)
		if(rx_data == '\n' || rx_data == '\r'){
			if(rx_index > 0) {
				// Null terminate the buffer
				rx_buffer[rx_index] = '\0';

				// Process the complete NMEA sentence
				lwgps_statement_t result = lwgps_process(&gps, rx_buffer, rx_index);
				
				// Debug print received NMEA sentence if enabled
				#if GPS_DEBUG_RAW_NMEA
				HAL_UART_Transmit(&huart1, (uint8_t*)"NMEA: ", 6, 100);
				HAL_UART_Transmit(&huart1, rx_buffer, rx_index, 100);
				HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, 100);
				#endif

				// Reset buffer for next sentence
				rx_index = 0;
			}
		}
		else if(rx_index < sizeof(rx_buffer) - 1){
			// Add character to buffer (leave space for null terminator)
			rx_buffer[rx_index++] = rx_data;
			gps_data_received_count++; // Count received characters
		}
		else {
			// Buffer overflow - reset
			rx_index = 0;
		}
	}
}

/**
 * @brief Print GPS debug information
 * @param None
 * @retval None
 */
void print_gps_debug_info(void)
{
    char debug_buffer[300];
    uint16_t len;
    
    len = sprintf(debug_buffer, "GPS Debug: Fix=%d, Valid=%d, Sats=%d, FixMode=%d, Lat=%.6f, Lon=%.6f, RxCount=%lu\r\n",
                  gps.fix, gps.is_valid, gps.sats_in_use, gps.fix_mode, gps.latitude, gps.longitude, gps_data_received_count);
    
    HAL_UART_Transmit(&huart1, (uint8_t*)debug_buffer, len, HAL_MAX_DELAY);
}

/**
 * @brief Check UART errors and reset if necessary
 * @param None
 * @retval None
 */
void check_uart_errors(void)
{
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE)) {
        // Overrun error - clear flag and restart reception
        __HAL_UART_CLEAR_OREFLAG(&huart2);
        HAL_UART_Receive_IT(&huart2, &rx_data, 1);
    }
    
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_FE)) {
        // Framing error - clear flag
        __HAL_UART_CLEAR_FEFLAG(&huart2);
    }
    
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_NE)) {
        // Noise error - clear flag
        __HAL_UART_CLEAR_NEFLAG(&huart2);
    }
}


/**
  * @brief  The application entry point.
  * @retval int
  */

int main(void)
{
    /* MCU Configuration */
    HAL_Init();
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_SPI1_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();

    /* Initialize sensors */
    initialize_sensors();

    /* Variables for sensor data */
    sensor_data_t accel_data;
    sensor_data_t gyro_data;
    uint32_t debug_counter = 0;

    lwgps_init(&gps);
    HAL_UART_Receive_IT(&huart2, &rx_data, 1);
    HAL_Delay(200);
    
    // Print initial GPS debug info
    print_gps_debug_info();
    
    /* Infinite loop */
    while (1)
    {
        // Read BME280 environmental data
        read_bme280_data();

        // Read IMU data
        read_accelerometer_data(&accel_data);
        read_gyroscope_data(&gyro_data);

        // Calculate orientation
        calculate_orientation(&accel_data, &gyro_data, &orientation);

        transmit_sensor_packet(altitude, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z,
                              gps.latitude, gps.longitude);

        // Print GPS debug info every 50 cycles (about every 10 seconds)
        if (++debug_counter >= 50) {
            print_gps_debug_info();
            check_uart_errors(); // Check for UART errors periodically
            debug_counter = 0;
        }

        // Wait before next measurement
        HAL_Delay(MEASUREMENT_DELAY);
    }
}

/**
  * @brief Initialize sensors
  * @param None
  * @retval None
  */
void initialize_sensors(void)
{
    // Configure BME280 sensor
    BME280_Config(OSRS_2, OSRS_2, OSRS_1, MODE_NORMAL, T_SB_0p5, IIR_16);
}

/**
  * @brief Read BME280 environmental data
  * @param None
  * @retval None
  */
void read_bme280_data(void)
{
    BME280_Measure();
    altitude = 44330 * (1 - pow((Pressure/101325), (1/5.225)));
}

/**
  * @brief Read accelerometer data from MPU9250
  * @param accel_data: Pointer to accelerometer data structure
  * @retval None
  */
void read_accelerometer_data(sensor_data_t *accel_data)
{
    uint8_t imu_data[IMU_DATA_SIZE];

    mpu9250_read_data(MPU9250_ACCEL_XOUT_H, imu_data, sizeof(imu_data));

    accel_data->x = ((int16_t)imu_data[0] << 8) + imu_data[1];
    accel_data->y = ((int16_t)imu_data[2] << 8) + imu_data[3];
    accel_data->z = ((int16_t)imu_data[4] << 8) + imu_data[5];
}

/**
  * @brief Read gyroscope data from MPU9250
  * @param gyro_data: Pointer to gyroscope data structure
  * @retval None
  */
void read_gyroscope_data(sensor_data_t *gyro_data)
{
    uint8_t imu_data[IMU_DATA_SIZE];

    mpu9250_read_data(MPU9250_GYRO_XOUT_H, imu_data, sizeof(imu_data));

    gyro_data->x = ((int16_t)imu_data[0] << 8) + imu_data[1];
    gyro_data->y = ((int16_t)imu_data[2] << 8) + imu_data[3];
    gyro_data->z = ((int16_t)imu_data[4] << 8) + imu_data[5];
}

/**
  * @brief Transmit sensor data packet via UART
  * @param accel_data: Pointer to accelerometer data structure
  * @retval None
  */
void transmit_sensor_packet(int altitude, float accel_x, float accel_y, float accel_z, float gyro_x, float gyro_y, float gyro_z, float latitude, float longitude)
{
    char buffer[BUFFER_SIZE];
    uint16_t len;

    // Check GPS fix status before using coordinates
    if (gps.fix == 0 || !gps.is_valid) {
        // GPS has no fix or is invalid - use zero values
        len = sprintf(buffer, "%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,0.000000,0.000000\r\n",
            altitude,
            accel_x, accel_y, accel_z,
            orientation.roll, orientation.pitch, orientation.yaw
        );
    } else {
        // GPS has valid fix - use actual coordinates
        len = sprintf(buffer, "%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%f,%f\r\n",
            altitude,
            accel_x, accel_y, accel_z,
            orientation.roll, orientation.pitch, orientation.yaw,
            gps.latitude, gps.longitude
        );
    }

    //orientation.roll, orientation.pitch, orientation.yaw
    // Create packet with header
    uint8_t packet[PACKET_HEADER_SIZE + len];
    packet[0] = TARGET_ADDR_HIGH;
    packet[1] = TARGET_ADDR_LOW;
    packet[2] = CHANNEL;
    memcpy(&packet[PACKET_HEADER_SIZE], buffer, len);

    // Transmit packet
    HAL_UART_Transmit(&huart3, packet, PACKET_HEADER_SIZE + len, HAL_MAX_DELAY);
}

/**
  * @brief Read data from MPU9250 via SPI
  * @param reg: Register address to read from
  * @param data: Pointer to data buffer
  * @param data_length: Number of bytes to read
  * @retval None
  */
void mpu9250_read_data(uint8_t reg, uint8_t *data, uint8_t data_length)
{
    uint8_t tx_buffer[1];
    tx_buffer[0] = 0x80 | reg;  // Read command with register address

    // Assert CS (Chip Select)
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

    // Send register address and read data
    HAL_SPI_Transmit(&hspi1, tx_buffer, 1, 100);
    HAL_SPI_Receive(&hspi1, data, data_length, 100);

    // Deassert CS
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
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
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
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
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 9600;
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
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{
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
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

    /*Configure GPIO pin : PA4 */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

//////////////////////

void calculate_orientation(sensor_data_t *accel_data, sensor_data_t *gyro_data, orientation_t *orientation)
{
    // Convert raw accelerometer data to g-force
    accel_x = (float)accel_data->x / ACCEL_SCALE_FACTOR;
    accel_y = (float)accel_data->y / ACCEL_SCALE_FACTOR;
    accel_z = (float)accel_data->z / ACCEL_SCALE_FACTOR;

    // Convert raw gyroscope data to degrees per second
    gyro_x = (float)gyro_data->x / GYRO_SCALE_FACTOR + 1.3;
    gyro_y = (float)gyro_data->y / GYRO_SCALE_FACTOR + 3.3;
    gyro_z = (float)gyro_data->z / GYRO_SCALE_FACTOR - 0.3;

    if (-0.5 <= gyro_x && gyro_x <= 0.5){
    	gyro_x = 0;
    }
    if (-0.5 <= gyro_y && gyro_y <= 0.5){
    	gyro_y = 0;
    }
    if (-0.5 <= gyro_z && gyro_z <= 0.5){
    	gyro_z = 0;
    }

    // Calculate time difference
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - last_time) / 1000.0f; // Convert to seconds
    last_time = current_time;

    // Skip first calculation (dt would be invalid)
    if (dt > 1.0f || dt <= 0) {
        dt = 0.001f; // Use small default value
    }

    /*// Method 1: Accelerometer-based angles (for static/slow movement)
    float accel_roll = atan2(accel_y, sqrt(accel_x*accel_x + accel_z*accel_z)) * RAD_TO_DEG;
    float accel_pitch = atan2(-accel_x, sqrt(accel_y*accel_y + accel_z*accel_z)) * RAD_TO_DEG;

    // Method 2: Gyroscope integration
    float gyro_roll = orientation->roll + gyro_x * dt;
    float gyro_pitch = orientation->pitch + gyro_y * dt;
    float gyro_yaw = orientation->yaw + gyro_z * dt;

    // Complementary filter (combines both methods)
    float alpha = 0.98f; // Filter coefficient (0.98 = 98% gyro, 2% accel)

    orientation->roll = alpha * gyro_roll + (1.0f - alpha) * accel_roll;
    orientation->pitch = alpha * gyro_pitch + (1.0f - alpha) * accel_pitch;
    orientation->yaw = gyro_yaw; // Only gyro for yaw (needs magnetometer for absolute)*/

    float accel_magnitude = sqrt(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z);
	uint8_t is_stationary = (fabs(accel_magnitude - 1.0f) < 0.15f); // Within 0.15g of 1g

	// FIX 3: Only integrate gyro when there's significant movement OR use stronger accel filter when stationary
	if (is_stationary) {
		// When stationary, trust accelerometer more and reduce gyro integration
		float accel_roll = atan2(accel_y, sqrt(accel_x*accel_x + accel_z*accel_z)) * RAD_TO_DEG;
		float accel_pitch = atan2(-accel_x, sqrt(accel_y*accel_y + accel_z*accel_z)) * RAD_TO_DEG;

		// Use strong accelerometer bias when stationary
		float alpha = 0.90f; // 90% gyro, 10% accel (less gyro influence)

		float gyro_roll = orientation->roll + gyro_x * dt;
		float gyro_pitch = orientation->pitch + gyro_y * dt;

		orientation->roll = alpha * gyro_roll + (1.0f - alpha) * accel_roll;
		orientation->pitch = alpha * gyro_pitch + (1.0f - alpha) * accel_pitch;

		// FIX 4: Reduce yaw drift when stationary
		if (fabs(gyro_z) < 0.3f) {
			orientation->yaw += gyro_z * dt * 0.5f; // Reduce yaw integration when barely moving
		} else {
			orientation->yaw += gyro_z * dt;
		}
	} else {
		// When moving, trust gyroscope more
		float accel_roll = atan2(accel_y, sqrt(accel_x*accel_x + accel_z*accel_z)) * RAD_TO_DEG;
		float accel_pitch = atan2(-accel_x, sqrt(accel_y*accel_y + accel_z*accel_z)) * RAD_TO_DEG;

		float alpha = 0.98f; // 98% gyro, 2% accel

		float gyro_roll = orientation->roll + gyro_x * dt;
		float gyro_pitch = orientation->pitch + gyro_y * dt;

		orientation->roll = alpha * gyro_roll + (1.0f - alpha) * accel_roll;
		orientation->pitch = alpha * gyro_pitch + (1.0f - alpha) * accel_pitch;
		orientation->yaw += gyro_z * dt;
	}

}

/**
 * @brief Simple accelerometer-only orientation (for static conditions)
 * @param accel_data: Raw accelerometer data
 * @param roll: Pointer to roll angle
 * @param pitch: Pointer to pitch angle
 * @retval None
 */
void calculate_simple_orientation(sensor_data_t *accel_data, float *roll, float *pitch)
{
    // Convert to g-force
    accel_x = (float)accel_data->x / ACCEL_SCALE_FACTOR;
    accel_y = (float)accel_data->y / ACCEL_SCALE_FACTOR;
    accel_z = (float)accel_data->z / ACCEL_SCALE_FACTOR;

    // Calculate angles
    *roll = atan2(accel_y, sqrt(accel_x*accel_x + accel_z*accel_z)) * RAD_TO_DEG;
    *pitch = atan2(-accel_x, sqrt(accel_y*accel_y + accel_z*accel_z)) * RAD_TO_DEG;
}

/**
 * @brief Get current orientation angles
 * @retval orientation_t Current orientation
 */



orientation_t get_orientation(void)
{
    return orientation;
}

/**
 * @brief Check if system is level (within tolerance)
 * @param tolerance: Tolerance in degrees
 * @retval 1 if level, 0 if not
 */
uint8_t is_system_level(float tolerance)
{
    return (fabs(orientation.roll) < tolerance && fabs(orientation.pitch) < tolerance);
}

////////////////

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
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
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */
