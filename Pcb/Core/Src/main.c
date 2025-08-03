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

/* External Variables --------------------------------------------------------*/
extern int32_t tRaw, pRaw, hRaw;

/* Private defines -----------------------------------------------------------*/
#define PACKET_HEADER_SIZE      3
#define BUFFER_SIZE            100
#define IMU_DATA_SIZE          14
#define MEASUREMENT_DELAY      200

// Packet configuration
#define TARGET_ADDR_HIGH       0x00
#define TARGET_ADDR_LOW        0x02
#define CHANNEL                0x17

// MPU9250 Register addresses
#define MPU9250_ACCEL_XOUT_H   59
#define MPU9250_GYRO_XOUT_H    67

/* Private typedef -----------------------------------------------------------*/
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} sensor_data_t;

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

// Sensor data variables
float Temperature, Pressure, altitude;
static float previous_altitude = 0;

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
void transmit_sensor_packet(int altitude, sensor_data_t *accel_data, sensor_data_t *gyro_data);
void initialize_sensors(void);

/* Private user code ---------------------------------------------------------*/

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

    /* Infinite loop */
    while (1)
    {
        // Read BME280 environmental data
        read_bme280_data();

        // Read IMU data
        read_accelerometer_data(&accel_data);
        read_gyroscope_data(&gyro_data);

        // Transmit accelerometer data via UART
        transmit_sensor_packet(altitude, &accel_data, &gyro_data);

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
void transmit_sensor_packet(int altitude, sensor_data_t *accel_data, sensor_data_t *gyro_data)
{
    char buffer[BUFFER_SIZE];
    uint16_t len;

    // Format sensor data string
    len = sprintf(buffer, "%d,%d,%d,%d,%.2f,%.2f,%.2f\r\n",
    			  altitude,
                  accel_data->x, accel_data->y, accel_data->z,
				  gyro_data->x/131.0, gyro_data->y/131.0, gyro_data->z/131.0);

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
