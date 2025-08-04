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

/* Private defines -----------------------------------------------------------*/
#define PACKET_HEADER_SIZE      3
#define BUFFER_SIZE             150
#define IMU_DATA_SIZE           14
#define MEASUREMENT_DELAY       200

#define GPS_DEBUG_RAW_NMEA      1

#define TARGET_ADDR_HIGH        0x00
#define TARGET_ADDR_LOW         0x02
#define CHANNEL                 0x17

#define MPU9250_ACCEL_XOUT_H    0x3B  // 59
#define MPU9250_GYRO_XOUT_H     0x43  // 67

#define ACCEL_SCALE_FACTOR      16384.0f  // ±2g
#define GYRO_SCALE_FACTOR       131.0f    // ±250 °/s
#define RAD_TO_DEG              57.2958f

/* Private typedef -----------------------------------------------------------*/
typedef struct {
    int16_t x, y, z;
} sensor_data_t;

typedef struct {
    float roll, pitch, yaw;
} orientation_t;

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef    hi2c1;
SPI_HandleTypeDef    hspi1;
UART_HandleTypeDef   huart1;    /* Debug */
UART_HandleTypeDef   huart2;    /* GPS */
UART_HandleTypeDef   huart3;    /* LoRa */

static orientation_t orientation = {0};
static uint32_t      last_time = 0;

lwgps_t              gps;
uint8_t              rx_buffer[128];
uint8_t              rx_index = 0;
uint8_t              rx_data = 0;
uint32_t             gps_data_received_count = 0;

float Temperature, Pressure, altitude;
float gyro_x, gyro_y, gyro_z;
float accel_x, accel_y, accel_z;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);

void initialize_sensors(void);
void read_bme280_data(void);
void read_accelerometer_data(sensor_data_t *accel_data);
void read_gyroscope_data(sensor_data_t *gyro_data);
void calculate_orientation(sensor_data_t *accel_data, sensor_data_t *gyro_data, orientation_t *orientation);
void transmit_sensor_packet(int altitude, float ax, float ay, float az, float gx, float gy, float gz, float lat, float lon);
void print_gps_debug_info(void);
void check_uart_errors(void);
void mpu9250_read_data(uint8_t reg, uint8_t *data, uint8_t len);

/* UART MSP Initialization (GPIO + NVIC) -------------------------------------*/


/* UART IRQ Handlers ---------------------------------------------------------*/
void USART2_IRQHandler(void) { HAL_UART_IRQHandler(&huart2); }
void USART3_IRQHandler(void) { HAL_UART_IRQHandler(&huart3); }

/* UART Rx Complete Callback for GPS (USART2) --------------------------------*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) {
        /* Re-arm reception */
        HAL_UART_Receive_IT(&huart2, &rx_data, 1);

        if (rx_data == '\n' || rx_data == '\r') {
            if (rx_index > 0) {
                rx_buffer[rx_index] = '\0';
                lwgps_process(&gps, rx_buffer, rx_index);
#if GPS_DEBUG_RAW_NMEA
                HAL_UART_Transmit(&huart1, (uint8_t *)"NMEA: ", 6, 100);
                HAL_UART_Transmit(&huart1, rx_buffer, rx_index, 100);
                HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, 100);
#endif
                rx_index = 0;
            }
        }
        else if (rx_index < sizeof(rx_buffer) - 1) {
            rx_buffer[rx_index++] = rx_data;
            gps_data_received_count++;
        }
        else {
            /* Buffer overflow, reset */
            rx_index = 0;
        }
    }
}

/* Print GPS debug info ------------------------------------------------------*/
void print_gps_debug_info(void)
{
    char dbg[256];
    int len = sprintf(dbg,
        "GPS Debug: fix=%d valid=%d sats=%d lat=%.6f lon=%.6f rx=%lu\r\n",
        gps.fix, gps.is_valid, gps.sats_in_use,
        gps.latitude, gps.longitude,
        gps_data_received_count
    );
    HAL_UART_Transmit(&huart1, (uint8_t*)dbg, len, HAL_MAX_DELAY);
}

/* Recover from UART errors on GPS ------------------------------------------*/
void check_uart_errors(void)
{
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE)) {
        __HAL_UART_CLEAR_OREFLAG(&huart2);
        HAL_UART_Receive_IT(&huart2, &rx_data, 1);
    }
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_FE))
        __HAL_UART_CLEAR_FEFLAG(&huart2);
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_NE))
        __HAL_UART_CLEAR_NEFLAG(&huart2);
}

/* Main -----------------------------------------------------------------------*/
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_SPI1_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();

    initialize_sensors();

    lwgps_init(&gps);
    HAL_UART_Receive_IT(&huart2, &rx_data, 1);
    HAL_Delay(200);
    print_gps_debug_info();

    sensor_data_t accel_data, gyro_data;
    uint32_t debug_counter = 0;

    while (1) {
        /* BME280 */
        read_bme280_data();

        /* IMU */
        read_accelerometer_data(&accel_data);
        read_gyroscope_data(&gyro_data);
        calculate_orientation(&accel_data, &gyro_data, &orientation);

        /* Transmit via LoRa */
        transmit_sensor_packet(
            (int)altitude,
            accel_data.x / ACCEL_SCALE_FACTOR,
            accel_data.y / ACCEL_SCALE_FACTOR,
            accel_data.z / ACCEL_SCALE_FACTOR,
            gyro_data.x / GYRO_SCALE_FACTOR,
            gyro_data.y / GYRO_SCALE_FACTOR,
            gyro_data.z / GYRO_SCALE_FACTOR,
            gps.latitude,
            gps.longitude
        );

        if (++debug_counter >= 50) {
            print_gps_debug_info();
            check_uart_errors();
            debug_counter = 0;
        }
        HAL_Delay(MEASUREMENT_DELAY);
    }
}

/* Initialize BME280 ---------------------------------------------------------*/
void initialize_sensors(void)
{
    BME280_Config(OSRS_2, OSRS_2, OSRS_1, MODE_NORMAL, T_SB_0p5, IIR_16);
}

/* Read BME280 data ----------------------------------------------------------*/
void read_bme280_data(void)
{
    BME280_Measure();
    altitude = 44330.0f * (1.0f - powf((Pressure / 101325.0f), (1.0f / 5.225f)));
}

/* Read accelerometer --------------------------------------------------------*/
void read_accelerometer_data(sensor_data_t *accel_data)
{
    uint8_t buf[IMU_DATA_SIZE];
    mpu9250_read_data(MPU9250_ACCEL_XOUT_H, buf, 6);
    accel_data->x = (int16_t)((buf[0] << 8) | buf[1]);
    accel_data->y = (int16_t)((buf[2] << 8) | buf[3]);
    accel_data->z = (int16_t)((buf[4] << 8) | buf[5]);
}

/* Read gyroscope ------------------------------------------------------------*/
void read_gyroscope_data(sensor_data_t *gyro_data)
{
    uint8_t buf[IMU_DATA_SIZE];
    mpu9250_read_data(MPU9250_GYRO_XOUT_H, buf, 6);
    gyro_data->x = (int16_t)((buf[0] << 8) | buf[1]);
    gyro_data->y = (int16_t)((buf[2] << 8) | buf[3]);
    gyro_data->z = (int16_t)((buf[4] << 8) | buf[5]);
}

/* MPU9250 SPI read ----------------------------------------------------------*/
void mpu9250_read_data(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t tx = 0x80 | reg;
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &tx, 1, 100);
    HAL_SPI_Receive(&hspi1, data, len, 100);
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
}

/* Calculate orientation (complementary filter) ------------------------------*/
void calculate_orientation(sensor_data_t *accel_data, sensor_data_t *gyro_data, orientation_t *o)
{
    /* Convert to units */
    accel_x = accel_data->x / ACCEL_SCALE_FACTOR;
    accel_y = accel_data->y / ACCEL_SCALE_FACTOR;
    accel_z = accel_data->z / ACCEL_SCALE_FACTOR;
    gyro_x  = gyro_data->x / GYRO_SCALE_FACTOR;
    gyro_y  = gyro_data->y / GYRO_SCALE_FACTOR;
    gyro_z  = gyro_data->z / GYRO_SCALE_FACTOR;

    uint32_t now = HAL_GetTick();
    float dt = (now - last_time) / 1000.0f;
    last_time = now;
    if (dt <= 0 || dt > 1.0f) dt = 0.01f;

    float accel_roll  = atan2f(accel_y, sqrtf(accel_x*accel_x + accel_z*accel_z)) * RAD_TO_DEG;
    float accel_pitch = atan2f(-accel_x, sqrtf(accel_y*accel_y + accel_z*accel_z)) * RAD_TO_DEG;

    float alpha = 0.98f;
    o->roll  = alpha * (o->roll  + gyro_x * dt) + (1.0f - alpha) * accel_roll;
    o->pitch = alpha * (o->pitch + gyro_y * dt) + (1.0f - alpha) * accel_pitch;
    o->yaw  += gyro_z * dt;
}

/* Transmit packet over LoRa (USART3) ----------------------------------------*/
void transmit_sensor_packet(int alt, float ax, float ay, float az, float gx, float gy, float gz, float lat, float lon)
{
    char buf[BUFFER_SIZE];
    int len;
    if (gps.fix == 0 || !gps.is_valid) {
        len = sprintf(buf, "%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,0.000000,0.000000\r\n",
                      alt, ax, ay, az, orientation.roll, orientation.pitch, orientation.yaw);
    } else {
        len = sprintf(buf, "%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f\r\n",
                      alt, ax, ay, az, orientation.roll, orientation.pitch, orientation.yaw,
                      lat, lon);
    }
    uint8_t packet[PACKET_HEADER_SIZE + len];
    packet[0] = TARGET_ADDR_HIGH;
    packet[1] = TARGET_ADDR_LOW;
    packet[2] = CHANNEL;
    memcpy(&packet[3], buf, len);
    HAL_UART_Transmit(&huart3, packet, 3 + len, HAL_MAX_DELAY);
}

/* System Clock Configuration ------------------------------------------------*/
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef       RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef       RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK   |
                                       RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_PCLK1  |
                                       RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
}

/* I2C1 Initialization -------------------------------------------------------*/
static void MX_I2C1_Init(void)
{
    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 100000;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
}

/* SPI1 Initialization -------------------------------------------------------*/
static void MX_SPI1_Init(void)
{
    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
}

/* USART1 Initialization (Debug) ---------------------------------------------*/
static void MX_USART1_UART_Init(void)
{
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
}

/* USART2 Initialization (GPS) ------------------------------------------------*/
static void MX_USART2_UART_Init(void)
{
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 9600;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
}

/* USART3 Initialization (LoRa) ----------------------------------------------*/
static void MX_USART3_UART_Init(void)
{
    huart3.Instance          = USART3;
    huart3.Init.BaudRate     = 115200;
    huart3.Init.WordLength   = UART_WORDLENGTH_8B;
    huart3.Init.StopBits     = UART_STOPBITS_1;
    huart3.Init.Parity       = UART_PARITY_NONE;
    huart3.Init.Mode         = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK) {
        Error_Handler();
    }
}

/* GPIO Initialization -------------------------------------------------------*/
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_Init = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* SPI_CS pin (PA4) */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    GPIO_Init.Pin   = GPIO_PIN_4;
    GPIO_Init.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_Init.Pull  = GPIO_NOPULL;
    GPIO_Init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_Init);
}

/* Error Handler -------------------------------------------------------------*/
void Error_Handler(void)
{
    __disable_irq();
    while (1) {
        /* Blink or log error */
    }
}

#ifdef  USE_FULL_ASSERT
/* Assert Failed -------------------------------------------------------------*/
void assert_failed(uint8_t *file, uint32_t line)
{
    /* User can log file/line here */
}
#endif
