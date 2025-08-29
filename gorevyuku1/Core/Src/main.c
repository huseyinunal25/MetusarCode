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
#include <stdbool.h>

/* Private defines -----------------------------------------------------------*/
#define PACKET_HEADER_SIZE      3
#define BUFFER_SIZE             150
#define MEASUREMENT_DELAY       190
#define LORA_MAX_PAYLOAD        58

#define GPS_DEBUG_RAW_NMEA      1

#define TARGET_ADDR_HIGH        0x00
#define TARGET_ADDR_LOW         0x03
#define CHANNEL                 0x18

// HX711 pin defines
#define DT_PIN GPIO_PIN_12 //input
#define DT_PORT GPIOB
#define SCK_PIN GPIO_PIN_13
#define SCK_PORT GPIOB
/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef    hi2c1;
UART_HandleTypeDef   huart1;    /* Debug */
UART_HandleTypeDef   huart2;    /* GPS */
UART_HandleTypeDef   huart3;    /* LoRa */
TIM_HandleTypeDef    htim2;


lwgps_t              gps;
uint8_t              rx_buffer[128];
uint8_t              rx_index = 0;
uint8_t              rx_data = 0;
uint32_t             gps_data_received_count = 0;

float Temperature, Pressure, altitude;
float gyro_x, gyro_y, gyro_z;
float accel_x, accel_y, accel_z;

uint32_t tare = -55734;
float knownOriginal = 500000;  // in milli gram
float knownHX711 = 211170;
int weight;
char buffer[20];
int size;

int32_t  weight_total = 0;
uint32_t weight_time;
int weight_flag = 0;

int32_t hxData_array[50];
int HX_period = 0;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);

//void call_weigh(void);
void initialize_sensors(void);
void read_bme280_data(void);
void transmit_sensor_packet(int alt, float lat, float lon, int wei);
void print_gps_debug_info(void);
void check_uart_errors(void);
void microDelay(uint16_t delay)
{
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  while (__HAL_TIM_GET_COUNTER(&htim2) < delay);
}
int32_t getHX711(void)
{
  uint32_t data = 0;
  uint32_t startTime = HAL_GetTick();
  while(HAL_GPIO_ReadPin(DT_PORT, DT_PIN) == GPIO_PIN_SET)
  {
    if(HAL_GetTick() - startTime > 1000)
      return 0;
  }
  for(int8_t len=0; len<24 ; len++)
  {
    HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_SET);
    microDelay(15);
    data = data << 1;
    HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_RESET);
    microDelay(15);
    if(HAL_GPIO_ReadPin(DT_PORT, DT_PIN) == GPIO_PIN_SET)
      data ++;
  }
  if(data & 0x800000)       // 24. bit işaret biti
      data |= 0xFF000000;
  HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_SET);
  microDelay(1);
  HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_RESET);
  microDelay(1);
  return data;
}


int weigh(int32_t weight_total)
{

  int32_t  samples = 50;
  int milligram;
  float coefficient;

  int32_t average = (int32_t)(weight_total / samples);
  coefficient = knownOriginal / knownHX711;
  milligram = (int)(average-tare)*coefficient;
  return milligram;
}

void data_HX(void){
	if (HX_period < 50){
	weight_total += getHX711();
	HX_period += 1;
	}
	else if (HX_period == 50) {
		weight = weigh(weight_total);
		HX_period = 0;
		weight_total = 0;
	}
}

/* UART MSP Initialization (GPIO + NVIC) -------------------------------------*/


/* UART IRQ Handlers ---------------------------------------------------------*/

/* UART Rx Complete Callback for GPS (USART2) --------------------------------*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) {
        /* Re-arm reception */
        HAL_UART_Receive_IT(&huart2, &rx_data, 1);

        if (rx_data != '\n' && rx_index < sizeof(rx_buffer) - 1) {
            rx_buffer[rx_index++] = rx_data;
            gps_data_received_count++;
        }
        else {
            if (rx_index > 0) {
                rx_buffer[rx_index] = '\0';
                lwgps_process(&gps, rx_buffer, rx_index);
            }
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
#if 0
    HAL_UART_Transmit(&huart1, (uint8_t*)dbg, len, HAL_MAX_DELAY);
#endif
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

    MX_TIM2_Init();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
    initialize_sensors();


    /* Configure GPS to output at 5Hz */
    uint8_t setRate5Hz[] = {
        0xB5, 0x62,
        0x06, 0x08,
        0x06, 0x00,
        0xC8, 0x00,   // 200 ms = 5 Hz
        0x01, 0x00,   // navRate = 1
        0x01, 0x00,   // timeRef = GPS time
        0xDE, 0x6A    // Checksum
    };

    HAL_Delay(1000);
    HAL_UART_Transmit(&huart2, setRate5Hz, sizeof(setRate5Hz), HAL_MAX_DELAY);
    HAL_Delay(1000);

    lwgps_init(&gps);
    HAL_UART_Receive_IT(&huart2, &rx_data, 1);
    HAL_Delay(200);
    print_gps_debug_info();


    uint32_t debug_counter = 0;

    HAL_TIM_Base_Start(&htim2);
    HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_SET);
    HAL_Delay(10);
	HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);



    while (1) {
        /* BME280 */
        read_bme280_data();

        data_HX();
        /*call_weigh();*/

        /* Transmit via LoRa */
        transmit_sensor_packet(
            (int)altitude,
            gps.latitude,
            gps.longitude,
			weight
        );


        // Add this call to send the binary data package over UART1

        if (++debug_counter >= 50) {
#if 0
            print_gps_debug_info();
#endif
            check_uart_errors();

            /* Debug BME280 data */
#if 0
            char debug_bme[100];
            int debug_len = sprintf(debug_bme, "BME280: T=%.2f P=%.2f Alt=%.2f\r\n",
                                   Temperature, Pressure, altitude);
            HAL_UART_Transmit(&huart1, (uint8_t*)debug_bme, debug_len, HAL_MAX_DELAY);
#endif

            debug_counter = 0;
        }
        HAL_Delay(MEASUREMENT_DELAY);
    }
}

/*void call_weigh(void){
	uint32_t current_time = HAL_GetTick();

	if (weight_flag == 0){
		weight_time = current_time;
		weight_flag = 1;
	}
	else if (weight_flag == 1 && (current_time - weight_time > 2000)){
		weight = weigh();
		weight_flag = 0;
	}

}*/

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
/* Transmit packet over LoRa (USART3) ----------------------------------------*/
void transmit_sensor_packet(int alt, float lat, float lon, int wei)
{
    char buf[BUFFER_SIZE];
    int len;
    if (!gps.is_valid) {
        len = sprintf(buf, "%d,0.0000,0.0000,%.2f,G %d\n",
                      alt, gps.altitude,wei);
    } else {
        len = sprintf(buf, "%d,%.4f,%.4f,%.2f,G %d\n",
                      alt, lat, lon, gps.altitude, wei);
    }
    uint8_t packet[PACKET_HEADER_SIZE + len];
    packet[0] = TARGET_ADDR_HIGH;
    packet[1] = TARGET_ADDR_LOW;
    packet[2] = CHANNEL;
    memcpy(&packet[3], buf, len);
    HAL_UART_Transmit(&huart3, packet, 3 + len, HAL_MAX_DELAY);

    /* Send additional line break to ensure proper separation */
    uint8_t line_break[] = "\r\n";
    HAL_UART_Transmit(&huart3, line_break, 2, HAL_MAX_DELAY);

    /* Add a small delay to ensure proper line separation */
    HAL_Delay(10);
}


/* System Clock Configuration ------------------------------------------------*/
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef       RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef       RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK   |
                                       RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_PCLK1  |
                                       RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
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

static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

	/*Configure GPIO pin : PB8 */
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* SPI_CS pin (PA4) */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = GPIO_PIN_4;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
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
