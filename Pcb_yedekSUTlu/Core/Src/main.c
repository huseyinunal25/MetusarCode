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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "BME280_STM32.h"
#include <stdio.h>
#include <string.h>
#include "lwgps/lwgps.h"
#include <stdbool.h>

//SUT
#include <stdint.h>
#include "stm32f1xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    int16_t x, y, z;
} sensor_data_t;

typedef struct {
    float roll, pitch, yaw;
} orientation_t;

/* Kalman Filter structure for each angle */
typedef struct {
    float Q;  // Process noise covariance
    float R;  // Measurement noise covariance
    float P;  // Estimate error covariance
    float K;  // Kalman gain
    float X;  // State estimate
} kalman_filter_t;

//SUT
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
#define PACKET_HEADER_SIZE      3
#define BUFFER_SIZE             150
#define IMU_DATA_SIZE           14
#define MEASUREMENT_DELAY       190
#define LORA_MAX_PAYLOAD        58

#define GPS_DEBUG_RAW_NMEA      1

#define TARGET_ADDR_HIGH        0x00
#define TARGET_ADDR_LOW         0x02
#define CHANNEL                 0x17

#define MPU9250_ACCEL_XOUT_H    0x3B  // 59
#define MPU9250_GYRO_XOUT_H     0x43  // 67

#define ACCEL_SCALE_FACTOR      16384.0f  // ±2g
#define GYRO_SCALE_FACTOR       131.0f    // ±250 °/s
#define RAD_TO_DEG              57.2958f

//SUT
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
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
I2C_HandleTypeDef    hi2c1;
SPI_HandleTypeDef    hspi1;
UART_HandleTypeDef   huart1;    /* Debug */
UART_HandleTypeDef   huart2;    /* GPS */
UART_HandleTypeDef   huart3;    /* LoRa */

static orientation_t orientation = {0};
static uint32_t      last_time = 0;

// Kalman filters for each angle
static kalman_filter_t kalman_roll = {0};
static kalman_filter_t kalman_pitch = {0};
static kalman_filter_t kalman_yaw = {0};

lwgps_t              gps;
uint8_t              rx_buffer[128];
uint8_t              rx_index = 0;
uint8_t              rx_data = 0;
uint32_t             gps_data_received_count = 0;

float Temperature, Pressure, altitude;
float gyro_x, gyro_y, gyro_z;
float accel_x, accel_y, accel_z;

//SUT
TelemetryData_t telemetry_data = {0};
uint8_t rx_buffer_sut[PACKET_SIZE];
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
void initialize_sensors(void);
void read_bme280_data(void);
void read_accelerometer_data(sensor_data_t *accel_data);
void read_gyroscope_data(sensor_data_t *gyro_data);
void calculate_orientation(sensor_data_t *accel_data, sensor_data_t *gyro_data, orientation_t *orientation);
void transmit_sensor_packet(int alt, float ax, float ay, float az, float gx, float gy, float gz, float lat, float lon);
void print_gps_debug_info(void);
void check_uart_errors(void);
void mpu9250_read_data(uint8_t reg, uint8_t *data, uint8_t len);

// Kalman filter functions
void kalman_filter_init(kalman_filter_t *kf, float Q, float R);
float kalman_filter_update(kalman_filter_t *kf, float measurement, float dt);
void initialize_kalman_filters(void);
void print_kalman_debug_info(void);
void adjust_kalman_parameters(void);
bool is_device_stationary(void);
void set_kalman_responsiveness(bool high_responsiveness);

//SUT
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
void float_to_big_endian_bytes(float value, uint8_t *buf) {
    uint8_t *p = (uint8_t*)&value;
    buf[0] = p[3];
    buf[1] = p[2];
    buf[2] = p[1];
    buf[3] = p[0];
}

void send_uart1_data_package(float altitude, float pressure,
                             float acc_x, float acc_y, float acc_z,
                             float ori_x, float ori_y, float ori_z) {
    uint8_t packet[36];
    int i = 0;

    packet[i++] = 0xAB;

    float_to_big_endian_bytes(altitude,  &packet[i]); i += 4;
    float_to_big_endian_bytes(pressure,  &packet[i]); i += 4;
    float_to_big_endian_bytes(acc_x,     &packet[i]); i += 4;
    float_to_big_endian_bytes(acc_y,     &packet[i]); i += 4;
    float_to_big_endian_bytes(acc_z,     &packet[i]); i += 4;
    float_to_big_endian_bytes(ori_x,     &packet[i]); i += 4;
    float_to_big_endian_bytes(ori_y,     &packet[i]); i += 4;
    float_to_big_endian_bytes(ori_z,     &packet[i]); i += 4;

    // Checksum: sum of bytes 0 to 32 (mod 256)
    uint16_t sum = 0;
    for (int j = 0; j < 33; ++j) {  // Düzeltme: 0-32 arası checksum
        sum += packet[j];
    }
    packet[i++] = sum % 256;

    packet[i++] = 0x0D;
    packet[i++] = 0x0A;

    HAL_UART_Transmit(&huart1, packet, 36, HAL_MAX_DELAY);
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

    if (huart->Instance == USART1) {
        // Process the received byte immediately
        ProcessIncomingData(incoming_buffer[buffer_index]);

        // Move to next buffer position
        buffer_index = (buffer_index + 1) % (PACKET_SIZE * 4);

        // Continue receiving next byte immediately - no delays
        HAL_UART_Receive_IT(&huart1, &incoming_buffer[buffer_index], 1);
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

/* Kalman Filter Implementation ----------------------------------------------*/
void kalman_filter_init(kalman_filter_t *kf, float Q, float R)
{
    kf->Q = Q;  // Process noise covariance
    kf->R = R;  // Measurement noise covariance
    kf->P = 1.0f;  // Initial estimate error covariance
    kf->K = 0.0f;  // Initial Kalman gain
    kf->X = 0.0f;  // Initial state estimate
}

float kalman_filter_update(kalman_filter_t *kf, float measurement, float dt)
{
    // Prediction step
    float X_pred = kf->X;  // State prediction (assuming constant velocity model)
    float P_pred = kf->P + kf->Q * dt;  // Error covariance prediction
    
    // Update step
    kf->K = P_pred / (P_pred + kf->R);  // Kalman gain
    kf->X = X_pred + kf->K * (measurement - X_pred);  // State update
    kf->P = (1.0f - kf->K) * P_pred;  // Error covariance update
    
    return kf->X;
}

void initialize_kalman_filters(void)
{
    // Initialize Kalman filters with appropriate noise parameters
    // Q: Process noise (gyroscope drift), R: Measurement noise (accelerometer noise)
    // Lower Q = more trust in model, Higher R = more trust in measurements
    
    // MORE RESPONSIVE SETTINGS (faster angle changes):
    // Roll and Pitch: Good accelerometer reference available
    kalman_filter_init(&kalman_roll, 0.01f, 0.05f);   // Roll filter - Higher Q, Lower R = more responsive
    kalman_filter_init(&kalman_pitch, 0.01f, 0.05f);  // Pitch filter - Higher Q, Lower R = more responsive
    
    // Yaw: No absolute reference, rely more on gyroscope
    kalman_filter_init(&kalman_yaw, 0.01f, 0.2f);    // Yaw filter - Higher Q = more responsive
    
    // ALTERNATIVE: STABLE SETTINGS (slower but smoother):
    // kalman_filter_init(&kalman_roll, 0.001f, 0.1f);   // Roll filter - Lower Q, Higher R = more stable
    // kalman_filter_init(&kalman_pitch, 0.001f, 0.1f);  // Pitch filter - Lower Q, Higher R = more stable
    // kalman_filter_init(&kalman_yaw, 0.001f, 0.5f);    // Yaw filter - Lower Q = more stable
    
    // Initialize orientation to zero
    orientation.roll = 0.0f;
    orientation.pitch = 0.0f;
    orientation.yaw = 0.0f;
    
    // Initialize time reference
    last_time = HAL_GetTick();
}

void print_kalman_debug_info(void)
{
    char dbg[256];
    int len = sprintf(dbg,
        "Kalman Debug: Roll(K=%.3f,P=%.3f,Q=%.4f) Pitch(K=%.3f,P=%.3f,Q=%.4f) Yaw(K=%.3f,P=%.3f,Q=%.4f) Motion:%s\r\n",
        kalman_roll.K, kalman_roll.P, kalman_roll.Q,
        kalman_pitch.K, kalman_pitch.P, kalman_pitch.Q,
        kalman_yaw.K, kalman_yaw.P, kalman_yaw.Q,
        is_device_stationary() ? "Stationary" : "Moving"
    );
#if 0  // Set to 1 to enable Kalman debug output
    HAL_UART_Transmit(&huart1, (uint8_t*)dbg, len, HAL_MAX_DELAY);
#endif
}

// Function to manually set responsiveness mode
void set_kalman_responsiveness(bool high_responsiveness)
{
    if (high_responsiveness) {
        // High responsiveness mode - angles change faster
        kalman_filter_init(&kalman_roll, 0.05f, 0.02f);   // Very high Q, very low R
        kalman_filter_init(&kalman_pitch, 0.05f, 0.02f);  // Very high Q, very low R
        kalman_filter_init(&kalman_yaw, 0.05f, 0.1f);     // Very high Q
    } else {
        // Stable mode - angles change slower but smoother
        kalman_filter_init(&kalman_roll, 0.001f, 0.1f);   // Low Q, high R
        kalman_filter_init(&kalman_pitch, 0.001f, 0.1f);  // Low Q, high R
        kalman_filter_init(&kalman_yaw, 0.001f, 0.5f);    // Low Q
    }
}

bool is_device_stationary(void)
{
    // Calculate the magnitude of acceleration and gyroscope
    float accel_magnitude = sqrtf(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z);
    float gyro_magnitude = sqrtf(gyro_x*gyro_x + gyro_y*gyro_y + gyro_z*gyro_z);
    
    // Device is considered stationary if:
    // - Acceleration is close to 1g (9.81 m/s²) with some tolerance
    // - Gyroscope readings are very low
    float accel_tolerance = 0.5f;  // 0.5 m/s² tolerance
    float gyro_threshold = 0.5f;   // 0.5 °/s threshold
    
    return (fabsf(accel_magnitude - 9.81f) < accel_tolerance) && (gyro_magnitude < gyro_threshold);
}

void adjust_kalman_parameters(void)
{
    static uint32_t last_adjustment = 0;
    uint32_t now = HAL_GetTick();
    
    // Adjust parameters every 2 seconds (more frequent for better responsiveness)
    if (now - last_adjustment > 2000) {
        float motion_intensity = sqrtf(gyro_x*gyro_x + gyro_y*gyro_y + gyro_z*gyro_z);
        
        if (is_device_stationary()) {
            // When stationary, use stable settings
            kalman_roll.Q = 0.001f;
            kalman_pitch.Q = 0.001f;
            kalman_yaw.Q = 0.001f;
        } else if (motion_intensity > 10.0f) {
            // High motion - very responsive
            kalman_roll.Q = 0.05f;
            kalman_pitch.Q = 0.05f;
            kalman_yaw.Q = 0.05f;
        } else if (motion_intensity > 5.0f) {
            // Medium motion - responsive
            kalman_roll.Q = 0.02f;
            kalman_pitch.Q = 0.02f;
            kalman_yaw.Q = 0.02f;
        } else {
            // Low motion - moderately responsive
            kalman_roll.Q = 0.01f;
            kalman_pitch.Q = 0.01f;
            kalman_yaw.Q = 0.01f;
        }
        
        last_adjustment = now;
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
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

  sensor_data_t accel_data, gyro_data;
  uint32_t debug_counter = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  initialize_sensors();

  /* Initialize Kalman filters for angle estimation */
  initialize_kalman_filters();
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    	/* BME280 */
        read_bme280_data();

        /* IMU */
        read_accelerometer_data(&accel_data);
        read_gyroscope_data(&gyro_data);
        calculate_orientation(&accel_data, &gyro_data, &orientation);

        /* Adjust Kalman filter parameters based on motion state */
        adjust_kalman_parameters();

        /* Debug GPS data before transmission */
#if 0
        char debug_gps[100];
        int gps_debug_len = sprintf(debug_gps, "GPS: valid=%d fix=%d lat=%.6f lon=%.6f\r\n",
                                   gps.is_valid, gps.fix, gps.latitude, gps.longitude);
        //HAL_UART_Transmit(&huart1, (uint8_t*)debug_gps, gps_debug_len, HAL_MAX_DELAY);
#endif

        /* Transmit via LoRa */
       /* transmit_sensor_packet(
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

        // Add this call to send the binary data package over UART1
        send_uart1_data_package(
            altitude,
            Pressure / 100.0f,
            accel_data.x / ACCEL_SCALE_FACTOR,
            accel_data.y / ACCEL_SCALE_FACTOR,
            accel_data.z / ACCEL_SCALE_FACTOR,
            orientation.roll,
            orientation.pitch,
            orientation.yaw
        );*/

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
            //HAL_UART_Transmit(&huart1, (uint8_t*)debug_bme, debug_len, HAL_MAX_DELAY);
#endif

            /* Debug Kalman filter performance */
            //print_kalman_debug_info();
            // adjust_kalman_parameters(); // Adjust parameters after each debug cycle - Moved to IMU calculation

            debug_counter = 0;
        }
        HAL_Delay(MEASUREMENT_DELAY);

        //SUT
        if (packet_received) {
		  packet_received = 0;

		  // Process the received packet
		  ParseTelemetryPacket(rx_buffer_sut);
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
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
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
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

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

/* Calculate orientation using Kalman filtering ------------------------------*/
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

    // Apply simple gyroscope bias compensation (temperature drift)
    static float gyro_bias_x = 0.0f, gyro_bias_y = 0.0f, gyro_bias_z = 0.0f;
    static uint32_t bias_update_counter = 0;
    
    if (is_device_stationary()) {
        // Update bias when stationary
        gyro_bias_x = 0.95f * gyro_bias_x + 0.05f * gyro_x;
        gyro_bias_y = 0.95f * gyro_bias_y + 0.05f * gyro_y;
        gyro_bias_z = 0.95f * gyro_bias_z + 0.05f * gyro_z;
        bias_update_counter++;
    }
    
    // Apply bias compensation
    float gyro_x_comp = gyro_x - gyro_bias_x;
    float gyro_y_comp = gyro_y - gyro_bias_y;
    float gyro_z_comp = gyro_z - gyro_bias_z;

    // Calculate accelerometer-based angles
    float accel_roll  = atan2f(accel_y, sqrtf(accel_x*accel_x + accel_z*accel_z)) * RAD_TO_DEG;
    float accel_pitch = atan2f(-accel_x, sqrtf(accel_y*accel_y + accel_z*accel_z)) * RAD_TO_DEG;
    
    // Apply Kalman filtering to roll and pitch
    o->roll = kalman_filter_update(&kalman_roll, accel_roll, dt);
    o->pitch = kalman_filter_update(&kalman_pitch, accel_pitch, dt);
    
    // For yaw, we only have gyroscope data, so we use a simple integration with Kalman filtering
    // to reduce drift
    float gyro_yaw_rate = gyro_z_comp;
    float predicted_yaw = o->yaw + gyro_yaw_rate * dt;
    
    // Use a higher measurement noise for yaw since we don't have absolute reference
    // This makes the filter rely more on the gyroscope prediction
    o->yaw = kalman_filter_update(&kalman_yaw, predicted_yaw, dt);
    
    // Normalize yaw to -180 to +180 degrees
    while (o->yaw > 180.0f) o->yaw -= 360.0f;
    while (o->yaw < -180.0f) o->yaw += 360.0f;
    
    // Apply angle limits to prevent unrealistic values
    o->roll = fmaxf(-90.0f, fminf(90.0f, o->roll));
    o->pitch = fmaxf(-90.0f, fminf(90.0f, o->pitch));
}

/* Transmit packet over LoRa (USART3) ----------------------------------------*/
void transmit_sensor_packet(int alt, float ax, float ay, float az, float gx, float gy, float gz, float lat, float lon)
{
    char buf[BUFFER_SIZE];
    int len;
    if (!gps.is_valid) {
        len = sprintf(buf, "%d,%d,%d,%d,0.0000,0.0000,%.2f\n",
                      alt, (int)orientation.roll, (int)orientation.pitch, (int)orientation.yaw, gps.altitude);
    } else {
        len = sprintf(buf, "%d,%d,%d,%d,%.4f,%.4f,%.2f\n",
                      alt, (int)orientation.roll, (int)orientation.pitch, (int)orientation.yaw,
                      lat, lon, gps.altitude);
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

//SUT
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

                    // Copy complete packet to rx_buffer_sut
                    uint16_t start_idx = (buffer_index - PACKET_SIZE + 1) % (PACKET_SIZE * 4);
                    for (uint8_t i = 0; i < PACKET_SIZE; i++) {
                        rx_buffer_sut[i] = incoming_buffer[(start_idx + i) % (PACKET_SIZE * 4)];
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
