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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float altitude;
    float pressure;
    float accel_x;
    float accel_y;
    float accel_z;
    float angle_x;
    float angle_y;
    float angle_z;
} RocketData_t;

typedef struct {
    uint8_t rocket_fired;
    uint8_t waited_5s;
    uint8_t altitude_exceeded_500m;
    uint8_t angle_bigger_70;
    uint8_t altitude_decreasing;
    uint8_t gpio14_activated;
    uint8_t altitude_550;
    uint8_t gpio15_activated;
} RocketStatus_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define START_SIGNAL_LENGTH 5
#define STOP_SIGNAL_LENGTH 5
#define DATA_PACKET_LENGTH 36
#define UART_BUFFER_SIZE 50

// Start signal: 0xAA, 0x22, 0x8E, 0x0D, 0x0A
#define START_SIGNAL_0 0xAA
#define START_SIGNAL_1 0x22
#define START_SIGNAL_2 0x8E
#define START_SIGNAL_3 0x0D
#define START_SIGNAL_4 0x0A

// Stop signal: 0xAA, 0x24, 0x90, 0x0D, 0x0A
#define STOP_SIGNAL_0 0xAA
#define STOP_SIGNAL_1 0x24
#define STOP_SIGNAL_2 0x90
#define STOP_SIGNAL_3 0x0D
#define STOP_SIGNAL_4 0x0A

// Data packet header
#define DATA_HEADER 0xAB

// Thresholds
#define ALTITUDE_500M 500
#define ALTITUDE_550M 550
#define ANGLE_THRESHOLD 60.0f
#define ACCEL_Z_THRESHOLD 3.0f
#define GPIO_ACTIVATION_TIME 3000 // 3 seconds in milliseconds
#define WAIT_TIME_AFTER_FIRE 5000 // 5 seconds in milliseconds
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t uart_rx_buffer[UART_BUFFER_SIZE];
uint8_t uart_rx_byte = 0;
uint8_t uart_rx_index = 0;
uint8_t test_active = 0;
uint8_t data_received = 0;

RocketData_t current_data;
RocketData_t previous_data;
RocketStatus_t rocket_status;

uint32_t gpio14_start_time = 0;
uint32_t gpio15_start_time = 0;
uint32_t rocket_fire_time = 0;
uint8_t gpio14_active = 0;
uint8_t gpio15_active = 0;
uint8_t rocket_fired_detected = 0;
 uint8_t altitude_decreasing_detected = 0;
 uint8_t altitude_550_detected = 0;

 // Counters for multiple value detection
 uint8_t angle_high_count = 0;
 uint8_t altitude_decreasing_count = 0;

// GPIO 14 blinking variables
uint8_t gpio14_blink_count = 0;
uint8_t gpio14_blink_active = 0;
uint32_t gpio14_blink_start_time = 0;
uint8_t altitude_data_started = 0;

uint32_t system_tick = 0;
uint8_t uart_restart_needed = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void ProcessUARTData(void);
void ParseRocketData(uint8_t* data);
uint8_t CalculateChecksum(uint8_t* data, uint8_t length);
void UpdateRocketStatus(void);
void SendRocketStatus(void);
void ControlGPIOs(void);
uint32_t GetSystemTime(void);
float BytesToFloat(uint8_t* bytes);
uint32_t BytesToUint32(uint8_t* bytes);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void ProcessUARTData(void)
{
    // Check for start signal
    if (uart_rx_index >= START_SIGNAL_LENGTH) {
        if (uart_rx_buffer[0] == START_SIGNAL_0 &&
            uart_rx_buffer[1] == START_SIGNAL_1 &&
            uart_rx_buffer[2] == START_SIGNAL_2 &&
            uart_rx_buffer[3] == START_SIGNAL_3 &&
            uart_rx_buffer[4] == START_SIGNAL_4) {
            test_active = 1;
            // Reset status
            memset(&rocket_status, 0, sizeof(RocketStatus_t));
            rocket_fired_detected = 0;
            altitude_decreasing_detected = 0;
            altitude_550_detected = 0;
            gpio14_active = 0;
            gpio15_active = 0;
            altitude_data_started = 0;
            gpio14_blink_active = 0;
            gpio14_blink_count = 0;
            uart_rx_index = 0;
            return;
        }
    }
    
    // Check for stop signal
    if (uart_rx_index >= STOP_SIGNAL_LENGTH) {
        if (uart_rx_buffer[0] == STOP_SIGNAL_0 &&
            uart_rx_buffer[1] == STOP_SIGNAL_1 &&
            uart_rx_buffer[2] == STOP_SIGNAL_2 &&
            uart_rx_buffer[3] == STOP_SIGNAL_3 &&
            uart_rx_buffer[4] == STOP_SIGNAL_4) {
            test_active = 0;
            uart_rx_index = 0;
            return;
        }
    }
    
    // Check for data packet - removed test_active requirement
    if (uart_rx_index >= DATA_PACKET_LENGTH) {
        if (uart_rx_buffer[0] == DATA_HEADER) {
            // Debug: Simplified checksum verification - just check header and terminators
            if (uart_rx_buffer[DATA_PACKET_LENGTH - 2] == 0x0D &&
                uart_rx_buffer[DATA_PACKET_LENGTH - 1] == 0x0A) {

                // Auto-activate test mode when valid data is received
                if (!test_active) {
                    test_active = 1;
                    // Reset status
                    memset(&rocket_status, 0, sizeof(RocketStatus_t));
                    rocket_fired_detected = 0;
                    altitude_decreasing_detected = 0;
                    altitude_550_detected = 0;
                    gpio14_active = 0;
                    gpio15_active = 0;
                    altitude_data_started = 0;
                    gpio14_blink_active = 0;
                    gpio14_blink_count = 0;
                }

                ParseRocketData(uart_rx_buffer);
                data_received = 1;
            }
        }
        uart_rx_index = 0;
    }
}

void ParseRocketData(uint8_t* data)
{
    // Store previous data
    memcpy(&previous_data, &current_data, sizeof(RocketData_t));

    // Parse altitude (4 bytes, FLOAT32, big endian)
    current_data.altitude = BytesToFloat(&data[1]);
    
    // Parse pressure (4 bytes, FLOAT32, big endian)
    current_data.pressure = BytesToFloat(&data[5]);

    // Parse accelerometer data (4 bytes each, FLOAT32, big endian)
    current_data.accel_x = BytesToFloat(&data[9]);
    current_data.accel_y = BytesToFloat(&data[13]);
    current_data.accel_z = BytesToFloat(&data[17]);

    // Parse orientation angles (4 bytes each, FLOAT32, big endian)
    current_data.angle_x = BytesToFloat(&data[21]);
    current_data.angle_y = BytesToFloat(&data[25]);
    current_data.angle_z = BytesToFloat(&data[29]);

    // Debug: Blink GPIO 14 when ANY data packet is received (to test UART communication)
    if (!altitude_data_started) {
        altitude_data_started = 1;
        gpio14_blink_active = 1;
        gpio14_blink_count = 0;
        gpio14_blink_start_time = GetSystemTime();
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // Start first blink
    }
}

uint8_t CalculateChecksum(uint8_t* data, uint8_t length)
{
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

void UpdateRocketStatus(void)
{
    uint32_t current_time = GetSystemTime();

         // Bit 0: rocket is fired (accelerometer in z axis is bigger than 2)
     if (current_data.accel_z > 2.0f) {
         rocket_status.rocket_fired = 1;
         if (!rocket_fired_detected) {
             rocket_fire_time = current_time;
             rocket_fired_detected = 1;
         }
     }

    // Bit 1: waited for 5s after rocket fired
    if (rocket_fired_detected && (current_time - rocket_fire_time) >= WAIT_TIME_AFTER_FIRE) {
        rocket_status.waited_5s = 1;
    }

         // Bit 2: altitude is exceeded 500m
     if (current_data.altitude >= (float)ALTITUDE_500M) {
         rocket_status.altitude_exceeded_500m = 1;
     }

         // Bit 3: angle in x or y axis is bigger than 70 - need 5 consecutive values
     if (fabs(current_data.angle_x) >= 55.0f || fabs(current_data.angle_y) >= 55.0f) {
         angle_high_count++;
         if (angle_high_count >= 3) {
             rocket_status.angle_bigger_70 = 1;
         }
     } else {
         angle_high_count = 0; // Reset counter if angle goes below threshold
     }

         // Bit 4: altitude started to decrease - need 1 decreasing value
     if (data_received && current_data.altitude < previous_data.altitude) {
         rocket_status.altitude_decreasing = 1;
         if (!altitude_decreasing_detected) {
             altitude_decreasing_detected = 1;
         }
     }

    // Bit 5: gpio 14 activated
    if (gpio14_active) {
        rocket_status.gpio14_activated = 1;
    }
    
         // Bit 6: altitude is 550 - only after altitude started decreasing
     if (rocket_status.altitude_decreasing && current_data.altitude <= (float)ALTITUDE_550M) {
         rocket_status.altitude_550 = 1;
         if (!altitude_550_detected) {
             altitude_550_detected = 1;
         }
     }

    // Bit 7: gpio 15 is activated
    if (gpio15_active) {
        rocket_status.gpio15_activated = 1;
    }
}

void SendRocketStatus(void)
{
    uint8_t status_byte = 0;

    // Build status byte
    status_byte |= (rocket_status.rocket_fired << 0);
    status_byte |= (rocket_status.waited_5s << 1);
    status_byte |= (rocket_status.altitude_exceeded_500m << 2);
    status_byte |= (rocket_status.angle_bigger_70 << 3);
    status_byte |= (rocket_status.altitude_decreasing << 4);
    status_byte |= (rocket_status.gpio14_activated << 5);
    status_byte |= (rocket_status.altitude_550 << 6);
    status_byte |= (rocket_status.gpio15_activated << 7);
    
    // Send status packet: 0xAA, status_byte, 0x00, 0x1A, 0x0D, 0x0A
    uint8_t status_packet[6] = {0xAA, status_byte, 0x00, 0x1A, 0x0D, 0x0A};
    HAL_UART_Transmit(&huart1, status_packet, 6, 100);
}

void ControlGPIOs(void)
{
    uint32_t current_time = GetSystemTime();

    // Handle GPIO 14 blinking for data flow validation
    if (gpio14_blink_active) {
        if (gpio14_blink_count < 6) { // 3 blinks = 6 state changes (ON-OFF-ON-OFF-ON-OFF)
            if (gpio14_blink_count % 2 == 0) { // Even = ON
                if (current_time - gpio14_blink_start_time >= 1000) { // 1 second ON
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
                    gpio14_blink_count++;
                    gpio14_blink_start_time = current_time;
                }
            } else { // Odd = OFF
                if (current_time - gpio14_blink_start_time >= 1000) { // 1 second OFF
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
                    gpio14_blink_count++;
                    gpio14_blink_start_time = current_time;
                }
            }
        } else {
            // Blinking complete
            gpio14_blink_active = 0;
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        }
        return; // Don't process other GPIO logic while blinking
    }
    
         // GPIO 14: Activate for 3 seconds when altitude is decreasing after rocket angle is bigger than 60
     if (rocket_status.altitude_decreasing && rocket_status.angle_bigger_70 && !gpio14_active) {
         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
         gpio14_active = 1;
         gpio14_start_time = current_time;

         // Send GPIO activation message
         uint8_t gpio_msg[] = "GPIO 14 ACTIVATED!\r\n";
         HAL_UART_Transmit(&huart1, gpio_msg, sizeof(gpio_msg)-1, 100);
     }

    // Turn off GPIO 14 after 3 seconds
    if (gpio14_active && (current_time - gpio14_start_time) >= GPIO_ACTIVATION_TIME) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        gpio14_active = 0;
    }
    
         // GPIO 15: Activate for 3 seconds after altitude started decreasing and <= 550
     if (rocket_status.altitude_decreasing && current_data.altitude <= 550.0f && !gpio15_active) {
         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
         gpio15_active = 1;
         gpio15_start_time = current_time;

         // Send GPIO activation message
         uint8_t gpio_msg[] = "GPIO 15 ACTIVATED!\r\n";
         HAL_UART_Transmit(&huart1, gpio_msg, sizeof(gpio_msg)-1, 100);
     }

         // Turn off GPIO 15 after 3 seconds
     if (gpio15_active && (current_time - gpio15_start_time) >= GPIO_ACTIVATION_TIME) {
         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
         gpio15_active = 0;
     }

     // Turn off GPIOs when not active (normal operation)
     if (!gpio14_active && !gpio14_blink_active) {
         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
     }
     if (!gpio15_active) {
         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
     }
}

uint32_t GetSystemTime(void)
{
    return system_tick;
}

float BytesToFloat(uint8_t* bytes)
{
    uint32_t temp = BytesToUint32(bytes);
    return *(float*)&temp;
}

uint32_t BytesToUint32(uint8_t* bytes)
{
    return (bytes[0] << 24) | (bytes[1] << 16) | (bytes[2] << 8) | bytes[3];
}

// UART interrupt callback is no longer needed with polling
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//     if (huart->Instance == USART1) {
//         // Add received byte to buffer
//         if (uart_rx_index < UART_BUFFER_SIZE) {
//             uart_rx_buffer[uart_rx_index++] = uart_rx_byte;
//         } else {
//             uart_rx_index = 0; // Reset if buffer overflow
//         }
//
//         // Set flag to restart UART reception from main loop
//         uart_restart_needed = 1;
//     }
// }

void HAL_SYSTICK_Callback(void)
{
    system_tick++;
}
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  // Initialize variables
  memset(&rocket_status, 0, sizeof(RocketStatus_t));
  memset(&current_data, 0, sizeof(RocketData_t));
  memset(&previous_data, 0, sizeof(RocketData_t));

  // Start UART reception with polling instead of interrupt
  // HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);

  // Disable UART reception completely to avoid conflicts
  // __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
  // HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  // HAL_NVIC_EnableIRQ(USART1_IRQn);
  
  // Rocket system startup message
  uint8_t debug_msg[] = "Rocket Parachute System Starting\r\n";
  HAL_UART_Transmit(&huart1, debug_msg, sizeof(debug_msg)-1, 100);

  // Debug: Test GPIO 14 immediately to verify it's working
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

  // Debug: Test GPIO 15 immediately to verify it's working
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

  // Debug: Send a test message via UART to verify UART is working
  uint8_t test_msg[] = "UART Test OK\r\n";
  HAL_UART_Transmit(&huart1, test_msg, sizeof(test_msg)-1, 1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  // Rocket parachute deployment logic
  uint32_t current_time = HAL_GetTick();

  // Poll UART for received data
  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) {
      uint8_t received_byte = (uint8_t)(huart1.Instance->DR & 0xFF);

      // Add byte to buffer
      if (uart_rx_index < UART_BUFFER_SIZE) {
          uart_rx_buffer[uart_rx_index++] = received_byte;
      } else {
          uart_rx_index = 0; // Reset if buffer overflow
      }
  }

  // Process UART data (start/stop signals and data packets)
  ProcessUARTData();
  
  // Simple test: Activate test mode when any data is received (for testing)
  if (uart_rx_index > 0 && !test_active) {
      test_active = 1;
      memset(&rocket_status, 0, sizeof(RocketStatus_t));
      rocket_fired_detected = 0;
      altitude_decreasing_detected = 0;
      altitude_550_detected = 0;
      gpio14_active = 0;
      gpio15_active = 0;
      altitude_data_started = 0;
      gpio14_blink_active = 0;
      gpio14_blink_count = 0;
      angle_high_count = 0;
      altitude_decreasing_count = 0;

      // Initialize test data for simulation
      current_data.altitude = 0.0f;
      current_data.angle_x = 0.0f;
      current_data.angle_y = 0.0f;
      current_data.accel_z = 0.0f;
      previous_data.altitude = 0.0f;

      // Send activation message
      uint8_t activate_msg[] = "Test Mode Activated! Simulating rocket data...\r\n";
      HAL_UART_Transmit(&huart1, activate_msg, sizeof(activate_msg)-1, 100);

      // Reset buffer
      uart_rx_index = 0;
  }

  // Update rocket status and control GPIOs if test is active
  if (test_active) {
      // Simulate rocket data progression for testing
      static uint32_t simulation_time = 0;
      static uint8_t simulation_phase = 0;

      if (current_time - simulation_time >= 2000) { // Update every 2 seconds
          simulation_time = current_time;

          // Store previous data
          previous_data.altitude = current_data.altitude;

          // Simulate different rocket phases
          switch (simulation_phase) {
              case 0: // Rocket fired (accel_z > 3)
                  current_data.accel_z = 5.0f;
                  current_data.altitude = 100.0f;
                  current_data.angle_x = 10.0f;
                  current_data.angle_y = 15.0f;
                  simulation_phase++;
                  break;

              case 1: // Altitude increasing, angles normal
                  current_data.accel_z = 2.0f;
                  current_data.altitude = 300.0f;
                  current_data.angle_x = 20.0f;
                  current_data.angle_y = 25.0f;
                  simulation_phase++;
                  break;

              case 2: // Altitude > 500m
                  current_data.altitude = 600.0f;
                  current_data.angle_x = 30.0f;
                  current_data.angle_y = 35.0f;
                  simulation_phase++;
                  break;

              case 3: // Angle > 60 degrees
                  current_data.altitude = 700.0f;
                  current_data.angle_x = 65.0f; // This should trigger GPIO 14
                  current_data.angle_y = 70.0f;
                  simulation_phase++;
                  break;

              case 4: // Altitude decreasing
                  current_data.altitude = 650.0f; // Decreasing
                  current_data.angle_x = 65.0f;
                  current_data.angle_y = 70.0f;
                  simulation_phase++;
                  break;

                             case 5: // Altitude = 550m
                   current_data.altitude = 550.0f; // This should trigger GPIO 15
                   current_data.angle_x = 65.0f;
                   current_data.angle_y = 70.0f;
                   simulation_phase++;
                   break;

               case 6: // Test altitude 1000m - should blink both LEDs
                   current_data.altitude = 1000.0f;
                   current_data.angle_x = 65.0f;
                   current_data.angle_y = 70.0f;
                   simulation_phase++;
                   break;

               default: // Reset simulation
                   simulation_phase = 0;
                   current_data.altitude = 0.0f;
                   current_data.angle_x = 0.0f;
                   current_data.angle_y = 0.0f;
                   current_data.accel_z = 0.0f;
                   break;
          }

          // Process the simulated data
          data_received = 1;
      }

      if (data_received) {
          UpdateRocketStatus();
          ControlGPIOs();
          SendRocketStatus();
          data_received = 0;
      }
  }

  // System heartbeat - blink GPIO 14 every 2 seconds when not in test mode
  static uint32_t last_heartbeat_time = 0;
  if (!test_active && (current_time - last_heartbeat_time >= 2000)) {
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
      last_heartbeat_time = current_time;

      // Send heartbeat message with UART buffer status
      uint8_t heartbeat_msg[100];
      sprintf((char*)heartbeat_msg, "System Ready - UART Index: %d\r\n", uart_rx_index);
      HAL_UART_Transmit(&huart1, heartbeat_msg, strlen((char*)heartbeat_msg), 100);
  }

  // Debug: Show test mode status
  static uint32_t last_debug_time = 0;
  if (test_active && (current_time - last_debug_time >= 1000)) {
      last_debug_time = current_time;

      // Send debug message when in test mode - simplified without sprintf
      uint8_t debug_msg[] = "Simulation Active - Phase: ";
      HAL_UART_Transmit(&huart1, debug_msg, sizeof(debug_msg)-1, 100);

      // Send phase number
      uint8_t phase_msg[10];
      if (current_data.altitude == 100.0f) {
          uint8_t phase[] = "0 (Rocket Fired)\r\n";
          HAL_UART_Transmit(&huart1, phase, sizeof(phase)-1, 100);
      } else if (current_data.altitude == 300.0f) {
          uint8_t phase[] = "1 (Normal Flight)\r\n";
          HAL_UART_Transmit(&huart1, phase, sizeof(phase)-1, 100);
      } else if (current_data.altitude == 600.0f) {
          uint8_t phase[] = "2 (Altitude > 500m)\r\n";
          HAL_UART_Transmit(&huart1, phase, sizeof(phase)-1, 100);
      } else if (current_data.altitude == 700.0f) {
          uint8_t phase[] = "3 (Angle > 60deg)\r\n";
          HAL_UART_Transmit(&huart1, phase, sizeof(phase)-1, 100);
      } else if (current_data.altitude == 650.0f) {
          uint8_t phase[] = "4 (Altitude Decreasing)\r\n";
          HAL_UART_Transmit(&huart1, phase, sizeof(phase)-1, 100);
             } else if (current_data.altitude == 550.0f) {
           uint8_t phase[] = "5 (Altitude = 550m)\r\n";
           HAL_UART_Transmit(&huart1, phase, sizeof(phase)-1, 100);
       } else if (current_data.altitude == 1000.0f) {
           uint8_t phase[] = "6 (Test Altitude 1000m)\r\n";
           HAL_UART_Transmit(&huart1, phase, sizeof(phase)-1, 100);
       } else {
           uint8_t phase[] = "Reset\r\n";
           HAL_UART_Transmit(&huart1, phase, sizeof(phase)-1, 100);
       }
  }
  
  HAL_Delay(100); // Longer delay to make it easier to see
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
