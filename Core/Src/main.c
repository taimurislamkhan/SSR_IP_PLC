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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "encoder.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "modbusSlave.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Define AC line frequency (50Hz or 60Hz)
#define AC_LINE_FREQUENCY 60  // Change to 60 for 60Hz systems

// Calculate half-cycle period in microseconds based on frequency
#define HALF_CYCLE_PERIOD_US (1000000 / (AC_LINE_FREQUENCY * 2))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define LINE_VOLTAGE 220.0f // Assuming 220V AC line voltage. Please change if different.
static float energy_joules[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static uint32_t last_energy_tick = 0;

uint8_t RxData[256];
uint8_t TxData[256];




void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (RxData[0] == SLAVE_ID)
	{
		switch (RxData[1]){
		case 0x03:
			readHoldingRegs();
			break;
		case 0x04:
			readInputRegs();
			break;
		case 0x01:
			readCoils();
			break;
		case 0x02:
			readInputs();
			break;
		case 0x06:
			writeSingleReg();
			break;
		case 0x10:
			writeHoldingRegs();
			break;
		case 0x05:
			writeSingleCoil();
			break;
		case 0x0F:
			writeMultiCoils();
			break;
		default:
			modbusException(ILLEGAL_FUNCTION);
			break;
		}
	}

	HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, 256);
}


uint16_t dimmerValue = 0;         // Dimmer value (0-100%)
char rxBuffer[10];              // Buffer to store received characters
uint8_t rxIndex = 0;            // Index for received characters
uint8_t rxComplete = 0;         // Flag to indicate reception complete
uint8_t rxData;                 // Single byte for UART reception

// Variables for AC dimming control
// Dimmer values and delays for each SSR (1-4)
volatile uint16_t dimmerValues[4] = {0, 0, 0, 0};  // Dimmer values (0-100%) for each SSR
volatile uint16_t dimmerDelaysUs[4] = {9999, 9999, 9999, 9999};  // Calculated delay times in microseconds
volatile uint8_t triacTriggered[4] = {0, 0, 0, 0};  // Flags to indicate if triacs have been triggered in current half-cycle
volatile uint8_t ssrControlByte = 0x0F;  // Control byte for SSRs (lower 4 bits control SSR1-4)

//// For backward compatibility with existing code
//#define dimmerValue dimmerValues[0]
//#define dimmerDelayUs dimmerDelaysUs[0]

uint16_t ADC_VAL[5];

// Zero current calibration values for each channel
uint16_t zeroCurrentCal[5] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void get_energy_values(float* current_mA, uint8_t ssr_states[]);
void ProcessReceivedValue(void); // Function to process received data
void CalibrateZeroCurrent(void); // Function to calibrate zero current values
void SetSSRState(uint8_t ssrNum, uint8_t state); // Function to control individual SSRs
void read_modbus_address(void); // Function to read slave address from GPIOs
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void read_modbus_address(void)
{
    // Set ADDR_OUT pins to high to power the DIP switches or jumpers
    HAL_GPIO_WritePin(ADDR_OUT_1_GPIO_Port, ADDR_OUT_1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ADDR_OUT_2_GPIO_Port, ADDR_OUT_2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ADDR_OUT_3_GPIO_Port, ADDR_OUT_3_Pin, GPIO_PIN_SET);

    // A small delay might be needed for signals to stabilize
    HAL_Delay(1);

    // Read the state of the ADDR_IN pins
    // Note: ADDR_IN_1__Pin is used as provided by the user.
    uint8_t addr_bit0 = HAL_GPIO_ReadPin(ADDR_IN_1__GPIO_Port, ADDR_IN_1__Pin); // LSB
    uint8_t addr_bit1 = HAL_GPIO_ReadPin(ADDR_IN_2_GPIO_Port, ADDR_IN_2_Pin);
    uint8_t addr_bit2 = HAL_GPIO_ReadPin(ADDR_IN_3_GPIO_Port, ADDR_IN_3_Pin); // MSB

    // Combine the bits to form the 3-bit slave address
    SLAVE_ID = (addr_bit2 << 2) | (addr_bit1 << 1) | addr_bit0;
    SLAVE_ID ++;
}

uint64_t timeout = 500; // 500 milliseconds
uint64_t previous_time=0;
#define I2C_TIMEOUT 100

uint8_t received_data[13]={0,0,0,0,0,0,0};
uint8_t sent_data[13];


//void I2C_Error_Handler(I2C_HandleTypeDef *hi2c) {
//    // Check for errors and clear flags
//    if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_AF)) {
//        // Clear the Acknowledge Failure Flag
//        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);
//    }
//
//    if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_ARLO)) {
//        // Clear the Arbitration Lost Flag
//        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ARLO);
//    }
//
//    if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BERR)) {
//        // Clear the Bus Error Flag
//        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_BERR);
//    }
//
//    if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_OVR)) {
//        // Clear the Overrun/Underrun Flag
//        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_OVR);
//    }
//
//    // Reset the I2C peripheral
//    HAL_I2C_DeInit(hi2c);
//    HAL_I2C_Init(hi2c);
//
//    // Re-enable I2C listening after resetting the peripheral
//    HAL_I2C_EnableListen_IT(hi2c);
//
//    // Wait for the bus to become free
//    while (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY) {
//        HAL_Delay(1);
//    }
//}
//
//#define MAX_CONSECUTIVE_ERRORS 5
//uint8_t consecutive_errors = 0;
//
//
//void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
//{
//	HAL_StatusTypeDef status;
//
////     Refresh IWDG counter to prevent reset
//        if (HAL_IWDG_Refresh(&hiwdg) != HAL_OK)
//        {
//            Error_Handler(); // Implement your error handler
//        }
//	if (TransferDirection == I2C_DIRECTION_TRANSMIT)
//	{
//		do {
//			status = HAL_I2C_Slave_Sequential_Receive_IT(hi2c, received_data, 13, I2C_LAST_FRAME);
//			if (status != HAL_OK) {
//				// Receive error
//				I2C_Error_Handler(&hi2c1);
//			} else {
//				// Update the states and timers after receiving new data
//				previous_time = HAL_GetTick();
//				consecutive_errors = 0;
//			}
//		} while (status != HAL_OK && consecutive_errors < MAX_CONSECUTIVE_ERRORS);
//	}
//	else
//	{
//		do {
//			status = HAL_I2C_Slave_Sequential_Transmit_IT(hi2c, sent_data, 13, I2C_LAST_FRAME);
//			if (status != HAL_OK) {
//				// Transmission error
//				I2C_Error_Handler(&hi2c1);
//			} else {
//				consecutive_errors = 0;
//			}
//		} while (status != HAL_OK && consecutive_errors < MAX_CONSECUTIVE_ERRORS);
//	}
//}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
	// Re-enable I2C listening after a complete transaction
	HAL_I2C_EnableListen_IT(hi2c);
}

void no_color(void) {
    HAL_GPIO_WritePin(OUTPUT2_GPIO_Port, OUTPUT2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(OUTPUT3_GPIO_Port, OUTPUT3_Pin, GPIO_PIN_RESET);
}

void red_color(void) {
    HAL_GPIO_WritePin(OUTPUT2_GPIO_Port, OUTPUT2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(OUTPUT3_GPIO_Port, OUTPUT3_Pin, GPIO_PIN_SET);
}

void green_color(void) {
    HAL_GPIO_WritePin(OUTPUT2_GPIO_Port, OUTPUT2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(OUTPUT3_GPIO_Port, OUTPUT3_Pin, GPIO_PIN_RESET);
}

void yellow_color(void) {
    HAL_GPIO_WritePin(OUTPUT2_GPIO_Port, OUTPUT2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(OUTPUT3_GPIO_Port, OUTPUT3_Pin, GPIO_PIN_SET);
}

void set_led_color(uint8_t led_color) {
    switch (led_color) {
        case 0: // No color
            no_color();
            break;
        case 1: // Red color
        	green_color();
            break;
        case 2: // Green color
            red_color();
            break;
        case 3: // Yellow color
            yellow_color();
            break;
        default:
            break;
    }
}

// Function to calibrate zero current values by taking 100 samples per channel
void CalibrateZeroCurrent(void)
{
  uint32_t sampleSums[5] = {0}; // Sum of samples for each channel
  const uint16_t numSamples = 100;

  // Take 100 samples for each channel
  for(uint16_t i = 0; i < numSamples; i++) {
    // Wait for next DMA conversion to complete
    HAL_Delay(1); // Small delay to ensure new samples

    // Add current samples to sums
    for(uint8_t channel = 0; channel < 5; channel++) {
      sampleSums[channel] += ADC_VAL[channel];
    }
  }

  // Calculate averages and store as calibration values
  for(uint8_t channel = 0; channel < 5; channel++) {
    zeroCurrentCal[channel] = (uint16_t)(sampleSums[channel] / numSamples);
  }
}


//// Process the received string and convert it to a dimmer value (0-100%)
//void ProcessReceivedValue(void)
//{
//  // Null-terminate the string
//  rxBuffer[rxIndex] = '\0';
//
//  // Convert string to integer
//  uint32_t tempValue = atoi(rxBuffer);
//
//  // Limit to range (0-100)
//  if (tempValue > 100) {
//    tempValue = 100;
//  }
//
//  // Update dimmer value for all SSRs (for now, we'll set all to the same value)
//  for (uint8_t i = 0; i < 4; i++) {
//    dimmerValues[i] = (uint16_t)tempValue;
//
//    // Convert value 1 to 2 for more stable operation
//    if (dimmerValues[i] == 1) {
//      dimmerValues[i] = 2;
//    }
//
//    // Calculate dimming delay in microseconds
//    // Map dimmer value 0-100 to delay HALF_CYCLE_PERIOD_US-0 (inverted, 0% = full off, 100% = full on)
//    if (dimmerValues[i] == 0) {
//      // If dimmer is 0%, keep SSR off
//      dimmerDelaysUs[i] = HALF_CYCLE_PERIOD_US; // Set to max delay (never trigger)
//    } else if (dimmerValues[i] == 100) {
//      // If dimmer is 100%, keep SSR fully on
//      dimmerDelaysUs[i] = 0;     // No delay (trigger immediately)
//    } else {
//      // Calculate delay (0% = HALF_CYCLE_PERIOD_US delay, 100% = 0us delay)
//      // Ensure the delay is within valid range for the timer
//      dimmerDelaysUs[i] = HALF_CYCLE_PERIOD_US - ((dimmerValues[i] * HALF_CYCLE_PERIOD_US) / 100);
//
//      // Make sure we have a valid delay value (not 0 or too large)
//      if (dimmerDelaysUs[i] >= HALF_CYCLE_PERIOD_US) {
//        dimmerDelaysUs[i] = HALF_CYCLE_PERIOD_US - 100; // Leave a small margin
//      } else if (dimmerDelaysUs[i] == 0) {
//        dimmerDelaysUs[i] = 1;
//      }
//    }
//  }
//
//  // Send confirmation back to user
//  char txBuffer[100];
//  sprintf(txBuffer, "Dimmer values set to: SSR1=%u%%, SSR2=%u%%, SSR3=%u%%, SSR4=%u%%\r\n",
//          dimmerValues[0], dimmerValues[1], dimmerValues[2], dimmerValues[3]);
//  HAL_UART_Transmit(&huart1, (uint8_t*)txBuffer, strlen(txBuffer), 100);
//
//  // Reset reception variables
//  rxIndex = 0;
//  rxComplete = 0;
//  memset(rxBuffer, 0, sizeof(rxBuffer));
//}
//
//// UART Reception Complete Callback
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//  if (huart->Instance == USART1) {
//    // Check for Enter key (CR or LF)
//    if (rxData == '\r' || rxData == '\n') {
//      if (rxIndex > 0) {
//        rxComplete = 1;
//      }
//    }
//    // Check for backspace
//    else if (rxData == 127 || rxData == 8) {
//      if (rxIndex > 0) {
//        rxIndex--;
//      }
//    }
//    // Check if the character is a digit and buffer isn't full
//    else if (rxData >= '0' && rxData <= '9' && rxIndex < sizeof(rxBuffer) - 1) {
//      rxBuffer[rxIndex++] = rxData;
//
//      // Echo the character back to the terminal
//      HAL_UART_Transmit(&huart1, &rxData, 1, 10);
//    }
//
//    // Start receiving next character
//    HAL_UART_Receive_IT(&huart1, &rxData, 1);
//  }
//}

// Timer Update Interrupt Callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1) {
    // Timer 1 controls SSR1
    if(ssrControlByte & 0x01) HAL_GPIO_WritePin(SSR1_GPIO_Port, SSR1_Pin, GPIO_PIN_SET);

    // Set flag to indicate triac has been triggered in this half-cycle
    triacTriggered[0] = 1;

    // Stop the timer
    HAL_TIM_Base_Stop_IT(&htim1);
  }
}

// GPIO EXTI Callback
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // Check if this is our zero-crossing pin
  if (GPIO_Pin == GPIO_PIN_13) {
    // Reset all triac triggered flags at each zero crossing
    for (uint8_t i = 0; i < 4; i++) {
      triacTriggered[i] = 0;
    }

    // Turn off all SSRs at zero crossing
    HAL_GPIO_WritePin(SSR1_GPIO_Port, SSR1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SSR2_GPIO_Port, SSR2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SSR3_GPIO_Port, SSR3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SSR4_GPIO_Port, SSR4_Pin, GPIO_PIN_RESET);

    // Handle SSR1 with Timer1
    if (dimmerValues[0] > 0) {
      if (dimmerValues[0] < 100) {
        // Make sure timer is stopped before reconfiguring
        HAL_TIM_Base_Stop_IT(&htim1);

        // Set up Timer1 for one-shot operation with the calculated delay
        __HAL_TIM_SET_COUNTER(&htim1, 0);  // Reset counter
        __HAL_TIM_SET_AUTORELOAD(&htim1, dimmerDelaysUs[0]);  // Set delay in microseconds

        // Start the timer in interrupt mode
        HAL_TIM_Base_Start_IT(&htim1);

      } else {
        // If dimmer is 100%, keep SSR fully on
        if(ssrControlByte & 0x01) HAL_GPIO_WritePin(SSR1_GPIO_Port, SSR1_Pin, GPIO_PIN_SET);
      }
    }
    
    // Handle SSR2 with Timer2
    if (dimmerValues[1] > 0) {
      if (dimmerValues[1] < 100) {
        // Make sure timer is stopped before reconfiguring
        HAL_TIM_Base_Stop_IT(&htim2);

        // Set up Timer2 for one-shot operation with the calculated delay
        __HAL_TIM_SET_COUNTER(&htim2, 0);  // Reset counter
        __HAL_TIM_SET_AUTORELOAD(&htim2, dimmerDelaysUs[1]);  // Set delay in microseconds

        // Start the timer in interrupt mode
        HAL_TIM_Base_Start_IT(&htim2);
      } else {
        // If dimmer is 100%, keep SSR fully on
        if(ssrControlByte & 0x02) HAL_GPIO_WritePin(SSR2_GPIO_Port, SSR2_Pin, GPIO_PIN_SET);
      }
    }
    
    // Handle SSR3 with Timer4
    if (dimmerValues[2] > 0) {
      if (dimmerValues[2] < 100) {
        // Make sure timer is stopped before reconfiguring
        HAL_TIM_Base_Stop_IT(&htim4);

        // Set up Timer4 for one-shot operation with the calculated delay
        __HAL_TIM_SET_COUNTER(&htim4, 0);  // Reset counter
        __HAL_TIM_SET_AUTORELOAD(&htim4, dimmerDelaysUs[2]);  // Set delay in microseconds

        // Start the timer in interrupt mode
        HAL_TIM_Base_Start_IT(&htim4);
      } else {
        // If dimmer is 100%, keep SSR fully on
        if(ssrControlByte & 0x04) HAL_GPIO_WritePin(SSR3_GPIO_Port, SSR3_Pin, GPIO_PIN_SET);
      }
    }
    
    // Handle SSR4 with Timer5
    if (dimmerValues[3] > 0) {
      if (dimmerValues[3] < 100) {
        // Make sure timer is stopped before reconfiguring
        HAL_TIM_Base_Stop_IT(&htim5);

        // Set up Timer5 for one-shot operation with the calculated delay
        __HAL_TIM_SET_COUNTER(&htim5, 0);  // Reset counter
        __HAL_TIM_SET_AUTORELOAD(&htim5, dimmerDelaysUs[3]);  // Set delay in microseconds

        // Start the timer in interrupt mode
        HAL_TIM_Base_Start_IT(&htim5);
      } else {
        // If dimmer is 100%, keep SSR fully on
        if(ssrControlByte & 0x08) HAL_GPIO_WritePin(SSR4_GPIO_Port, SSR4_Pin, GPIO_PIN_SET);
      }
    }
  }
}
int32_t current_mA[4];

// Function to set individual SSR state (1-4)
void SetSSRState(uint8_t ssrNum, uint8_t state)
{
    if(ssrNum < 1 || ssrNum > 4) return; // Invalid SSR number

    uint8_t bitMask = 1 << (ssrNum - 1); // Calculate bit position (0-3)

    if(state) {
        ssrControlByte |= bitMask;  // Set bit to enable SSR
    } else {
        ssrControlByte &= ~bitMask; // Clear bit to disable SSR
    }
}

// Function to set individual SSR dimmer value (0-100%)
void SetSSRDimmerValue(uint8_t ssrNum, uint16_t value)
{
    if(ssrNum < 1 || ssrNum > 4) return; // Invalid SSR number

    // Limit to range (0-100)
    if (value > 100) {
        value = 100;
    }

    // Convert value 1 to 2 for more stable operation
    if (value == 1) {
        value = 2;
    }

    // Update dimmer value for the specified SSR
    dimmerValues[ssrNum - 1] = value;

    // Calculate dimming delay in microseconds
    if (value == 0) {
        // If dimmer is 0%, keep SSR off
        dimmerDelaysUs[ssrNum - 1] = HALF_CYCLE_PERIOD_US; // Set to max delay (never trigger)
    } else if (value == 100) {
        // If dimmer is 100%, keep SSR fully on
        dimmerDelaysUs[ssrNum - 1] = 0;     // No delay (trigger immediately)
    } else {
        // Calculate delay (0% = HALF_CYCLE_PERIOD_US delay, 100% = 0us delay)
        dimmerDelaysUs[ssrNum - 1] = HALF_CYCLE_PERIOD_US - ((value * HALF_CYCLE_PERIOD_US) / 100);

        // Make sure we have a valid delay value (not 0 or too large)
        if (dimmerDelaysUs[ssrNum - 1] >= HALF_CYCLE_PERIOD_US) {
            dimmerDelaysUs[ssrNum - 1] = HALF_CYCLE_PERIOD_US - 100; // Leave a small margin
        } else if (dimmerDelaysUs[ssrNum - 1] == 0) {
            dimmerDelaysUs[ssrNum - 1] = 1;
        }
    }
}

void GetCurrentReadings()
{
    // Take 100 samples and average them
    #define NUM_SAMPLES 10
    uint32_t adc_sum[4] = {0};

    for(int i = 0; i < NUM_SAMPLES; i++)
    {
      HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_VAL, 4);
      //HAL_Delay(1); // Small delay between samples
      for(int ch = 0; ch < 4; ch++)
      {
        adc_sum[ch] += ADC_VAL[ch];
      }
    }

    // Calculate average ADC values
    int32_t adc_avg[4];
    for(int ch = 0; ch < 4; ch++)
    {
      adc_avg[ch] = adc_sum[ch] / NUM_SAMPLES;
    }

    // Combined conversion factor: (3300 * 1000 * 0.707) / (4095 * 132) = 4.3162
    const float CURRENT_CONVERSION_FACTOR = 4.3162f;

    // Convert averaged ADC values to current (mA)
    for(int ch = 0; ch < 4; ch++)
    {
      int32_t value = (adc_avg[ch] - zeroCurrentCal[ch]) * CURRENT_CONVERSION_FACTOR;
       current_mA[ch] = (value < 0) ? 0 : value;
//      current_mA[ch] = value;
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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_I2C1_Init();
    /* USER CODE BEGIN 2 */

  read_modbus_address(); // Read slave ID from GPIO pins

  HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, 256);
  sent_data[2]=1;
  //HAL_I2C_EnableListen_IT(&hi2c1);
  // Start ADC with DMA
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_VAL, 5);

  // Wait a short time for ADC to stabilize
  HAL_Delay(1);

  // Perform zero current calibration
  CalibrateZeroCurrent();

  // Enable EXTI Line 13 interrupt for zero-crossing detection
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  // Initialize SSR pin to OFF state
  HAL_GPIO_WritePin(SSR1_GPIO_Port, SSR1_Pin, GPIO_PIN_RESET);

  // Initialize Timer1 for microsecond timing
  // Stop any running timer operations first
  HAL_TIM_Base_Stop(&htim1);
  HAL_TIM_Base_Stop_IT(&htim1);

  // Enable Timer1 interrupt with lower priority
  HAL_NVIC_SetPriority(TIM1_UP_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);

  // Initialize Timer2 for microsecond timing
  // Stop any running timer operations first
  HAL_TIM_Base_Stop(&htim2);
  HAL_TIM_Base_Stop_IT(&htim2);

  // Enable Timer2 interrupt with lower priority
  HAL_NVIC_SetPriority(TIM2_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);

  // Initialize Timer4 for microsecond timing (SSR3)
  // Stop any running timer operations first
  HAL_TIM_Base_Stop(&htim4);
  HAL_TIM_Base_Stop_IT(&htim4);

  // Enable Timer4 interrupt
  HAL_NVIC_SetPriority(TIM4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);

  // Initialize Timer5 for microsecond timing (SSR4)
  // Stop any running timer operations first
  HAL_TIM_Base_Stop(&htim5);
  HAL_TIM_Base_Stop_IT(&htim5);

  // Enable Timer5 interrupt
  HAL_NVIC_SetPriority(TIM5_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(TIM5_IRQn);

//  // Start UART reception in interrupt mode
//  HAL_UART_Receive_IT(&huart1, &rxData, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //uint32_t lastPrintTime = 0;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // Inputs Reading
//		uint8_t left_switch = HAL_GPIO_ReadPin(INPUT4_GPIO_Port, INPUT4_Pin);
//		uint8_t right_switch = HAL_GPIO_ReadPin(INPUT3_GPIO_Port, INPUT3_Pin);
//		uint8_t home_switch = HAL_GPIO_ReadPin(INPUT2_GPIO_Port, INPUT2_Pin);

	    GetCurrentReadings();
//		uint16_t pressure_sensor_val = (uint16_t)ADC_VAL[4];
//
//		uint16_t adcVal1 = (uint16_t)current_mA[0];
//		uint16_t adcVal2 = (uint16_t)current_mA[1];
//		uint16_t adcVal3 = (uint16_t)current_mA[2];
//		uint16_t adcVal4 = (uint16_t)current_mA[3];

//		sent_data[0] = right_switch;
//		sent_data[1] = left_switch;
//		sent_data[2] = home_switch;
//
//		// Store current1 values
//		sent_data[3] = (uint8_t)(adcVal1 >> 8);
//		sent_data[4] = (uint8_t)(adcVal1 & 0xFF);
//
//		// Store current2 values
//		sent_data[5] = (uint8_t)(adcVal2 >> 8);
//		sent_data[6] = (uint8_t)(adcVal2 & 0xFF);
//
//		// Store current3 values
//		sent_data[7] = (uint8_t)(adcVal3 >> 8);
//		sent_data[8] = (uint8_t)(adcVal3 & 0xFF);
//
//		// Store current4 values
//		sent_data[9] = (uint8_t)(adcVal4 >> 8);
//		sent_data[10] = (uint8_t)(adcVal4 & 0xFF);
//
//		// Pressure Sensor values
//		sent_data[11] = (uint8_t)(pressure_sensor_val >> 8);
//		sent_data[12] = (uint8_t)(pressure_sensor_val & 0xFF);


//	  uint8_t _ssr1_state = received_data [0];
//	  uint8_t _ssr2_state = received_data [1];
//	  uint8_t _ssr3_state = received_data [2];
//	  uint8_t _ssr4_state = received_data [3];
//	  uint8_t _cooling_valve = received_data [4];
//	  uint8_t _led_color = received_data[8];
//	  uint8_t _ssr1_dimmer = received_data[9];
//	  uint8_t _ssr2_dimmer = received_data[10];
//	  uint8_t _ssr3_dimmer = received_data[11];
//	  uint8_t _ssr4_dimmer = received_data[12];


	  uint8_t _ssr1_dimmer = Holding_Registers_Database[0]; //40001
	  uint8_t _ssr2_dimmer = Holding_Registers_Database[1]; //40002
	  uint8_t _ssr3_dimmer = Holding_Registers_Database[2]; //40003
	  uint8_t _ssr4_dimmer = Holding_Registers_Database[3]; //40004

	  uint8_t _ssr1_state = Coils_Database [0] & 0x1; // 	1
	  uint8_t _ssr2_state = Coils_Database [0] & 0x2; //	2
	  uint8_t _ssr3_state = Coils_Database [0] & 0x4; //	3
	  uint8_t _ssr4_state = Coils_Database [0] & 0x8; //	4

	  encoders[0].Reset = Coils_Database [0] & 0x10;  //	5
	  encoders[0].Reset = Coils_Database [0] & 0x20;  //	6
	  encoders[0].Reset = Coils_Database [0] & 0x40;  //	7
	  encoders[0].Reset = Coils_Database [0] & 0x80;  //	8

	  uint8_t ssr_states[] = {_ssr1_state, _ssr2_state, _ssr3_state, _ssr4_state};
	  float current_mA_float[4];
	  for (int i = 0; i < 4; i++) {
		  current_mA_float[i] = (float)current_mA[i];
	  }
	  get_energy_values(current_mA_float, ssr_states);

//	  uint8_t _cooling_valve = received_data [4];
//	  uint8_t _led_color = received_data[8];

    
	  SetSSRDimmerValue(1,_ssr1_dimmer);
	  SetSSRDimmerValue(2,_ssr2_dimmer);
	  SetSSRDimmerValue(3,_ssr3_dimmer);
	  SetSSRDimmerValue(4,_ssr4_dimmer);
	  SetSSRState(1,_ssr1_state);
	  SetSSRState(2,_ssr2_state);
	  SetSSRState(3,_ssr3_state);
	  SetSSRState(4,_ssr4_state);

	  encoder_sync(encoders,TOTAL_ENCODERS);

	  Input_Registers_Database[0] = (uint16_t)encoders[0].Distance*100;
	  Input_Registers_Database[1] = (uint16_t)encoders[1].Distance*100;
	  Input_Registers_Database[2] = (uint16_t)encoders[2].Distance*100;
	  Input_Registers_Database[3] = (uint16_t)encoders[3].Distance*100;

	  read_modbus_address(); // Read slave ID from GPIO pins

//	  set_led_color(_led_color);
//	  HAL_GPIO_WritePin(OUTPUT1_GPIO_Port, OUTPUT1_Pin, _cooling_valve);
	  HAL_Delay(20);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV5;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_PLL2;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL2_ON;
  RCC_OscInitStruct.PLL2.PLL2MUL = RCC_PLL2_MUL8;
  RCC_OscInitStruct.PLL2.HSEPrediv2Value = RCC_HSE_PREDIV2_DIV5;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the Systick interrupt time
  */
  __HAL_RCC_PLLI2S_ENABLE();
}

/* USER CODE BEGIN 4 */
void get_energy_values(float* current_mA, uint8_t ssr_states[])
{
	// Calculate time delta since last call
	uint32_t current_tick = HAL_GetTick();
	// Initialize last_energy_tick on the first run
	if (last_energy_tick == 0)
	{
		last_energy_tick = current_tick;
		return;
	}
	float time_delta_seconds = (float)(current_tick - last_energy_tick) / 1000.0f;
	last_energy_tick = current_tick;

	// Loop through each of the 4 SSR channels
	for (int i = 0; i < 4; i++)
	{
		// Only accumulate energy if the corresponding SSR is ON
		if (ssr_states[i])
		{
			// Convert current from mA to A
			float current_A = current_mA[i] / 1000.0f;
			// Calculate power in Watts (P = V * I)
			float power_W = LINE_VOLTAGE * current_A;
			// Calculate energy delta in Joules (E = P * t)
			float energy_delta_J = power_W * time_delta_seconds;
			// Add to the cumulative energy for the channel
			energy_joules[i] += energy_delta_J;
		}
		// Store the integer part of the cumulative energy in the holding register.
		// Note: This will overflow if energy exceeds 65535 Joules.
		// A 32-bit value using two registers would be better for long-term accumulation.
		Holding_Registers_Database[4 + i] = (uint16_t)energy_joules[i];
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
