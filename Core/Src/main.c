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

/**
 * USING MCP2515 LIBRARY FROM HERE: https://www.micropeta.com/video138
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CANSPI.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  int16_t angle;
  int16_t velocity;
  int16_t current;
  uint8_t temp;

  uint16_t encoder;
  uint16_t encoderRaw;
  int16_t encoderOffset;
} Motor;


typedef enum {
  MOTOR_BOTH_OFF, //0
  MOTOR_BOTH_RANDOM, //1
  MOTOR_1_CONSTANT, //2 // for the pig on a stick
  MOTOR_2_CONSTANT //3
} MotorProfile;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

#define RX_BUFF_SIZE 500
#define MAX_VELO_RPM 15
char state = 'i';

uint32_t lastPrintTime = 0;
uint32_t printInterval = 200;

uCAN_MSG rxMessage;

uint8_t rx_buff[RX_BUFF_SIZE] = {0};
uint8_t rx_buff_arm = 0;

static char msg[64];

MotorProfile currentProfile = MOTOR_BOTH_RANDOM;

Motor motorA;
Motor motorB;

Motor motors[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void setVelocity(uint16_t canID, int32_t velocity_dps_hundreth);
void sendCommand(uint16_t canID, uint8_t command);
void resetMotor(uint16_t canID);
void getFeedback();
void setMotorProfile(MotorProfile profile);

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
	motors[0] = motorA;
	motors[1] = motorB;
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, rx_buff, 1);

  CANSPI_Initialize();
  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // if we receive data from the motor, parse it into the motor struct
	  if (CANSPI_Receive(&rxMessage)) {
		  getFeedback();
	  }

	  // 'a is the only state when motors are running
	  if (state == 'a'){
		  setVelocity(0x141, MAX_VELO_RPM * 6 * 100);
		  setVelocity(0x142, MAX_VELO_RPM * 6 * 100);
	  } else {
		  resetMotor(0x141);
		  resetMotor(0x142);
	  }

	  // timed print
	  if (HAL_GetTick() - lastPrintTime >= printInterval) {
		  lastPrintTime = HAL_GetTick();
		  memset(msg, 0, sizeof(msg));  // clear garbage from buffer
		  int len = snprintf(msg, sizeof(msg),
				  "%d %d %d %d %d %d\n",
				  motors[0].angle, motors[0].velocity, motors[0].encoder,
				  motors[1].angle, motors[1].velocity, motors[1].encoder);

		  // Send over UART using interrupt
		  HAL_UART_Transmit_IT(&huart1, (uint8_t*)msg, len);
	  }
	  /* USER CODE END WHILE */

	  /* USER CODE BEGIN 3 */
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

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	char recentChar = *(rx_buff + rx_buff_arm); // the most recent character that has not been read yet

	/** fsm for the RPM:
		-------------------------------
		 i | idle (choose profile)
		 a | begin profile
	 **/
	switch (state){
	case 'i':
		resetMotor(0x141);
		resetMotor(0x142);
		if (recentChar == 'a') {
			state = 'a';
		}
		break;

	case 'a':
		if (recentChar == 'r') {
			resetMotor(0x141);
			resetMotor(0x142);
			state = 'i';
		}
	}

	// move to next index in the circular buffer to be read
	rx_buff_arm ++;

	// make sure we stay within the buffer
	if(rx_buff_arm >= RX_BUFF_SIZE){
		rx_buff_arm = 0;
	}

	// flush our command history through UART
	HAL_UART_Transmit_IT(&huart1, rx_buff, RX_BUFF_SIZE);

	uint8_t char_arr[1] = {recentChar};
	HAL_UART_Transmit_IT(&huart1, char_arr, 1);

	// necessary to prime the next callback
	HAL_UART_Receive_IT(&huart1, rx_buff + rx_buff_arm, 1); // the next character will be stored in the next index
}

void setVelocity(uint16_t canID, int32_t velocity_dps_hundreth){
	uCAN_MSG txMessage;

	// sending data to motor
	txMessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
	txMessage.frame.id = canID; // ID can be between Hex1 and Hex7FF (1-2047 decimal)
	txMessage.frame.dlc = 8;

	txMessage.frame.data0 = 0xA2;
	txMessage.frame.data1 = 0x00;
	txMessage.frame.data2 = 0x00;
	txMessage.frame.data3 = 0x00;
	txMessage.frame.data4 = (uint8_t)(velocity_dps_hundreth);
	txMessage.frame.data5 = (uint8_t)(velocity_dps_hundreth>>8);
	txMessage.frame.data6 = (uint8_t)(velocity_dps_hundreth>>16);
	txMessage.frame.data7 = (uint8_t)(velocity_dps_hundreth>>24);

	CANSPI_Transmit(&txMessage);
}


void sendCommand(uint16_t canID, uint8_t command){
	uCAN_MSG txMessage;

	// sending data to motor
	txMessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
	txMessage.frame.id = canID; // ID can be between Hex1 and Hex7FF (1-2047 decimal)
	txMessage.frame.dlc = 8;

	txMessage.frame.data0 = command;
	txMessage.frame.data1 = 0x00;
	txMessage.frame.data2 = 0x00;
	txMessage.frame.data3 = 0x00;
	txMessage.frame.data4 = 0x00;
	txMessage.frame.data5 = 0x00;
	txMessage.frame.data6 = 0x00;
	txMessage.frame.data7 = 0x00;

	CANSPI_Transmit(&txMessage);
}


void resetMotor(uint16_t canID){
  sendCommand(canID, 0x76);
}

void getFeedback(){
	uint8_t data[8] = {rxMessage.frame.data0, rxMessage.frame.data1, rxMessage.frame.data2,
					   rxMessage.frame.data3, rxMessage.frame.data4, rxMessage.frame.data5,
					   rxMessage.frame.data6, rxMessage.frame.data7};

	// first motor
	if (rxMessage.frame.id == 0x141 && rxMessage.frame.data0 == 0xA2) {
		motors[0].angle = (int16_t)(data[6] | (data[7]<<8));
		motors[0].velocity = (int16_t)(data[4] | (data[5]<<8));
		motors[0].current = (int16_t)(data[2] | (data[3]<<8));
		motors[0].temp = data[1];
	}else if(rxMessage.frame.id == 0x141 && data[0] == 0x90){
        motors[0].encoder = (int16_t)(data[2] | (data[3]<<8));
        motors[0].encoderRaw = (int16_t)(data[4] | (data[5]<<8));
        motors[0].encoderOffset = (int16_t)(data[6] | (data[7]<<8));
    }

	// second motor
	if (rxMessage.frame.id == 0x142 && rxMessage.frame.data0 == 0xA2) {
		motors[1].angle = (int16_t)(data[6] | (data[7]<<8));
		motors[1].velocity = (int16_t)(data[4] | (data[5]<<8));
		motors[1].current = (int16_t)(data[2] | (data[3]<<8));
		motors[1].temp = data[1];
	}else if(rxMessage.frame.id == 0x142 && data[0] == 0x90){
        motors[1].encoder = (int16_t)(data[2] | (data[3]<<8));
        motors[1].encoderRaw = (int16_t)(data[4] | (data[5]<<8));
        motors[1].encoderOffset = (int16_t)(data[6] | (data[7]<<8));
    }
}

void setMotorProfile(MotorProfile profile) {
  currentProfile = profile;
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
