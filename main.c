/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd16x2.h" // Including LCD Library
#include "stdio.h"
#include <math.h>
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define adxl_address 0x53<<1 //SDO=1 else (0x53) if SDO = GND
#define POWER_CTL 0x2D
#define DATA_FORMAT 0x31
#define DATA_X0 0x32

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

uint8_t data_rec[6];
uint8_t chipid=0;
int16_t x,y,z;
float xg, yg, zg;
float roll, pitch, yaw;
char x_char[512], y_char[512], z_char[512];




// ******** Write Function ********
void adxl_write(uint8_t reg, uint8_t value)
{
	uint8_t data[2];
	data[0] = reg;
	data[1] = value;
	HAL_I2C_Master_Transmit(&hi2c1, adxl_address, data, 2, 10);
}

// ******** Read Function ********
void adxl_read(uint8_t reg, uint8_t numberofbytes)
{
	HAL_I2C_Mem_Read(&hi2c1, adxl_address, reg, 1, data_rec, numberofbytes, 100);
}

// ******** ADXL345 Initalization ********
void adxl_init()
{
	adxl_read(0x00,1);

	adxl_write(POWER_CTL,0); // Reset All Bits
	adxl_write(POWER_CTL,0x10);
	adxl_write(POWER_CTL,0x08);

	adxl_write(DATA_FORMAT,0x01); // ± 4g Resolution

	/// ADXL X,Y,Z OFFSET
	adxl_write(0x1E,-3);
	adxl_write(0x1F,-1);
	adxl_write(0x20,-11);

}

// ******** Getting Data Interface ********
void getting_data()
{

	HAL_Delay(100);
	lcd16x2_1stLine();
	HAL_GPIO_WritePin(GPIOB, LED_R_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, LED_G_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, LED_B_Pin, GPIO_PIN_SET);
	lcd16x2_printf("Getting Data");
	HAL_Delay(150);
	lcd16x2_setCursor(0,12);
	lcd16x2_printf(".");
	HAL_Delay(500);
	lcd16x2_setCursor(0,13);
	lcd16x2_printf(".");
	HAL_Delay(500);
	lcd16x2_setCursor(0,14);
	lcd16x2_printf(".");
	HAL_Delay(500);
	lcd16x2_clear();
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIOB, LED_R_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, LED_G_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, LED_B_Pin, GPIO_PIN_RESET);
}


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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
  adxl_init();

      lcd16x2_init_4bits(RS_GPIO_Port, RS_Pin, EN_Pin,                     // Initializing LCD
    		               D4_GPIO_Port, D4_Pin, D5_Pin, D6_Pin, D7_Pin);
      lcd16x2_1stLine();                                                   // Begin 1st Line of the LCD
      lcd16x2_printf("  Inclination  ");								   // Printing
      lcd16x2_2ndLine();												   // Begin 2nd Line of the LCD
      lcd16x2_printf("   Indicator   ");								       // Printing
      lcd16x2_cursorShow(0);											   // Cursor OFF
      HAL_Delay(3000);
      lcd16x2_clear();												       // Clears LCD

      HAL_UART_Transmit(&huart2,(uint8_t*)"STM32F401RE ADXL345\n\r", 20,1000);
      HAL_UART_Transmit(&huart2,(uint8_t*)"Range ±4g\n\r", 10,1000);
      HAL_UART_Transmit(&huart2,(uint8_t*)"Baud Rate 9600\n\r", 15,1000);
      HAL_UART_Transmit(&huart2,(uint8_t*)"=========================\n\r", 25,1000);
      HAL_UART_Transmit(&huart2,(uint8_t*)"\n\n\n\r", 5,100);
      HAL_Delay(500);

      getting_data();


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  adxl_read(DATA_X0,6);

	  	  x = (data_rec[1]<<8) | data_rec[0];
	  	  y = (data_rec[3]<<8) | data_rec[2];
	  	  z = (data_rec[5]<<8) | data_rec[4];

	  	  // **** Convert x,y,z values to "g"s ****
	  	  xg = x*0.0078;
	  	  yg = y*0.0078;
	  	  zg = z*0.0078;


	  	  pitch = 180 * atan(xg/sqrt(yg*yg + zg*zg))/M_PI;
	  	  roll = 180 * atan(yg/sqrt(xg*xg + zg*zg))/M_PI;


	  	 // HAL_UART_Transmit(&huart2,(uint8_t*)"\n\r", 1,100);

	  	  sprintf(x_char,"RAW Data : X: %d   Y: %d   Z: %d\n\r",x, y, z);
	  	  //HAL_UART_Transmit(&huart2, (uint8_t*)x_char, strlen(x_char), 100);
	  	  sprintf(x_char, "Acceleration X: %.4fg Y: %.4fg Z: %.4fg\n\r",xg, yg, zg);
	  	  //HAL_UART_Transmit(&huart2, (uint8_t*)x_char, strlen(x_char), 100);
	  	  sprintf(x_char, "Roll: %.4f ° Pitch: %.4f ° \n\n\r",roll, pitch);
	  	  //HAL_UART_Transmit(&huart2, (uint8_t*)x_char, strlen(x_char), 100);

	  	  //HAL_UART_Transmit(&huart2,(uint8_t*)"===================================\n\r", 35,1000);
	  	  //HAL_UART_Transmit(&huart2,(uint8_t*)"\n\r", 1,100);
	  	  HAL_Delay(500);




	  	  lcd16x2_1stLine();
	  	  lcd16x2_printf(" Roll:%.1f ",roll);
	  	  lcd16x2_2ndLine();
	  	  lcd16x2_printf("Pitch:%.1f ",pitch);
	  	  lcd16x2_setCursor(0,12);



	  	  if(roll>3 ){
	  		  lcd16x2_setCursor(0,12);
	  		  lcd16x2_printf("-->");


	  		  // PURPLE LED
	  		  HAL_GPIO_WritePin(GPIOB, LED_R_Pin, GPIO_PIN_SET);
	  		  HAL_GPIO_WritePin(GPIOB, LED_G_Pin, GPIO_PIN_RESET);
	  		  HAL_GPIO_WritePin(GPIOB, LED_B_Pin, GPIO_PIN_SET);
	  		  HAL_Delay(500);

	  	  }
	  	  if(roll < -3){
	  		  lcd16x2_setCursor(0,12);
	  		  lcd16x2_printf("<--");
	  		  // YELLOW LED
	  		  HAL_GPIO_WritePin(GPIOB, LED_R_Pin, GPIO_PIN_SET);
	  		  HAL_GPIO_WritePin(GPIOB, LED_G_Pin, GPIO_PIN_SET);
	  		  HAL_GPIO_WritePin(GPIOB, LED_B_Pin, GPIO_PIN_RESET);
	  		  HAL_Delay(500);
	  	  }
	  	  if(roll < 3 && roll > -3){
	  		  lcd16x2_setCursor(0,12);
	  		  lcd16x2_printf("==  ");
	  		  // No Color
	  		  HAL_GPIO_WritePin(GPIOB, LED_R_Pin, GPIO_PIN_RESET);
	  		  HAL_GPIO_WritePin(GPIOB, LED_G_Pin, GPIO_PIN_RESET);
	  		  HAL_GPIO_WritePin(GPIOB, LED_B_Pin, GPIO_PIN_RESET);
	  	  }
	  	  if(pitch>3 ){
	  	  		  lcd16x2_setCursor(1,12);
	  	  		  lcd16x2_printf("up  ");
	  	  		  // GREEN LED
	  	  		  HAL_GPIO_WritePin(GPIOB, LED_R_Pin, GPIO_PIN_RESET);
	  	  		  HAL_GPIO_WritePin(GPIOB, LED_G_Pin, GPIO_PIN_SET);
	  	  		  HAL_GPIO_WritePin(GPIOB, LED_B_Pin, GPIO_PIN_RESET);
	  	  }
	  	  if(pitch < -3){
	  	  		  lcd16x2_setCursor(1,12);
	  	  		  lcd16x2_printf("down");
	  	  		  // RED LED
	  	  		  HAL_GPIO_WritePin(GPIOB, LED_R_Pin, GPIO_PIN_SET);
	  	  		  HAL_GPIO_WritePin(GPIOB, LED_G_Pin, GPIO_PIN_RESET);
	  	  		  HAL_GPIO_WritePin(GPIOB, LED_B_Pin, GPIO_PIN_RESET);

	  	  }
	  	  if(pitch < 3 && pitch > -3){
	  	  		  lcd16x2_setCursor(1,12);
	  	  		  lcd16x2_printf("==  ");
	  	  		  // No Color
	  	  		  HAL_GPIO_WritePin(GPIOB, LED_R_Pin, GPIO_PIN_RESET);
	  	  		  HAL_GPIO_WritePin(GPIOB, LED_G_Pin, GPIO_PIN_RESET);
	  	    	  HAL_GPIO_WritePin(GPIOB, LED_B_Pin, GPIO_PIN_RESET);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|RS_Pin|EN_Pin|D7_Pin
                          |D6_Pin|D5_Pin|D4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_B_Pin|LED_G_Pin|LED_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin RS_Pin EN_Pin D7_Pin
                           D6_Pin D5_Pin D4_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|RS_Pin|EN_Pin|D7_Pin
                          |D6_Pin|D5_Pin|D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_B_Pin LED_G_Pin LED_R_Pin */
  GPIO_InitStruct.Pin = LED_B_Pin|LED_G_Pin|LED_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
