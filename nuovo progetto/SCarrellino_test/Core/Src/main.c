/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "can.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "i2c-lcd.h"
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

/* USER CODE BEGIN PV */
#define lcd_address 0x27<<1 
uint8_t comando_h ;   // 4 MSB
  uint8_t comando_l ; // 4 LSB
  uint8_t maschera_h_en1 ; // parte alta del comando con enable 1 in modalità scrittura comando
  uint8_t maschera_h_en0; // parte alta del comando con enable 0 in modalità scrittura comando
  uint8_t maschera_l_en1; // parte bassa del comando con enable 1 in modalità scrittura comando
  uint8_t maschera_l_en0 ; // parte b

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void my_lcd_cmd(uint8_t cmd){
   comando_h = cmd&0xF0;   // 4 MSB
   comando_l = (cmd&0x0F)<<4; // 4 LSB
   maschera_h_en1 = comando_h | 0b1100; // parte alta del comando con enable 1 in modalità scrittura comando
   maschera_h_en0 = comando_h | 0b1000; // parte alta del comando con enable 0 in modalità scrittura comando
   maschera_l_en1 = comando_l | 0b1100; // parte bassa del comando con enable 1 in modalità scrittura comando
   maschera_l_en0 = comando_l | 0b1000; // parte bassa del comando con enable 0 in modalità scrittura comando


  HAL_I2C_Master_Transmit(&hi2c1,lcd_address, (uint8_t *)maschera_h_en1, sizeof(maschera_h_en1), HAL_MAX_DELAY);
  HAL_Delay(1);
  HAL_I2C_Master_Transmit(&hi2c1,lcd_address, (uint8_t *)maschera_h_en0, sizeof(maschera_h_en0), HAL_MAX_DELAY);
  HAL_Delay(2);

  HAL_I2C_Master_Transmit(&hi2c1,lcd_address, (uint8_t *)maschera_l_en1, sizeof(maschera_l_en1), HAL_MAX_DELAY);
  HAL_Delay(1);
  HAL_I2C_Master_Transmit(&hi2c1,lcd_address, (uint8_t *)maschera_l_en0, sizeof(maschera_l_en0), HAL_MAX_DELAY);
  HAL_Delay(5);
}

void my_lcd_init(void){

  HAL_Delay(20);
  my_lcd_cmd (0x02);	/* 4bit mode */
	my_lcd_cmd (0x28);	/* Initialization of 16X2 LCD in 4bit mode */
	my_lcd_cmd (0x0C);	/* Display ON Cursor OFF */
	my_lcd_cmd (0x06);	/* Auto Increment cursor */
	my_lcd_cmd (0x01);	/* Clear display */
	my_lcd_cmd (0x80);	/* Cursor at home position */

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
  //HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */


  if(HAL_I2C_IsDeviceReady(&hi2c1, lcd_address, 100,HAL_MAX_DELAY) != HAL_OK){
  
  Error_Handler();
   }
   else{HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
   }



  //lcd_init();
  
//0111111111111 con 0b11111111   0b11111111110
//0101111111110 con 0b11111110
//0111111111100 con 0b11111100
//0111111111000 con 0b11111000
//0100001111000 con {0b11111110, 0b000}

//clear schermo
 // uint8_t dati = 0b1000 ;
  //HAL_I2C_Master_Transmit(&hi2c1,lcd_address, (uint8_t *)dati, sizeof(dati), HAL_MAX_DELAY);
 //uint8_t buffer_prova = 0b000;
  //HAL_I2C_Master_Transmit(&hi2c1,lcd_address, &buffer_prova, sizeof(buffer_prova), HAL_MAX_DELAY);

my_lcd_init();
//HAL_I2C_Master_Transmit(&hi2c1,lcd_address, (uint8_t *)0b1100, sizeof(0b1100), HAL_MAX_DELAY);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
