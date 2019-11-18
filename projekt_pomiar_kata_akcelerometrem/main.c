/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

#include <limits.h> // stale definiujace wartosci typów zmiennych
#include <math.h> // funkcje matematyczne
#define M_PI 3.14159265358979323846 // stala wykorzystywana przy obliczaniu rotacji 

const float PRZYSPIESZENIE_ZIEMSKIE = 9.80665; // m/(s^2) stala wykorzystywana przy normalizacji pomiaru
const float ROZDZIELCZOSC_AKCELEROMETRU =  2.0; // +/- 2g  stala odpowiadajaca przyjetej rozdzielczosci w akcelerometrze
#define MMA8451Q_USTAWIENIA 0x19 // wlaczamy ODR na 100 Hz i wlaczamy pomiar danych 

#define MMA8451Q_ADRESS (0x1C<<1) // adres urzadzenia
#define MMA8451Q_CTRL_REG1 0x2A // [ASLP_RATE1][ASLP_RATE0][DR2][DR1][DR0][LNOISE][F_READ][ACTIVE]

#define MMA8451Q_OUT_X_MSB 0x01
#define MMA8451Q_OUT_X_LSB 0x02
#define MMA8451Q_OUT_Y_MSB 0x03
#define MMA8451Q_OUT_Y_LSB 0x04
#define MMA8451Q_OUT_Z_MSB 0x05
#define MMA8451Q_OUT_Z_LSB 0x06

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t tab_osie[6];

int16_t os_x = 0;
int16_t os_y = 0;
int16_t os_z = 0;

float os_x_g = 0;
float os_y_g = 0;
float os_z_g = 0;

float roll = 0;
float pitch = 0;

uint8_t tab[64];
uint16_t rozmiar_tab = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	// wyslanie informacji przez uart
	rozmiar_tab = sprintf(tab, "os_x: %d, os_y: %d, os_z: %d, roll: %d, pitch: %d \n\r", (int)os_x_g, (int)os_y_g, (int)os_z_g, (int)roll, (int)pitch);
	HAL_UART_Transmit_IT(&huart1, tab, rozmiar_tab);
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM16_Init();

  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim16);
	// ustawienie rejestrów akcelerometru
	uint8_t ustawienia = MMA8451Q_USTAWIENIA;
	HAL_I2C_Mem_Write(&hi2c1, MMA8451Q_ADRESS, MMA8451Q_CTRL_REG1, 1, &ustawienia, 1, 100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	// odczyt danych z akcelerometru
	HAL_I2C_Mem_Read(&hi2c1, MMA8451Q_ADRESS, MMA8451Q_OUT_X_MSB , 1, tab_osie, 6, 100);
	
	// wartosci surowe 
	os_x = ( (tab_osie[0] << 8) | tab_osie[1] );
	os_y = ( (tab_osie[2] << 8) | tab_osie[3] );
	os_z = ( (tab_osie[4] << 8) | tab_osie[5] );
		
	// skalowanie do ustawionej rozdzielczosci
	os_x_g = ((float)os_x * ROZDZIELCZOSC_AKCELEROMETRU)/(float)INT16_MAX;
	os_y_g = ((float)os_y * ROZDZIELCZOSC_AKCELEROMETRU)/(float)INT16_MAX;
	os_z_g = ((float)os_z * ROZDZIELCZOSC_AKCELEROMETRU)/(float)INT16_MAX;
		
	// normalizowanie
	os_x_g = os_x_g * PRZYSPIESZENIE_ZIEMSKIE; 
	os_y_g = os_y_g * PRZYSPIESZENIE_ZIEMSKIE;
	os_z_g = os_z_g * PRZYSPIESZENIE_ZIEMSKIE;
	
	// przeliczanie katów
	pitch = -(atan2(os_x_g,sqrt(os_y_g*os_y_g + os_z_g*os_z_g)) *180.0) /M_PI;
	roll = (atan2(os_y_g,os_z_g) *180.0) /M_PI;
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
