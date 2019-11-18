#include "stm32f3xx_hal.h"
#include "spi.h"
#include "usb_device.h"
#include "gpio.h"
#include "usbd_cdc_if.h"
#include "l3gd20.h"

void SystemClock_Config(void);

extern uint8_t UserRxBufferFS[4];

int main(void){
		uint8_t  zapytaj_o_zmienna      = 0x0;
		uint8_t  wprowadz_zmienna       = 0x0;
		uint8_t  i_know_who_i_am        = 0x0;
		
		int16_t  g_x 									= 0x0;
		int16_t  g_y 									= 0x0;
		int16_t  g_z 									= 0x0;
		
		uint8_t  g_t 									  = 0x0;
	
		uint8_t  tab_xyz[6]							= {0x0,0x0,0x0,0x0,0x0,0x0};
		
		int16_t odchylenie_x = 0x0;
		int16_t odchylenie_y = 0x0;
		int16_t odchylenie_z = 0x0;
	
		char  	 tab_char[30]    = "\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0";
		uint8_t  tab_uint8_t[30] = "\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0";
		unsigned i;
	
/* MCU Configuration----------------------------------------------------------*/
/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
HAL_Init();
/* Configure the system clock */
SystemClock_Config();
/* Initialize all configured peripherals */
MX_GPIO_Init(); MX_SPI1_Init(); MX_USB_DEVICE_Init();
/* MCU Configuration----------------------------------------------------------*/
  
		/// Who am i
		zapytaj_o_zmienna =  L3GD20_WHO_AM_I | L3GD20_READ_BIT;
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1,&zapytaj_o_zmienna,sizeof(zapytaj_o_zmienna),1);
			HAL_SPI_Receive(&hspi1,&i_know_who_i_am  ,sizeof(i_know_who_i_am),1);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
	
		HAL_Delay(1);
	
		/// REG1
		zapytaj_o_zmienna =  L3GD20_CTRL_REG1;
		wprowadz_zmienna = L3GD20_CTRL_REG1_IN;
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1,&zapytaj_o_zmienna,sizeof(wprowadz_zmienna),1);
			HAL_SPI_Transmit(&hspi1,&wprowadz_zmienna,sizeof(wprowadz_zmienna),1);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);

		HAL_Delay(1);
		
		/// REG4
		zapytaj_o_zmienna =  L3GD20_CTRL_REG4;
		wprowadz_zmienna = 0x80; /*0b10000000*/
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1,&zapytaj_o_zmienna,sizeof(wprowadz_zmienna),1);
			HAL_SPI_Transmit(&hspi1,&wprowadz_zmienna,sizeof(wprowadz_zmienna),1);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
		
		HAL_Delay(1);
	
		while(1)
		{
				/// pobieramy x,y,z
			
				zapytaj_o_zmienna = L3GD20_OUT_X_L | L3GD20_READ_BIT | L3GD20_MS_BIT;
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
					HAL_SPI_Transmit(&hspi1,&zapytaj_o_zmienna,sizeof(zapytaj_o_zmienna),1);
					HAL_SPI_Receive (&hspi1,tab_xyz,sizeof(tab_xyz),1);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);			
				
				HAL_Delay(25);
			
				/// obliczamy x,y,z
				
						     /*High*/           /*Low*/
				g_x = (int16_t)((tab_xyz[1] << 8) | tab_xyz[0]);
				g_y = (int16_t)((tab_xyz[3] << 8) | tab_xyz[2]);
				g_z = (int16_t)((tab_xyz[5] << 8) | tab_xyz[4]);
			
				/*od -32767 przez 0 do +32767 */
			
				/// obliczenie kata
				
				/*konwertowanie pomiaru na stopnie*/
				/*mnozenie przez stala = 8.75 i skalowanie z jednostki mili ; bo FS = 250 dps*/
				g_x = ( g_x * 875 / 100000 ); 
				g_y = ( g_y * 875 / 100000 ); 
				g_z = ( g_z * 875 / 100000 ); 
			
				/// calkowanie 
				/*stala = suma czasu petli - 50ms + niedokladnosc transmisji spi -> stala czasowa = 0.05*/
				odchylenie_x += g_x * 5 / 100; 
				odchylenie_y += g_y * 5 / 100; 
				odchylenie_z += g_z * 5 / 100; 
			
				/// temperatura

				zapytaj_o_zmienna = L3GD20_OUT_TEMP | L3GD20_READ_BIT;
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
					HAL_SPI_Transmit(&hspi1,&zapytaj_o_zmienna,sizeof(zapytaj_o_zmienna),1);
					HAL_SPI_Receive(&hspi1,&g_t,sizeof(g_t),1);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);			
				
				HAL_Delay(25);
				
				/// komunikacja przez VCOM
			
				if(UserRxBufferFS[0] == '1'){
					/* 0-360  * 16 ;   -180 stopni | 0 stopni | +180 stopni  */
					/* 0-5760      ;        0      |   2880   |  5760        */
						sprintf(tab_char,">g;%d,%d,%d<\n\r",odchylenie_x,odchylenie_y,odchylenie_z);
						for(i=0;i<30;i++)
							tab_uint8_t[i] = tab_char[i];
						CDC_Transmit_FS(tab_uint8_t,sizeof(tab_uint8_t));
						UserRxBufferFS[0] = 0;
				}

		}		
}

/*----------------------------------------------------------------------------*/

/** System Clock Configuration */
void SystemClock_Config(void){
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line){}
#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
