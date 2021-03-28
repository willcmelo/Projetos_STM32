/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fonts.h"
#include "tft.h"
#include "user_setting.h"
#include "functions.h"

//Biblioteca de ajuste de funções para o SD Card(Criada por terceiro)
#include "fatfs_sd.h"
#include <string.h>
#include <stdio.h>
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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t ID=0;
FATFS fs0, fs1;  /* Work area (filesystem object) for logical drives */
FIL fsrc, fdst;  /* File objects */
BYTE buffer[4096];   /* File copy buffer */
FRESULT fr;      /* FatFs function common result code */
UINT br, bw;     /* File read/write count */
//Transferência de arquivos BMP do SD Card para o LCD
uint32_t first;
extern const unsigned short fundo0[60000];
extern const unsigned short fundo_1_copy[60000];
extern const unsigned short fundo_2[60000];
extern const unsigned short fundo_3[60000];
int Flag ;
uint8_t RX1_Char = 0x00;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_USART_RxCpltCallback(UART_HandleTypeDef *huart);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//int32_t i;
	//int32_t size;
	//char texto[50];
	//char nome[40];
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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &RX1_Char, 1);
  tft_gpio_init();
  HAL_TIM_Base_Start(&htim1);
  ID = tft_readID();
  HAL_Delay(100);
  tft_init (ID);
  setRotation(1);
  fillScreen(BLACK);
  printnewtstr(20, YELLOW, &mono12x7bold, 1, (uint8_t *)"Bem Vindo ao Game");
  printnewtstr(40, YELLOW, &mono12x7bold, 1,(uint8_t *)" DESCUBRA O BANDIDO ");
  printnewtstr(60, RED, &mono12x7bold, 1,(uint8_t *)" Dica 1:  ");
  printnewtstr(80, YELLOW, &mono12x7bold, 1, (uint8_t *)"A bota aparece");
  printnewtstr(100, RED, &mono12x7bold, 1,(uint8_t *)" Dica 2:  ");
  printnewtstr(120, YELLOW, &mono12x7bold, 1,(uint8_t *)"A Luz indica ");
  printnewtstr(140, WHITE, &mono12x7bold, 1,(uint8_t *)" Tecla 1, 2 e  3 ");
  printnewtstr(160, WHITE, &mono12x7bold, 1,(uint8_t *)" SELECIONA  ");
  printnewtstr(180, WHITE, &mono12x7bold, 1,(uint8_t *)" O CULPADO !!! ");
  printnewtstr(200, WHITE, &mono12x7bold, 1,(uint8_t *)" BOA SORTE ");
  HAL_Delay(10000);
  fillScreen(BLACK);
  drawRGBBitmap(10, 237, fundo0, 300, 200);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	  while (1)
	  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		  if(RX1_Char == '1') //Personagem 1
		  		    	  	{

		  			              drawRGBBitmap(10, 237, fundo_1_copy, 300, 200);
		  		    	  	      HAL_UART_Receive_IT(&huart2, &RX1_Char, 1);
		  		    	  	      RX1_Char = 0x00;
		  		    	  	      HAL_Delay(5000);

		  		    	  	      fillScreen(BLACK);
		  		    	  	      printnewtstr(100, YELLOW, &mono12x7bold, 1,(uint8_t *)" QUE PENA ");
		  		    	  	      printnewtstr(120, YELLOW, &mono12x7bold, 1,(uint8_t *)" VOCE PERDEU ");
		  		    	  	      HAL_UART_Receive_IT(&huart2, &RX1_Char, 1);
		  		    	  	      RX1_Char = 0x00;

		  		    	  	}
		  		    	  	else if(RX1_Char == '2')  // Personagem 2
		  		    	    	  	{
		  		    	  		                drawRGBBitmap(10, 237, fundo_2, 300, 200);
		  		    	    	  	    	    HAL_UART_Receive_IT(&huart2, &RX1_Char, 1);
		  		    	    	  	    		RX1_Char = 0x00;
		  		    	    	  	    	    HAL_Delay(5000);

		  		    	    	  	    	    fillScreen(BLACK);
		  		    	    	  	    		printnewtstr(100, YELLOW, &mono12x7bold, 1,(uint8_t *)" PARABENS ");
		  		    	    	  	    	    printnewtstr(120, YELLOW, &mono12x7bold, 1,(uint8_t *)" VOCE ACERTOU ");
		  		    	    	  	    	    HAL_UART_Receive_IT(&huart2, &RX1_Char, 1);
		  		    	    	  	    	    RX1_Char = 0x00;

		  		    	    	  	}
		  		    	  	else if(RX1_Char == '3')  //Personagem 3
		  		  		    	    {

		  		  		    	 		        drawRGBBitmap(10, 237, fundo_3, 300, 200);
		  		  		    	 		        HAL_UART_Receive_IT(&huart2, &RX1_Char, 1);
		  		  		    	 		    	RX1_Char = 0x00;
		  		  		    	 		        HAL_Delay(5000);

		  		  		    	 		   	    fillScreen(BLACK);
		  		  		    	 		   		printnewtstr(100, YELLOW, &mono12x7bold, 1,(uint8_t *)" QUE PENA ");
		  		  		    	 		   		printnewtstr(120, YELLOW, &mono12x7bold, 1,(uint8_t *)" VOCE PERDEU ");
		  		  		    	 		   		HAL_UART_Receive_IT(&huart2, &RX1_Char, 1);
		  		  		    	 		   	    RX1_Char = 0x00;

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xFFFF-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_USART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_IT(&huart2, &RX1_Char, 1);
}
//Envia uma string como um bitmp para o LCD TFT
//Recebe: String com 512 bytes (1 setor de arquivo) que fazem parte da imagem
//				Flag que sinaliza se é o primeiro setor (onde está o cabeçalho)
/*
void LCD_TxBMP(unsigned char* data, unsigned int BitmapStart)
{
	static unsigned int tamanho = 0;
	static unsigned char sobrou[3] = {0,0,0};
	unsigned int i = 0;
	unsigned int setor = 512;
	unsigned short int cor;
	unsigned char tam[11];
	unsigned char altura, x = 30, y = 250, bits_por_pixel;
	static unsigned char largura = 0, erro_bits = 0, bytes_extras = 0, pixels_por_linha = 0;

	//Se é o primeiro setor do arquivo, possui o cabeçalho
	if(BitmapStart)
	{
		BitmapStart = 0;

		//Reseta variável estática
		pixels_por_linha = 0;

		//Pula o cabeçalho
		i = 54;
		//Lê o tamanho da área de dados do arquivo em bytes
		tamanho = data[0x22] + (unsigned int)(data[0x23]<<8) + (unsigned int)(data[0x24]<<16) + (unsigned int)(data[0x25]<<24);
		//Leandro (01/09/2019) - Lê a largura e a altura da imagem para definir o tamanho da janela
		largura = data[18];
		altura = data[22];
		//Configura a janela
		setAddrWindow(x, y-altura+1, x+largura-1, y);
		//Envia para o LCD sinalização de início de envio de dados
		inicioDados();
		//Verifica se existirão bytes extras no arquivo em função da largura da imagem
		//Obervação: Existe uma restrição de que cada linha deva ter N bytes, sendo N um número
		//divisível por 4. Caso contrário, o BMP deve ser preenchido com bytes não válidos. Por
		//exemplo, se a imagem tem 1 x 100 pixels em 24 bits/pixel, o BMP teria 3 bytes válidos em
		//cada linha e mais 1 byte que não tem qualquer significado.
		switch((largura*3)%4)
		{
			case 1: bytes_extras = 3; break;
			case 2: bytes_extras = 2; break;
			case 3: bytes_extras = 1; break;
			default: bytes_extras = 0; break;
		}

		//Lê a quantidade de bits por pixel (neste caso é aceito apenas 24 bits por pixel)
		bits_por_pixel = data[28];
		//Testa a quatidade de bits
		if((bits_por_pixel != 24) || (largura > 300) || (altura > 300))
			erro_bits = 1;
		else
			erro_bits = 0;
	}

	//Se houver erro na quantidade de bits retorna e não envia para o LCD
	if(erro_bits)
	{
		return;
	}

	//Envia os pixels enquanto não acabar o setor ou o Bitmap
	while((i <= (512-3)) && (tamanho >= 3))	//24 bits por pixels
	{
		//Se completou uma linha
		if(pixels_por_linha == largura)
		{
			//Zera o contador
			pixels_por_linha = 0;
			//Verifica se tem bytes nulos para ignorar
			if(bytes_extras >= sobrou[0])
			{
				//Desconta os bytes_extras-sobrou[0] do tamanho do setor
				tamanho -= (bytes_extras-sobrou[0]);
				//Incrementa a posição do byte a ser lido do setor
				i += (bytes_extras-sobrou[0]);
				//Atualiza o valor da sobra
				sobrou[0] = 0;
				//Verifica se não cabe mais nenhum pixel, encerra o loop
				if((i>(512-3)) || (tamanho < 3))
					break;
			}
			else
			{
				//Atualiza o valor da sobra
				sobrou[0] -= bytes_extras;
			}
			//break;
			if(tamanho<3)
				break;
		}

		if(sobrou[0] == 0)			//Tamanho -= 3
		{
			//((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
			//Seguencia BGR (24 bits) --> RGB (565)
			cor = (data[i] >> 3) | ((data[i+1] & 0xFC) << 3) | ((data[i+2] & 0xF8) << 8);
			i += 3;
			tamanho -= 3;
		}
		else if(sobrou[0] == 1)	//Tamanho -= 2
		{
			//Sobrou a cor Azul
			cor = (sobrou[2] >> 3) | ((data[i] & 0xFC) << 3) | ((data[i+1] & 0xF8) << 8);
			i += 2;
			tamanho -= 2;
		}
		else if(sobrou[0] == 2)	//Tamanho -= 1
		{
			//Sobrou a cor Azul e Verde
			cor = (sobrou[1] >> 3) | ((sobrou[2] & 0xFC) << 3) | ((data[i] & 0xF8) << 8);
			i += 1;
			tamanho -= 1;
		}
		else
		{
			i = 512;
			setor = 0;
			tamanho = 0;
			break;
		}
		//Envia pixel 565 para o LCD
		desenhaPixel(cor);

		sobrou[0] = 0;	//Sobra algum byte apenas no final do setor (i>= 510)

		//Incrementa o número de pixels enviados por linha e testa
		pixels_por_linha++;
	}
	//Se ainda não acabou o arquivo
	if(tamanho >= 3)
	{
		//Salva o número de bytes que sobreram para formar um pixels
		sobrou[0] = 512 - i;
		//Completa os 512 bytes do setor
		tamanho -= sobrou[0];
		//Salva o penúltimo byte
		sobrou[1] = data[510];
		//Salva o último byte
		sobrou[2] = data[511];
	}
	else
	{
		//Envia para o LCD sinalização de fim de envio de dados
		fimDados();
	}
}

//Gera uma cor no formato RGB 565
uint16_t geraRGB565(uint8_t red, uint8_t green, uint8_t blue)
{
	//rrrrrggg gggbbbbb
	return ((red<<8)&0xF800) + ((green<<5)&0x07E0) + ((blue>>3)&0x001F);
}

/*----------------------------------------------------------------------------/*

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
