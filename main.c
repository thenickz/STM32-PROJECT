/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "ssd1306.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define 	incremento  6553
#define 	decremento  6553
#define 	limite		65535


#define		BT_INC		GPIO_PIN_2
#define		BT_DEC		GPIO_PIN_0
#define		BT_SEL		GPIOB, GPIO_PIN_9


#define 	SD_CARD_CS	GPIOB, GPIO_PIN_0
#define		LED_debug	GPIOC, GPIO_PIN_13
#define		Buzzer		TIM3 -> CCR4
#define		Fan			TIM2 -> CCR3
#define 	POT			HAL_ADC_GetValue(&hadc1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
uint8_t flag1 = 0;    //flag do botao de incremento
uint8_t flag2 = 0;    //flag do botao de decremento
uint8_t USB_Buffer[255]; //vetor com as informacoes do usb
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
/*funcao que pisca led interno da bluepill*/
void toggle_blink(){
	HAL_GPIO_TogglePin(LED_debug);
	HAL_Delay(100);
	HAL_GPIO_TogglePin(LED_debug);
}
/*funcao que escreve no painel oled o RPM atual da Fan*/
void ssd1306_RPM(uint16_t rpm) {

    uint8_t y = 0;        				// ponto inicial
    ssd1306_Fill(Black); 				// preenche tudo com preto pelo oled

    char str[256];
    sprintf((char *)str, "Fan Speed: %u RPM", rpm);	// prepara string com o rpm atual

    #ifdef SSD1306_INCLUDE_FONT_6x8
    ssd1306_SetCursor(2, y);				// posiciona o cursor na primeira linha e primeira coluna
    ssd1306_WriteString(str, Font_6x8, White);  	// escreve no display a string com fonte 6x8 na cor branca
    #endif

    ssd1306_UpdateScreen();				// Atualiza a tela
}
/*Funcao para criar um arquivo no sd card*/
/*contem varios printf comentados para verificar cada processo se necessário*/
void process_SD_card( void )
{
  FATFS       FatFs;                //Fatfs handle
  FIL         fil;                  //File handle
  FRESULT     fres;                 //Result after operations
  char        buf[100];

  do
  {
    //Monta o SD Card
    fres = f_mount(&FatFs, "", 1);    			// 1 = montar agora
    if (fres != FR_OK) 					// se nao localizar o cartão encerra a funcao
    {
      //printf("No SD Card found : (%i)\r\n", fres);
      break;
    }
    //printf("SD Card Mounted Successfully!!!\r\n");

    /*Le o tamanho total do cartão SD e o tamanho livre*/
    FATFS *pfs;
    DWORD fre_clust;
    uint32_t totalSpace, freeSpace;

    f_getfree("", &fre_clust, &pfs);
    totalSpace = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
    freeSpace = (uint32_t)(fre_clust * pfs->csize * 0.5);

    //printf("TotalSpace : %lu bytes, FreeSpace = %lu bytes\n", totalSpace, freeSpace);

    /*Abre o arquivo*/
    fres = f_open(&fil, "info.txt", FA_WRITE | FA_READ | FA_CREATE_ALWAYS);
    if(fres != FR_OK)							// se nao conseugir abrir, encerra
    {
      //printf("File creation/open Error : (%i)\r\n", fres);
      break;
    }

    //printf("Writing data!!!\r\n");
    // Escreve dados
    f_puts("Trabalho Finalmente ta pronto", &fil);
	  
    //Fecha arquivo
    f_close(&fil);

    //Abre arquivo
    fres = f_open(&fil, "info.txt", FA_READ);
    if(fres != FR_OK)						// se nao conseguir abrir, encerra
    {
      //printf("File opening Error : (%i)\r\n", fres);
      break;
    }

    //Le os dados do arquivo
    f_gets(buf, sizeof(buf), &fil);

    //printf("Read Data : %s\n", buf);

    // Fecha o arquvo
    f_close(&fil);
    //printf("Closing File!!!\r\n");
  } while( 0 );

  //No final, desmonta o sd card
  f_mount(NULL, "", 0);
  //printf("SD Card Unmounted Successfully!!!\r\n");
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
	uint32_t tick; 			// armazena valor do systick
	/* Variaveis para USB */
	uint16_t usb_fan_speed;		// armazena a velocidade
	uint16_t usb_pot_value;		// armazena valor do potenciometro
	uint16_t usb_sd_status = 0;	// armazena se o cartao sd foi gravado
	uint16_t usb_buzzer_value;	// armazena valor do buzzer
	// armazena string que vai enviar todos os dados para o usb
	char json_string[] = "{\"FAN\": %u, \"POT\": %u, \"SD\": %u, \"BUZZER\": %u} \n";

	/* Variaveis para controlar melhor a lógica dos componentes*/ 
	uint16_t BuzzerValue = 0; // Valor do Buzzer atual
	uint16_t estouro = 0;	  // Valor de estouro do volume do Buzzer
	uint16_t readValue = 0;	  // guarda o valor lido no adc do potencimetro
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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	
  /* Parametros de inicialização */
  HAL_ADCEx_Calibration_Start(&hadc1); 		// prepara o ADC1 para funcionar
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);	// Ativa o gerador de pwm do canal 3 do timer 2
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);	// Ativa o gerador de pwm do canal 4 do timer 4
  ssd1306_Init();				// Inicia o OLED
  HAL_GPIO_WritePin(LED_debug, 1);		// Deixa o led_debug desligado

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //process_SD_card();

  while (1)
  {
	  /* verifica se chegou requisição no usb */
	  if (USB_Buffer[0] != 0){
		  switch(USB_Buffer[0]){
			  case 'I':{		// se for "I" ativa a flag1 para Incremento
				  flag1 = 1;
			  }
			  case 'D':{
				  flag2 = 1;	// se for "D" ativa a flag2 para Decremento
			  }
		  }
	  }

	  /* buzzer com botao e interrupcao externa */
	  if (flag1){
		  
		  if (BuzzerValue + incremento <= limite){	// primeiro verifica se pode incrementar sem Overflow(estouro)
			  BuzzerValue += incremento;		// se sim, incrementa
		  }else{
			  estouro = limite - BuzzerValue;	// se nao, descobre quanto que falta para o estouro
			  BuzzerValue += estouro;		// incrementa apenas o quanto falta para o estouro
		  }
		  flag1 = 0;					// reseta a flag da interrupcao externa
		  toggle_blink();				// led_debug para ter um feedback que o botao foi pressionado e executado como esperado

	  }
	  if (flag2){						
		  if (BuzzerValue - decremento >= 0){		// primeiro verifica se pode decrementar sem Overflow(estouro)
			  BuzzerValue -= decremento;		// se sim, decrementa
		  }else{
			  estouro = BuzzerValue;		// se nao, descobre quanto que falta para o estouro
			  BuzzerValue -= estouro;		// decrementa apenas o quanto falta para o estouro
		  }
		  flag2 = 0;					// reseta a flag da interrupcao externa
		  toggle_blink();				// led_debug para ter um feedback que o botao foi pressionado e executado como esperado
	  }
	  Buzzer = BuzzerValue;					//PWM do Buzzer recebe algum valor

	  /* fan com ponteciometro */
	  if (HAL_GPIO_ReadPin(BT_SEL) == 1){			// verifica se o BT_SEL nao esta pressionado
	
		  HAL_ADC_Start(&hadc1);			// inicia o ADC do potenciometro
		  HAL_ADC_PollForConversion(&hadc1,1);		// Prepara para receber informacao do ADC
		  readValue = POT;				// armazena o valor atual recebido
		  Fan = 65535*readValue/4095;			// converte o valor do ADC para a resolucao do PWM
		  HAL_Delay(1);					// Delay de 1ms

	  }else{						// se BT_SEL estiver pressionado
		  /* Lógica de implementação do SD_Card*/
		  readValue = 0;				// armazena zero
		  Fan = readValue;				// velocidade da fan zerada
		  if (!usb_sd_status){				// verifica se o sd card nao foi gravado
			  //process_SD_card();			// grava dados no sd card (como acabou a memoria do stm32 essa parte foi comentada)
			  usb_sd_status = 1;			// atualiza o status do sd card para gravado
		  }
	  }

	  /* Lógica USB e OLED */
	  if ((HAL_GetTick() - tick ) > 333){										// a cada 333ms o USB e atualizado
		  tick = HAL_GetTick();											// armazena valor atual do systick
		  ssd1306_RPM(16500*readValue/4095);									// escreve o valor em RPM do OLED
		  usb_fan_speed = readValue;										// armazena o valor do ADC para o fan
		  usb_pot_value = readValue;										// armazena o valor do ADC para o potenciometro
		  usb_buzzer_value = BuzzerValue;									// armazena o valor do timer do Buzzer
		  sprintf(USB_Buffer, json_string, usb_fan_speed, usb_pot_value, usb_sd_status, usb_buzzer_value);	// prepara a string para se parecer com um JSON
		  CDC_Transmit_FS(USB_Buffer, strlen(USB_Buffer));							// envia a string para o usb, NodeRED recebe os dados

	  }

	  memset(USB_Buffer, 0, 255);											// zera todas as posicoes



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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* Funcao com a Logica da Interrupcao Externa */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == BT_INC)
    {
    	flag1 = 1;		// flag que o botao de incremento foi pressionado
    }
    if(GPIO_Pin == BT_DEC)
    {
    	flag2 = 1;		// flag que o botao de decremento foi pressionado
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
