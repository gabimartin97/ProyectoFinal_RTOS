/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "string.h"
#include "fatfs_sd.h"
#include "stdbool.h"
#include "FIR.h"
#include "make_wav.h"
//#include "math.h"
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
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
/*----------------------TARJETA SD ------------------------------------------*/
static const char *SD_errorSD_okMsg = "ERROR IN MOUNTING SD CARD \n";
static const char *SD_okMsg = "SD CARD mounted successfully \n";

FATFS fs; //file system
FIL fil; //file
FRESULT fresult; //to store the result
char buffer[1024]; // buffer para enviar y recibir mensajes por usb
UINT br, bw;  //file read/write count

/*Capacity related variables*/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;
/*----------------------TARJETA SD ------------------------------------------*/

/*----------------------ADC------------------------------------------*/
#define recordingTime 5 // Tiempo de grabacion en segundos
// ---------FRECUENCIA DE MUESTREO ----------/
#define  sampleRate 5303
/* Calculado ->>
 * tconv = (Sampling Time + 15 cycles)/(APB2 Clock / Prescaler) = (480 + 15)/(2.625 Mhz)
 * tconv sampleRate = 1/tconv
 */
// ---------FRECUENCIA DE MUESTREO ----------/
#define adcBuff1Size 500
#define adcBuff2Size 500
static uint32_t adcBuff1[adcBuff1Size]; //Buffer 1 para almacenar lecturas del ADC
static uint32_t adcBuff2[adcBuff2Size]; //Buffer 2 para almacenar lecturas del ADC
int adcCount1 = 0;
int adcCount2 = 0;
static bool buff1 = true;
static bool buff2 = false;
bool doneADC = false;
static bool start = false;

uint32_t samples_count = 0; //Contador de muestras
static const uint32_t muestras = recordingTime * sampleRate; //Cantidad de muestras totales que se deben adquirir
/*----------------------ADC------------------------------------------*/

/****************FILTRO*********************/
#define FIR_FILTER_LENGHT 64
FIRFilter filtroPB;
static float FIR_IMPULSE_RESPONSE[FIR_FILTER_LENGHT] = { -0.0005017f,
		-0.0010401f, -0.0015571f, -0.0019069f, -0.0019183f, -0.0014621f,
		-0.0005568f, 0.0005421f, 0.0013410f, 0.0012421f, -0.0002012f,
		-0.0029985f, -0.0065400f, -0.0096489f, -0.0109215f, -0.0092850f,
		-0.0045845f, 0.0020577f, 0.0083285f, 0.0113412f, 0.0086597f,
		-0.0005801f, -0.0148709f, -0.0302278f, -0.0409512f, -0.0411462f,
		-0.0265899f, 0.0036339f, 0.0463075f, 0.0944202f, 0.1386798f, 0.1698179f,
		0.1810296f, 0.1698179f, 0.1386798f, 0.0944202f, 0.0463075f, 0.0036339f,
		-0.0265899f, -0.0411462f, -0.0409512f, -0.0302278f, -0.0148709f,
		-0.0005801f, 0.0086597f, 0.0113412f, 0.0083285f, 0.0020577f,
		-0.0045845f, -0.0092850f, -0.0109215f, -0.0096489f, -0.0065400f,
		-0.0029985f, -0.0002012f, 0.0012421f, 0.0013410f, 0.0005421f,
		-0.0005568f, -0.0014621f, -0.0019183f, -0.0019069f, -0.0015571f,
		-0.0010401f };

FIL unfilteredData;
FIL filteredData;
float circularBuffer[FIR_FILTER_LENGHT] = { 0 };
UINT br2, bw2;  //file read/write count
		/****************FILTRO*********************/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Clar buffer
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
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

	HAL_Delay(1000);
	if (f_mount(&fs, "", 0) != FR_OK) {

		CDC_Transmit_FS((uint8_t*) SD_errorSD_okMsg, strlen(SD_errorSD_okMsg)); //Montaje de la tarjta SD fallido
	} else {

		CDC_Transmit_FS((uint8_t*) SD_okMsg, strlen(SD_okMsg)); //Montaje de la tarjta SD correcto

	}
	/*-----------------Mount SD card-----------*/

	/*-----------------Card capacity details-----------*/
	f_getfree("", &fre_clust, &pfs);

	total = (uint32_t) ((pfs->n_fatent - 2) * pfs->csize * 0.5);
	HAL_Delay(1000);
	sprintf(buffer, "SD CARD total Size: \t%lu\n", total);
	CDC_Transmit_FS((uint8_t*) buffer, strlen(buffer));
	memset(buffer, 0, strlen(buffer));

	free_space = (uint32_t) (fre_clust * pfs->csize * 0.5);
	sprintf(buffer, "SD CARD Free space: \t%lu\n", free_space);
	HAL_Delay(1000);
	CDC_Transmit_FS((uint8_t*) buffer, strlen(buffer));
	memset(buffer, 0, strlen(buffer));
	/*-----------------Card capacity details-----------*/

	f_mount(NULL, "", 1);

	HAL_Delay(1000);

	if (f_mount(&fs, "", 0) != FR_OK) {

		CDC_Transmit_FS((uint8_t*) SD_errorSD_okMsg, strlen(SD_errorSD_okMsg));
	} else {

		CDC_Transmit_FS((uint8_t*) SD_okMsg, strlen(SD_okMsg));

	}
	memset(buffer, 0, strlen(buffer));
	HAL_Delay(1000);

	/*-----------CREO EL ARCHIVO PARA ALMACENAR LOS DATOS DEL ADC SIN FILTRAR--------------------*/
	if (f_open(&unfilteredData, "unf.csv",
	FA_CREATE_ALWAYS | FA_READ | FA_WRITE) != FR_OK) {
		sprintf(buffer, "ERROR abriendo unfiltered 1");
		CDC_Transmit_FS((uint8_t*) buffer, strlen(buffer));
		memset(buffer, 0, strlen(buffer));
	}
	/*-----------CREO EL ARCHIVO PARA ALMACENAR LOS DATOS DEL ADC SIN FILTRAR--------------------*/

	//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); //Cuando se apaga el LED comienza la conversion

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		/* Debido a que tantas mustras del ADC no entran en la RAM. Vamos grabando de a bloques del tamaño
		 * 	de adcBuff[] y cuando este se llena, volvamos todo al archivo de texto. la variable adcCount lleva el
		 * 	conteo de cuanto buffer se lleno
		 */

		// Oprimo el pulsador
		if(!start && (HAL_GPIO_ReadPin(Pulsador_GPIO_Port, Pulsador_Pin) == GPIO_PIN_RESET) )
		{
			start = true;
			HAL_Delay(1000);
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
			HAL_ADC_Start_IT(&hadc1);   //ARRANCA EL ADC POR INTERRPUCION

		}


		if (adcCount1 >= adcBuff1Size) { //Si se llenó el buffer 1
			adcCount1 = 0;
			buff1 = false;
			buff2 = true;				//hago que el ADC comience a almacenar en el buff2

			for (int i = 0; i < adcBuff1Size; i++) {

				f_printf(&unfilteredData, "%04u,", adcBuff1[i]);  //Guardamos en archivo el buffer 1
			}

		}

		if (adcCount2 >= adcBuff2Size) { //Si se llenó el buffer 2
			adcCount2 = 0;
			buff1 = true;				//hago que el ADC comience a almacenar en el buff1
			buff2 = false;

			for (int i = 0; i < adcBuff2Size; i++) {

				f_printf(&unfilteredData, "%04u,", adcBuff2[i]); //Guardamos en archivo el buffer 2
			}

		}

		if (samples_count >= muestras)	// Si se obvtuvieron todas las muestras deseadas
				{

			HAL_ADC_Stop_IT(&hadc1); 				//Detenemos el ADC

			if (buff1 && adcCount1 > 0 && adcCount1 < adcBuff1Size) { //almacenamos lo que quedó del buff 1

				for (int i = 0; i < (adcCount1 - 1); i++) {

					f_printf(&unfilteredData, "%04u,", adcBuff1[i]);
				}
				f_printf(&unfilteredData, "%04u", adcBuff1[adcCount1 - 1]); // Ultima muestra sin coma
			}
			if (buff2 && adcCount2 > 0 && adcCount2 < adcBuff2Size) {  //almacenamos lo que quedó del buff 2

				for (int i = 0; i < (adcCount2 - 1); i++) {

					f_printf(&unfilteredData, "%04u,", adcBuff2[i]);
				}
				f_printf(&unfilteredData, "%04u", adcBuff2[adcCount2 - 1]); // Ultima muestra sin coma
			}

			f_close(&unfilteredData);
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			samples_count = 0;
			doneADC = true;
			HAL_Delay(1000);

		}

		if (doneADC) { //Una vez grabado el adc en el archivo hay que leerlo y aplicarle el filtro
			doneADC = false;

			FIRFilter_Init(&filtroPB, FIR_IMPULSE_RESPONSE, circularBuffer,
			FIR_FILTER_LENGHT); 		//inicio el filtro

			if (f_open(&filteredData, "filtered.csv",		//Creo el archivo para almacenar muestras filtradas
			FA_CREATE_ALWAYS | FA_READ | FA_WRITE) != FR_OK) {
				memset(buffer, 0, strlen(buffer));
				sprintf(buffer, "ERROR abriendo filtered ");
				CDC_Transmit_FS((uint8_t*) buffer, strlen(buffer));
			}

			HAL_Delay(100);

			if (f_open(&unfilteredData, "unf.csv", FA_READ) != FR_OK) {		//Abro el archivo de las muestras sin filtro
				memset(buffer, 0, strlen(buffer));
				sprintf(buffer, "ERROR abriendo unfiltered 2");
				CDC_Transmit_FS((uint8_t*) buffer, strlen(buffer));
			} else {

				char readBuffer[4] = { 0 };	//buffer para leer 4 char que son los digitos de un dato int
				char writeBuffer[6] = { 0 }; //buffer para escribir en char un numero int
				float number = 0.0f;
				int intNumber = 0;
				int i = 0;
				UINT bytesLeidos = 0;
				//UINT bytesEscritos = 0;
				//FRESULT fr;
				BYTE buffer[1]; // array de 1, es decir, un solo caracter

				for (;;) { //Bucle que lee de un archivo csv, aplica el filtro y escribe en el otro

					f_read(&unfilteredData, buffer, sizeof buffer,
							&bytesLeidos);  //Leo  un char
					if (bytesLeidos == 0) break; /* error or eof */

					switch (buffer[0]) {
					case ',': //Si el char es una coma quiere decir que ya lei el numero completo
						i = 0;
						number = atof(readBuffer); //paso de una cadena de char a un float
						FIRFilter_Update(&filtroPB, number); // Actualizo el filtro FIR con el float
						intNumber = (int) filtroPB.out; //El nuevo valor es la salida del filtro FIR
						if (intNumber < 0)		//Por si el filtro devuelve valores negativos
							intNumber = 0;
						sprintf(writeBuffer, "%04d,", intNumber); //Escribo una cadena de char a partir de un int
						f_puts(writeBuffer, &filteredData);
						memset(readBuffer, 0, strlen(readBuffer)); 	  //Vacio los buffer
						memset(writeBuffer, 0, strlen(writeBuffer)); //Vacio los buffer
						break;
					default: //Si el char no es una coma voy almacenando los digitos del numero en un array
						readBuffer[i++] = buffer[0];
						if (i > strlen(readBuffer))
							i = 0;
						break;
					}

				}

				//Borro la ultima ',' del archivo
				//f_lseek(&filteredData, f_tell(&filteredData) - 1);
				//f_putc('\0', &filteredData);

			}

			f_close(&unfilteredData);
			f_close(&filteredData);
			HAL_Delay(100);

			//Creo  un archivo .wav a partir de un archivo csv con esta funcion
			write_wav_from_csv("unf.csv", "unfilt.wav", muestras, sampleRate);
			sprintf(buffer, "Done unfilt");
			CDC_Transmit_FS((uint8_t*) buffer, strlen(buffer));
			memset(buffer, 0, strlen(buffer));
			HAL_Delay(200);
			write_wav_from_csv("filtered.csv", "filt.wav", muestras, sampleRate);
			sprintf(buffer, "Done filt");
			CDC_Transmit_FS((uint8_t*) buffer, strlen(buffer));
			memset(buffer, 0, strlen(buffer));
			HAL_Delay(200);

			f_mount(NULL, "", 1); //Desmonto la SD

			memset(buffer, 0, strlen(buffer));
			sprintf(buffer, "\n DONE");
			CDC_Transmit_FS((uint8_t*) buffer, strlen(buffer));
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Pulsador_Pin */
  GPIO_InitStruct.Pin = Pulsador_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Pulsador_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	//Tenemos dos vectores buffer

	if (buff1 && (adcCount1 < adcBuff1Size)) {
		adcBuff1[adcCount1++] = HAL_ADC_GetValue(&hadc1);
		samples_count++;
	}

	if (buff2 && (adcCount2 < adcBuff2Size)) {
		adcBuff2[adcCount2++] = HAL_ADC_GetValue(&hadc1);
		samples_count++;
	}
	/*If continuousconversion mode is DISABLED uncomment below*/
	HAL_ADC_Start_IT(&hadc1);
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
	while (1) {
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

