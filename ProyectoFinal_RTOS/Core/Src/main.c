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
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fatfs_sd.h"
#include "string.h"
#include "stdbool.h"
#include "stdio.h"
#include "FIR.h"
#include "stdlib.h"
#include "make_wav.h"
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
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

osThreadId LecturaPulsadorHandle;
osThreadId TarjetaSDHandle;
osThreadId MainTaskHandle;
osThreadId ManejoLEDsHandle;
osThreadId RunningTaskHandle;
osMessageQId ADC1_QueueHandle;
uint8_t ADC1_QueueBuffer[ 500 * sizeof( uint16_t ) ];
osStaticMessageQDef_t ADC1_QueueControlBlock;
osMessageQId Pulsadores_QueueHandle;
osMessageQId SD_CMD_QueueHandle;
osMessageQId SD_STATUS_QueueHandle;
osMessageQId ADC1_Queue2Handle;
uint8_t ADC1_Queue2Buffer[ 100 * sizeof( uint16_t ) ];
osStaticMessageQDef_t ADC1_Queue2ControlBlock;
/* USER CODE BEGIN PV */

// ---------ESTADOS DEL PROGRAMA----------/
static bool ready = true;
static bool start = false;
static bool stop = false;
static bool grabarECG = false;
static bool grabarAudio = false;

static bool grabandoAudio = false;
static bool grabandoECG = false;
static bool filtrandoAudio = false;
static bool creandoWAV = false;
static bool error = false;
// ---------ESTADOS DEL PROGRAMA----------/

/*----------------------TARJETA SD ------------------------------------------*/

typedef enum {
	CMD_Nada,					//Ninguno
	CMD_GrabarADC_ECG,
	CMD_GrabarADC_Audio,
	CMD_Filtrado_Audio,
	CMD_ArchivoWav,
	CMD_Stop

} ComandosSD;

typedef enum {
	STAT_Listo,					//Ninguno
	STAT_Grabando_ECG,
	STAT_Grabando_Audio,
	STAT_Filtrando_Audio,
	STAT_Creando_WAV,
	STAT_Error

} EstadoSD;
/*----------------------TARJETA SD ------------------------------------------*/

/*----------------------ADC------------------------------------------*/
#define recordingAudioTime 5 // Tiempo de grabacion en segundos
#define recordingECGTime 5 // Tiempo de grabacion en segundos
// ---------FRECUENCIA DE MUESTREO ----------/
#define  SAMPLE_RATE_AUDIO 5303
#define  SAMPLE_RATE_ECG 1303
/* Calculado ->>
 * tconv = (Sampling Time + 15 cycles)/(APB2 Clock / Prescaler) = (480 + 15)/(2.625 Mhz)
 * tconv SAMPLE_RATE_AUDIO = 1/tconv
 */
// ---------FRECUENCIA DE MUESTREO ----------/

uint16_t contadorTest = 0;
uint32_t contadorIT_ADC = 0; //contador rutina de interrupción ADC
static const uint32_t samplesAudio_Total = recordingAudioTime
		* SAMPLE_RATE_AUDIO; //Cantidad de samplesAudio_Total totales que se deben adquirir
static const uint32_t samplesECG_Total = recordingECGTime * SAMPLE_RATE_ECG; //Cantidad de samplesAudio totales que se deben adquirir
uint32_t samplesAudio_count = 0; //Contador de samplesAudio
uint32_t samplesECG_count = 0; //Contador de samplesAudio
bool listoADC_Audio = false;
bool listoADC_ECG = false;

uint32_t datosDMA_ADC[2];

int numGrabacionADC = 0;

/****************FILTRO*********************/
#define FIR_FILTER_LENGHT 16
FIRFilter filtroPB;
//Coeficientes del filtro
static float FIR_IMPULSE_RESPONSE[FIR_FILTER_LENGHT] = { -0.0034586f,
		-0.0046729f, -0.0046977f, 0.0041916f, 0.0301665f, 0.0745832f,
		0.1277316f, 0.1715579f, 0.1885725f, 0.1715579f, 0.1277316f, 0.0745832f,
		0.0301665f, 0.0041916f, -0.0046977f, -0.0046729f };
float circularBuffer[FIR_FILTER_LENGHT] = { 0 };
uint16_t dato = 0;
/****************FILTRO*********************/

// ---------enum PULSADORES ----------/
typedef enum {
	P_None,					//Ninguno
	P_Start_Stop,
	P_GrabarECG,
	P_GrabarAudio
} Pulsadores;				//Los 3 pulsadores posibles. El 4to es de reset

// ---------enum PULSADORES ----------/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_DMA_Init(void);
void StartLecturaPulsadores(void const * argument);
void StartTarjetaSD(void const * argument);
void StartMainTask(void const * argument);
void StartManejoLEDs(void const * argument);
void StartRunningTask(void const * argument);

/* USER CODE BEGIN PFP */

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();

  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
 //OJO CON EL BUG ^^^^^
  //TIENE QUE IR ASI
  /*
   MX_GPIO_Init();
   MX_DMA_Init();
   MX_ADC1_Init();
   MX_SPI1_Init();
   MX_FATFS_Init();
   */
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of ADC1_Queue */
  osMessageQStaticDef(ADC1_Queue, 500, uint16_t, ADC1_QueueBuffer, &ADC1_QueueControlBlock);
  ADC1_QueueHandle = osMessageCreate(osMessageQ(ADC1_Queue), NULL);

  /* definition and creation of Pulsadores_Queue */
  osMessageQDef(Pulsadores_Queue, 1, uint16_t);
  Pulsadores_QueueHandle = osMessageCreate(osMessageQ(Pulsadores_Queue), NULL);

  /* definition and creation of SD_CMD_Queue */
  osMessageQDef(SD_CMD_Queue, 5, uint16_t);
  SD_CMD_QueueHandle = osMessageCreate(osMessageQ(SD_CMD_Queue), NULL);

  /* definition and creation of SD_STATUS_Queue */
  osMessageQDef(SD_STATUS_Queue, 5, uint16_t);
  SD_STATUS_QueueHandle = osMessageCreate(osMessageQ(SD_STATUS_Queue), NULL);

  /* definition and creation of ADC1_Queue2 */
  osMessageQStaticDef(ADC1_Queue2, 100, uint16_t, ADC1_Queue2Buffer, &ADC1_Queue2ControlBlock);
  ADC1_Queue2Handle = osMessageCreate(osMessageQ(ADC1_Queue2), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of LecturaPulsador */
  osThreadDef(LecturaPulsador, StartLecturaPulsadores, osPriorityLow, 0, 128);
  LecturaPulsadorHandle = osThreadCreate(osThread(LecturaPulsador), NULL);

  /* definition and creation of TarjetaSD */
  osThreadDef(TarjetaSD, StartTarjetaSD, osPriorityHigh, 0, 5000);
  TarjetaSDHandle = osThreadCreate(osThread(TarjetaSD), NULL);

  /* definition and creation of MainTask */
  osThreadDef(MainTask, StartMainTask, osPriorityNormal, 0, 512);
  MainTaskHandle = osThreadCreate(osThread(MainTask), NULL);

  /* definition and creation of ManejoLEDs */
  osThreadDef(ManejoLEDs, StartManejoLEDs, osPriorityLow, 0, 256);
  ManejoLEDsHandle = osThreadCreate(osThread(ManejoLEDs), NULL);

  /* definition and creation of RunningTask */
  osThreadDef(RunningTask, StartRunningTask, osPriorityLow, 0, 128);
  RunningTaskHandle = osThreadCreate(osThread(RunningTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

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
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  HAL_GPIO_WritePin(LED_Status_GPIO_Port, LED_Status_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SD_CS_Pin|LED3_Pin|LED2_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Status_Pin */
  GPIO_InitStruct.Pin = LED_Status_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_Status_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_CS_Pin LED3_Pin LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin|LED3_Pin|LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SW_1_Pin SW_2_Pin SW_3_Pin */
  GPIO_InitStruct.Pin = SW_1_Pin|SW_2_Pin|SW_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* ---------------------- RUTINA DE INTERRUPCIÓN ADC ---------------------*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {

	/*
	 * LOS DATOS DEL ADC SE GRABAN MEDIANTE DMA (DIRECT MEMORY ACESS)
	 * EL MICRO LO HACE EN SEGUNDO PLANO AUTOMATICAMENTE
	 * No se porqué al agregar DMA con 2 canales tuve que duplicar la frecuencia del
	 * APB2clock de 21 a 42 MHz, porque sino me muestreaba a la mitad de velocidad que yo queria
	 * 5303 samples/s.
	 * */

	if (grabandoAudio && !listoADC_Audio) {
		//datosDMA_ADC[1] contiene el valor del canal 2 del ADC (Pin A1)
		osMessagePut(ADC1_QueueHandle, datosDMA_ADC[1], 0); //Lo coloco en la cola de mensajes correspondiente al muestreo del audio
		samplesAudio_count++;
	}
	//Se muestrea el ECG 4 veces mas lento que el audio
	//datosDMA_ADC[0] contiene el valor del canal 1 del ADC (Pin A0)
	if (grabandoECG && (contadorIT_ADC % 4 == 0) && !listoADC_ECG){
		osMessagePut(ADC1_Queue2Handle, datosDMA_ADC[0], 0);  //Lo coloco en la cola de mensajes correspondiente al muestreo del ECG
		samplesECG_count++;
	}
	contadorIT_ADC++;
	//osMessagePut(ADC1_QueueHandle, contadorTest++, 0);// Prueba para ver si se graban todos los valores
	/*If continuousconversion mode is DISABLED uncomment below*/

}
/* ---------------------- RUTINA DE INTERRUPCIÓN ADC ---------------------*/
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartLecturaPulsadores */
/**
 * @brief  Funcion que implementa la tarea encargada de leer los pulsadores.
 * Esta misma le envia una cola de mensajes a la tarea Main con el valor correspondiente
 * al pulsador presionado
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLecturaPulsadores */
void StartLecturaPulsadores(void const * argument)
{
  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	Pulsadores teclaPulsada = P_None;
	for (;;)

	{

		if (HAL_GPIO_ReadPin(SW_1_GPIO_Port, SW_1_Pin) == GPIO_PIN_RESET) {
			teclaPulsada = P_Start_Stop;
			osMessagePut(Pulsadores_QueueHandle, teclaPulsada, 100);
			osDelay(250);

		}
		if (HAL_GPIO_ReadPin(SW_2_GPIO_Port, SW_2_Pin) == GPIO_PIN_RESET) {
			teclaPulsada = P_GrabarECG;
			osMessagePut(Pulsadores_QueueHandle, teclaPulsada, 100);
			osDelay(250);
		}
		if (HAL_GPIO_ReadPin(SW_3_GPIO_Port, SW_3_Pin) == GPIO_PIN_RESET) {
			teclaPulsada = P_GrabarAudio;
			osMessagePut(Pulsadores_QueueHandle, teclaPulsada, 100);
			osDelay(250);
		}

		osDelay(50);

	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTarjetaSD */
/**
 * @brief Funcion que implementa la tarea TarjetaSD
 * Originalmente esta tarea se iba a encargar únicamente de leer y escribir datos en la tarjeta SD
 * pero por practicidad también se encarga de filtrar el archivo de audio y de generar el .WAV
 * Recibe mensajes de la main task con los comandos que le indican que hacer, y ademas recibe 2 colas
 * de mensajes de la rutina de interrupción del ADC, con los datos tanto del muestreo del ECG y del audio
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTarjetaSD */
void StartTarjetaSD(void const * argument)
{
  /* USER CODE BEGIN StartTarjetaSD */
	/*-------------CREACION DE VARIABLES DE LA SD TASK -----------------*/
	ComandosSD comandoMainTask = CMD_Nada;//Comandos que envia por mensaje la mainTask
	uint16_t datoADCAudio = 0;
	uint16_t datoADC_ECG = 0;

	/* ------- ESTADOS DE LA TAREA SD -----*/
	//Estas variables se setean con comandos de la main task
	bool AlmacenarADCAudio = false;
	bool AlmacenarADC_ECG = false;
	bool errorSD = false;
	bool stop = false;
	bool filtrarAudio = false;
	bool crearWAV = false;
	/* ------- ESTADOS DE LA TAREA SD -----*/

	/* PARA ARMAR LOS NOMBRES DE LOS ARCHIVOS*/
	static const char *wav = ".wav";
	static const char *csv = ".csv";
	static const char *filtrada = "%i_filt"; //El %i sirve para que la funcion sprintf lo reemplaze por un numero
	static const char *grabacion = "%i_unf";
	static const char *ECG = "%i_ECG";

	char nombreArchivo1[24] = { 0 }; //Archivo1 y Archivo2 son para Audio
	char nombreArchivo2[24] = { 0 };
	char nombreArchivoECG[24] = { 0 };
	/* PARA ARMAR LOS NOMBRES DE LOS ARCHIVOS*/

	FIL Archivo1;  //Estructura con los datos del archivo 1
	FIL Archivo2;  //Estructura con los datos del archivo 2
	FIL ArchivoECG;  //Estructura con los datos del archivo ECG
	FATFS fs; //file system

	/*-------------CREACION DE VARIABLES DE LA SD TASK -----------------*/

	/*-------------SE EJECUTA UNA SOLA VEZ ANTES DEL LOOP -----------------*/
	//Intento montar la tarjeta SD antes de entrar en el loop
	if (f_mount(&fs, "", 1) != FR_OK) {
		errorSD = true;
		osMessagePut(SD_STATUS_QueueHandle, STAT_Error, 0); //Envio mensaje de error a la main Task
	} else {
		f_mount(NULL, "", 1); //Desmonto la SD
		osDelay(200);
		osMessagePut(SD_STATUS_QueueHandle, STAT_Listo, 0); // esta okey

	}
	osDelay(100);
	/*-------------SE EJECUTA UNA SOLA VEZ ANTES DEL LOOP -----------------*/
	/* Infinite loop */
	for (;;) {

		//------------COMANDOS PROVENIENTES DE LA MAIN TASK----------------------------//
		if (osMessageWaiting(SD_CMD_QueueHandle) > 0) { //Si hay un mensaje nuevo proveniente de la main task
			comandoMainTask = osMessageGet(SD_CMD_QueueHandle, 0).value.v;

			switch (comandoMainTask) {

			case CMD_Stop:
				stop = true;
				break;
			case CMD_GrabarADC_Audio: //La main task me ordena que comience a almacenar Audio proveniente del ADC

				/*-----------Monto la tarjeta SD--------------------*/
				if (f_mount(&fs, "", 1) != FR_OK) {
					errorSD = true;
					osMessagePut(SD_STATUS_QueueHandle, STAT_Error, 0); //Envio mensaje de error a la main Task
				}
				/*-----------Monto la tarjeta SD--------------------*/
				/*-----------CREO EL ARCHIVO PARA ALMACENAR LOS DATOS DEL ADC SIN FILTRAR--------------------*/
				memset(nombreArchivo1, 0, sizeof(nombreArchivo1));
				memset(nombreArchivo2, 0, sizeof(nombreArchivo2));
				sprintf(nombreArchivo1, grabacion, numGrabacionADC);
				strcat(nombreArchivo1, csv);

				/*-----------CREO EL ARCHIVO PARA ALMACENAR LOS DATOS DEL ADC SIN FILTRAR--------------------*/
				if (f_open(&Archivo1, nombreArchivo1,
						FA_CREATE_ALWAYS | FA_READ | FA_WRITE) != FR_OK) {
					errorSD = true;
					osMessagePut(SD_STATUS_QueueHandle, STAT_Error, 0); //Envio mensaje de error a la main Task
				} else {
					AlmacenarADCAudio = true;
					osMessagePut(SD_STATUS_QueueHandle, STAT_Grabando_Audio, 0); //Le envio mensaje a la main diciendo que comienza la grabacion
					HAL_ADC_Start_DMA(&hadc1, datosDMA_ADC, 2); //Arranco al ADC por DMA
				}

				break;
			case CMD_GrabarADC_ECG:  //La main me ordena que comience a almacenar ECG proveniente del ADC

				/*-----------CREO EL ARCHIVO PARA ALMACENAR LOS DATOS DEL ADC DEL ECG--------------------*/
				memset(nombreArchivoECG, 0, sizeof(nombreArchivoECG));
				sprintf(nombreArchivoECG, ECG, numGrabacionADC);
				strcat(nombreArchivoECG, csv);
				/*-----------CREO EL ARCHIVO PARA ALMACENAR LOS DATOS DEL ADC DEL ECG--------------------*/

				if (!AlmacenarADCAudio) { //Si tambien estoy grabando audio encontces la tarjeta SD ya esta montada
					/*-----------Monto la tarjeta SD--------------------*/
					if (f_mount(&fs, "", 1) != FR_OK) {
						errorSD = true;
						osMessagePut(SD_STATUS_QueueHandle, STAT_Error, 0); //Envio mensaje de error a la main Task
					}
					/*-----------Monto la tarjeta SD--------------------*/
				}

				if (f_open(&ArchivoECG, nombreArchivoECG,
						FA_CREATE_ALWAYS | FA_READ | FA_WRITE) != FR_OK) {
					errorSD = true;
					osMessagePut(SD_STATUS_QueueHandle, STAT_Error, 0); //Envio mensaje de error a la main Task
				} else {
					if (!AlmacenarADCAudio) HAL_ADC_Start_DMA(&hadc1, datosDMA_ADC, 2); //Si ya estoy almacenando audio, no hace falta arrancar ADC

					osMessagePut(SD_STATUS_QueueHandle, STAT_Grabando_ECG, 0); //Envio mensaje a la main diciendo que estoy grabando ECG
					AlmacenarADC_ECG = true;
				}

				break;

			case CMD_Filtrado_Audio:

				filtrarAudio = true;

				break;
			case CMD_ArchivoWav:
				crearWAV = true;
				break;

			default:
				break;
			}
		}

		//------------COMANDOS PROVENIENTES DE LA MAIN TASK----------------------------//

		if ((AlmacenarADCAudio || AlmacenarADC_ECG) && !errorSD) {


			if (osMessageWaiting(ADC1_QueueHandle) > 0) { //Me fijo si la interrupcion del ADC me mando dato
				datoADCAudio =
						(uint16_t) osMessageGet(ADC1_QueueHandle, 0).value.v;
				f_printf(&Archivo1, "%u,", datoADCAudio); // Voy grabando los datos de audio en el archivo
			}else
			if (osMessageWaiting(ADC1_Queue2Handle) > 0) { //Me fijo si la interrupcion del ADC me mando dato
				datoADC_ECG =
						(uint16_t) osMessageGet(ADC1_Queue2Handle, 0).value.v;
				f_printf(&ArchivoECG, "%u,", datoADC_ECG); // Voy grabando los datos de ECG en el archivo
			}else
			{

			if (stop) //La main task me puede detener por pulsador o porque ya se registraron los datos necesarios
			{

				stop = false;
				if (AlmacenarADCAudio)
					f_close(&Archivo1); //cerrar archivo
				osDelay(100);
				if (AlmacenarADC_ECG)
				{
					f_lseek(&ArchivoECG, f_tell(&ArchivoECG) -1 );  //Borro la ultima ','
					f_putc('\0',&ArchivoECG );						//Borro la ultima ','
					osDelay(10);
					f_close(&ArchivoECG); //cerrar archivo
				}
				osDelay(100);
				AlmacenarADCAudio = false;
				AlmacenarADC_ECG = false;
				osMessagePut(SD_STATUS_QueueHandle, STAT_Listo, 0); //Le digo a la Main que ya termine de almacenar
			}
			osDelay(5);
			}

		}

		if (filtrarAudio && !errorSD) {
			FIRFilter_Init(&filtroPB, FIR_IMPULSE_RESPONSE, circularBuffer,
			FIR_FILTER_LENGHT); 		//inicio el filtro

			sprintf(nombreArchivo2, filtrada, numGrabacionADC);
			strcat(nombreArchivo2, csv);

			if (f_open(&Archivo2, nombreArchivo2, //Creo el archivo para almacenar samplesAudio_Total filtradas
					FA_CREATE_ALWAYS | FA_READ | FA_WRITE) != FR_OK) {
				errorSD = true;
				osMessagePut(SD_STATUS_QueueHandle, STAT_Error, 0); //Envio mensaje de error a la main Task
			}

			osDelay(100);

			if (f_open(&Archivo1, nombreArchivo1, FA_READ) != FR_OK) {//Abro el archivo de las samplesAudio_Total sin filtro
				errorSD = true;
				osMessagePut(SD_STATUS_QueueHandle, STAT_Error, 0); //Envio mensaje de error a la main Task
			} else {
				osMessagePut(SD_STATUS_QueueHandle, STAT_Filtrando_Audio, 0);
				char readBuffer[4] = { 0 };	//buffer para leer 4 char que son los digitos de un dato int
				char writeBuffer[6] = { 0 }; //buffer para escribir en char un numero int
				float number = 0.0f;
				int intNumber = 0;
				int i = 0;
				UINT bytesLeidos = 0;
				BYTE buffer[1]; // array de 1, es decir, un solo caracter
				FRESULT resultado;
				/*
				 * A continuación voy a leer un archivo .csv quiere decir que todos los datos estan escritos como cadena
				 * de caracteres separados por coma, por ejemplo 1234,5678,1234,3423.... etc
				 * es por eso que debo leer los 4 caracteres del numero y formar un int con ellos
				 * si leo una coma ',' quiere decir que ya lei el numero completo y debo pasar al siguiente
				 * */
				int contadorDelay = 0; // contador para agregar un osDelay y que el sistema no se cuelgue
				for (;;) { //Bucle que lee de un archivo csv, aplica el filtro y escribe en el otro

					resultado = f_read(&Archivo1, buffer, sizeof buffer,
							&bytesLeidos); //Leo  un char
					if (bytesLeidos == 0)
						break; /* error or eof */

					switch (buffer[0]) {
					case ',': //Si el char es una coma quiere decir que ya lei el numero completo
						i = 0;
						number = atof(readBuffer); //paso de una cadena de char a un float
						FIRFilter_Update(&filtroPB, number); // Actualizo el filtro FIR con el float
						intNumber = (int) filtroPB.out; //El nuevo valor es la salida del filtro FIR
						if (intNumber < 0){ //Por si el filtro devuelve valores negativos
							//Salteo numeros negativos
						} else
						{
						sprintf(writeBuffer, "%04d,", intNumber); //Escribo una cadena de char a partir de un int
						f_puts(writeBuffer, &Archivo2);
						}
						memset(readBuffer, 0, sizeof(readBuffer)); //Vacio los buffer
						memset(writeBuffer, 0, sizeof(writeBuffer)); //Vacio los buffer
						contadorDelay++;

						break;
					default: //Si el char no es una coma voy almacenando los digitos del numero en un array
						readBuffer[i++] = buffer[0];
						if (i > strlen(readBuffer))
							i = 0;
						break;

					}
					if (contadorDelay >= 20) //Delay cada x datos escritos para que se ejecuten otras tareas
							{
						contadorDelay = 0;
						osDelay(1);
					}

				}
				f_close(&Archivo1);
				f_close(&Archivo2);
				filtrarAudio = false;
				f_mount(NULL, "", 1); //Desmonto la SD
				osMessagePut(SD_STATUS_QueueHandle, STAT_Listo, 0);

			}

		}

		if (crearWAV && !errorSD) {
			//Creo  un archivo .wav a partir de un archivo csv con esta funcion
			memset(nombreArchivo2, 0, sizeof(nombreArchivo2));
			sprintf(nombreArchivo2, filtrada, numGrabacionADC);
			strcat(nombreArchivo2, wav);

			memset(nombreArchivo1, 0, sizeof(nombreArchivo1));
			sprintf(nombreArchivo1, filtrada, numGrabacionADC);
			strcat(nombreArchivo1, csv);

			if (f_mount(&fs, "", 1) != FR_OK) {
				errorSD = true;
				osMessagePut(SD_STATUS_QueueHandle, STAT_Error, 0); //Envio mensaje de error a la main Task
			}

			if (f_open(&Archivo2, nombreArchivo2, //Creo el archivo para almacenar samplesAudio_Total filtradas
					FA_CREATE_ALWAYS | FA_READ | FA_WRITE) != FR_OK) {
				errorSD = true;
				osMessagePut(SD_STATUS_QueueHandle, STAT_Error, 0); //Envio mensaje de error a la main Task
			}

			osDelay(100);
			if (f_open(&Archivo1, nombreArchivo1, FA_READ) != FR_OK) {//Abro el archivo de las samplesAudio_Total sin filtro
				errorSD = true;
				osMessagePut(SD_STATUS_QueueHandle, STAT_Error, 0); //Envio mensaje de error a la main Task
			} else {
				osMessagePut(SD_STATUS_QueueHandle, STAT_Creando_WAV, 0);

				uint16_t num_channels;
				uint16_t bytes_per_sample;
				uint32_t byte_rate;
				uint32_t samplesCount = 0;
				UINT bytesWritten;
				num_channels = 1; /* monoaural */
				bytes_per_sample = 2; //16 bits
				byte_rate = SAMPLE_RATE_AUDIO * num_channels * bytes_per_sample;

				/* --------------------- ESCRIBO TODO EL HEADER DEL FORMATO WAV ---------------- */
				/* write RIFF header */

				//fwrite("RIFF", 1, 4, Archivo2);
				f_write(&Archivo2, "RIFF", 4, &bytesWritten);
				write_little_endian(
						36
								+ bytes_per_sample * samplesAudio_Total
										* num_channels, 4, &Archivo2);
				//fwrite("WAVE", 1, 4, Archivo2);
				f_write(&Archivo2, "WAVE", 4, &bytesWritten);
				/* write fmt  subchunk */
				//fwrite("fmt ", 1, 4, Archivo2);
				f_write(&Archivo2, "fmt ", 4, &bytesWritten);
				write_little_endian(16, 4, &Archivo2); /* SubChunk1Size is 16 */
				write_little_endian(1, 2, &Archivo2); /* PCM is format 1 */
				write_little_endian(num_channels, 2, &Archivo2);
				write_little_endian(SAMPLE_RATE_AUDIO, 4, &Archivo2);
				write_little_endian(byte_rate, 4, &Archivo2);
				write_little_endian(num_channels * bytes_per_sample, 2,
						&Archivo2); /* block align */
				write_little_endian(8 * bytes_per_sample, 2, &Archivo2); /* bits/sample */
				/* --------------------- ESCRIBO TODO EL HEADER DEL FORMATO WAV ---------------- 	 */
				/* write data subchunk */
				//fwrite("data", 1, 4, Archivo2);
				f_write(&Archivo2, "data", 4, &bytesWritten);
				osDelay(100);

				char readBuffer[4] = { 0 };
				int intNumber = 0;
				int normalizedNumber;
				int i = 0;
				UINT bytesLeidos = 0;
				BYTE buffer[1]; // array de 1, es decir, un solo caracter

				for (;;) {
					f_read(&Archivo1, buffer, sizeof buffer, &bytesLeidos); //Leo  un char
					if (bytesLeidos == 0)
						break; /* error or eof */

					switch (buffer[0]) {
					case ',': //Si el char es una coma quiere decir que ya lei el numero completo
						i = 0;
						intNumber = atoi(readBuffer);
						//Escalo el valor del adc (0 a 4095) a valores int16_t que van desde -32768 a 32767
						normalizedNumber = (((65535) / (4095)) * intNumber)
								- 32768;

						//if(normalizedNumber != 0) //Si el wav arranca con datos 0 se chotea

						write_little_endian((int16_t) (normalizedNumber),
								bytes_per_sample, &Archivo2);

						memset(readBuffer, 0, sizeof(readBuffer));
						samplesCount++;
						break;
					default: //Si el char no es una coma voy almacenando los digitos del numero en un array
						readBuffer[i++] = buffer[0];
						if (i > sizeof(readBuffer))
							i = 0;
						break;

					}
					if (samplesCount % 20 == 0) //delay para que se ejecuten otras tareas
							{
						osDelay(1);
					}
					if (samplesCount >= samplesAudio_Total)
						break;
				}
				f_close(&Archivo1);
				f_close(&Archivo2);

			}
			crearWAV = false;
			f_mount(NULL, "", 1); //Desmonto la SD
			osMessagePut(SD_STATUS_QueueHandle, STAT_Listo, 0);

		}

		if (!AlmacenarADCAudio && !AlmacenarADC_ECG) {
			osDelay(10);
		}

	}
  /* USER CODE END StartTarjetaSD */
}

/* USER CODE BEGIN Header_StartMainTask */
/**
 * @brief Function implementing the MainTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartMainTask */
void StartMainTask(void const * argument)
{
  /* USER CODE BEGIN StartMainTask */

	/*-------------CREACION DE VARIABLES DE LA MAIN TASK */
	Pulsadores teclaPulsada = P_None;
	EstadoSD estadoSDTask = STAT_Listo;
	bool listoSD = false;
	/*-------------CREACION DE VARIABLES DE LA MAIN TASK */

	/* Infinite loop */
	for (;;) {

		/*-----------------PULSADORES PRESIONADOS ---------------------*/
		if (osMessageWaiting(Pulsadores_QueueHandle) > 0) {
			teclaPulsada = osMessageGet(Pulsadores_QueueHandle, 0).value.v;

			switch (teclaPulsada) {
			case P_Start_Stop:

				if (!error && ready && !stop)
				{
					start = true;
				} else
				{
					stop = true;
				}


				break;
			case P_GrabarECG:
				if (ready)
					grabarECG = !grabarECG;

				break;
			case P_GrabarAudio:
				if (ready)
					grabarAudio = !grabarAudio;

				break;
			default:
				break;
			}

		}
		/*-----------------PULSADORES PRESIONADOS ---------------------*/

		/*-----------------Comunicacion con la tarea SD ---------------------*/
		if (osMessageWaiting(SD_STATUS_QueueHandle) > 0) {
			estadoSDTask = osMessageGet(SD_STATUS_QueueHandle, 0).value.v;

			switch (estadoSDTask) {
			case STAT_Listo:
				listoSD = true;
				break;
			case STAT_Error:
				error = true;
				break;
			case STAT_Grabando_Audio:
				grabandoAudio = true;
				break;
			case STAT_Filtrando_Audio:
				filtrandoAudio = true;
				break;
			case STAT_Creando_WAV:
				creandoWAV = true;
				break;

			case STAT_Grabando_ECG:
				grabandoECG = true;
				break;

			default:
				break;
			}

		}
		/*-----------------PULSADORES PRESIONADOS ---------------------*/

		/*----------------- LOGICA DEL PROGRAMA ---------------------*/
		if (start && ready) {
			start = false;
			ready = false;

			if (grabarAudio || grabarECG)

			{
				listoSD = false;
				numGrabacionADC++;
				if (grabarAudio) {
					samplesAudio_count = 0;
					osMessagePut(SD_CMD_QueueHandle, CMD_GrabarADC_Audio, 0);
				} else
					listoADC_Audio = true;
				if (grabarECG) {
					samplesECG_count = 0;
					osMessagePut(SD_CMD_QueueHandle, CMD_GrabarADC_ECG, 0);

				} else
					listoADC_ECG = true;
			}

		}

		if (samplesAudio_count >= samplesAudio_Total || stop) {

			listoADC_Audio = true;
			samplesAudio_count = 0;
		}

		if (samplesECG_count >= samplesECG_Total || stop) {
			stop=false;
			listoADC_ECG = true;
			samplesECG_count = 0;
		}

		if (listoADC_Audio && listoADC_ECG) {
			HAL_ADC_Stop_DMA(&hadc1);
			contadorIT_ADC = 0;
			listoADC_Audio = false;
			listoADC_ECG = false;
			osMessagePut(SD_CMD_QueueHandle, CMD_Stop, 0);
			osDelay(10);

		}

		if (grabandoECG && listoSD) {

			grabandoECG = false;
			if(!grabandoAudio)
			{

				listoSD = false;
				ready = true;
			}
		}

		if (grabandoAudio && listoSD) {
			listoSD = false;
			grabandoAudio = false;
			osMessagePut(SD_CMD_QueueHandle, CMD_Filtrado_Audio, 0);

		}
		if (filtrandoAudio && listoSD) {
			listoSD = false;
			filtrandoAudio = false;
			osMessagePut(SD_CMD_QueueHandle, CMD_ArchivoWav, 0);

		}

		if (creandoWAV && listoSD)

		{
			listoSD = false;
			creandoWAV = false;
			ready = true;

		}

		osDelay(50);
	}
  /* USER CODE END StartMainTask */
}

/* USER CODE BEGIN Header_StartManejoLEDs */
/**
 * @brief Function implementing the ManejoLEDs thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartManejoLEDs */
void StartManejoLEDs(void const * argument)
{
  /* USER CODE BEGIN StartManejoLEDs */
	/* Infinite loop */
	for (;;) {

		if (!error) {
			if (ready) // Si no estoy grabando
			{
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, grabarECG);
				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, grabarAudio);

			} else //Si estoy grabando
			{
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
				if (grabarECG)
					HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
				if (grabandoAudio)
					HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
			}
		} else {
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin); // PARPADEA LED 1 CON ERROR
		}

		osDelay(500);

	}
  /* USER CODE END StartManejoLEDs */
}

/* USER CODE BEGIN Header_StartRunningTask */
/**
 * @brief Function implementing the RunningTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartRunningTask */
void StartRunningTask(void const * argument)
{
  /* USER CODE BEGIN StartRunningTask */
	/* Infinite loop */
	for (;;) {
		HAL_GPIO_TogglePin(LED_Status_GPIO_Port, LED_Status_Pin);
		osDelay(250);

	}
  /* USER CODE END StartRunningTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

