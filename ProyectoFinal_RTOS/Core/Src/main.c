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

osThreadId LecturaPulsadorHandle;
osThreadId GuardarSDHandle;
osThreadId MainTaskHandle;
osThreadId ManejoLEDsHandle;
osThreadId RunningTaskHandle;
osMessageQId ADC_QueueHandle;
osMessageQId Pulsadores_QueueHandle;
/* USER CODE BEGIN PV */

// ---------ESTADOS DEL PROGRAMA----------/
static bool ready = true;
static bool start = false;
static bool grabarECG = false;
static bool grabarAudio = false;

static bool grabandoAudio = false;
static bool filtrandoAudio = false;
static bool error = false;
// ---------ESTADOS DEL PROGRAMA----------/

/*----------------------TARJETA SD ------------------------------------------*/

FATFS fs; //file system
FIL fil; //file
FRESULT fresult; //to store the result

UINT br, bw;  //file read/write count

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
uint32_t samples_count = 0; //Contador de muestras
uint16_t contadorTest = 0;
static const uint32_t muestras = recordingTime * sampleRate; //Cantidad de muestras totales que se deben adquirir
FIL unfilteredData;

/* PARA ARMAR LOS NOMBRES DE LOS ARCHIVOS*/
static const char *wav = ".wav";
static const char *csv = ".csv";
static const char *filtrada = "%i_filt"; //El %i sirve para que la funcion sprintf lo reemplaze por un numero
static const char *grabacion = "%i_unf";

char nombreArchivo1[24] = { 0 };
char nombreArchivo2[24] = { 0 };
int numGrabacionECG = 0;
int numGrabacionAudio = 0;

// ---------PULSADORES ----------/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
void StartLecturaPulsadores(void const * argument);
void StartGuardarSD(void const * argument);
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
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

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
  /* definition and creation of ADC_Queue */
  osMessageQDef(ADC_Queue, 1000, uint16_t);
  ADC_QueueHandle = osMessageCreate(osMessageQ(ADC_Queue), NULL);

  /* definition and creation of Pulsadores_Queue */
  osMessageQDef(Pulsadores_Queue, 1, uint16_t);
  Pulsadores_QueueHandle = osMessageCreate(osMessageQ(Pulsadores_Queue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of LecturaPulsador */
  osThreadDef(LecturaPulsador, StartLecturaPulsadores, osPriorityLow, 0, 128);
  LecturaPulsadorHandle = osThreadCreate(osThread(LecturaPulsador), NULL);

  /* definition and creation of GuardarSD */
  osThreadDef(GuardarSD, StartGuardarSD, osPriorityHigh, 0, 1024);
  GuardarSDHandle = osThreadCreate(osThread(GuardarSD), NULL);

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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

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

  /*Configure GPIO pin : Pulsador_Pin */
  GPIO_InitStruct.Pin = Pulsador_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Pulsador_GPIO_Port, &GPIO_InitStruct);

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
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {

	//Coloco el valor del ADC en la cola de mensajes para la tarea que almacena en la SD
	osMessagePut(ADC_QueueHandle, HAL_ADC_GetValue(&hadc1), 0);
	samples_count++;

	//osMessagePut(ADC_QueueHandle, contadorTest++, 0); Prueba para ver si se graban todos los valores
	/*If continuousconversion mode is DISABLED uncomment below*/
	if (samples_count < muestras)
	{
		HAL_ADC_Start_IT(&hadc1);
	}
	else
	{
		grabandoAudio = false;
		HAL_ADC_Stop_IT(&hadc1);
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartLecturaPulsadores */
/**
 * @brief  Function implementing the LecturaPulsador thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLecturaPulsadores */
void StartLecturaPulsadores(void const * argument)
{
  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;)

	{

		if (HAL_GPIO_ReadPin(SW_1_GPIO_Port, SW_1_Pin) == GPIO_PIN_RESET) {
			osMessagePut(Pulsadores_QueueHandle, 1, 100);
			osDelay(250);

		}
		if (HAL_GPIO_ReadPin(SW_2_GPIO_Port, SW_2_Pin) == GPIO_PIN_RESET) {
			osMessagePut(Pulsadores_QueueHandle, 2, 100);
			osDelay(250);
		}
		if (HAL_GPIO_ReadPin(SW_3_GPIO_Port, SW_3_Pin) == GPIO_PIN_RESET) {
			osMessagePut(Pulsadores_QueueHandle, 3, 100);
			osDelay(250);
		}

		osDelay(50);

	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartGuardarSD */
/**
 * @brief Function implementing the GuardarSD thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartGuardarSD */
void StartGuardarSD(void const * argument)
{
  /* USER CODE BEGIN StartGuardarSD */


	uint32_t escritos = 0;
	osEvent mensaje;
	uint32_t valor = 0; // es un valor de 32 bits porque no puedo dereferenciar el (*void)p que llega

	if (f_mount(&fs, "", 1) != FR_OK) {

		error = true; //Montaje de la tarjta SD fallidoç
		osThreadSuspend(GuardarSDHandle);
	} else
	{
		osDelay(200);
		f_open(&unfilteredData, "PRUEBA.csv", FA_CREATE_ALWAYS | FA_READ | FA_WRITE);

	}
	osDelay(100);

	/* Infinite loop */
	for (;;) {




		if (!grabandoAudio && (escritos >= muestras)) {
			//cerrar archivo
			f_close(&unfilteredData);
			osDelay(100);
			f_mount(NULL, "", 1); //Desmonto la SD
			ready = true;

		}
		mensaje = osMessageGet(ADC_QueueHandle, osWaitForever);

		if (mensaje.status == osEventMessage) {

			valor = mensaje.value.v;
			f_printf(&unfilteredData, "%u,", (uint16_t) valor);
			escritos++;


		}

	}
  /* USER CODE END StartGuardarSD */
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
	/* Infinite loop */
	for (;;) {

		/*-----------------PULSADORES PRESIONADOS ---------------------*/
		if (osMessageWaiting(Pulsadores_QueueHandle) > 0) {
			uint32_t pulsador = osMessageGet(Pulsadores_QueueHandle, 0).value.v;

			switch (pulsador) {
			case 1:
				if (ready) {
					start = !start;
				} else {
					ready = true;
				}

				break;
			case 2:
				if (ready)
					grabarECG = !grabarECG;

				break;
			case 3:
				if (ready)
					grabarAudio = !grabarAudio;

				break;
			default:
				break;
			}

			/*-----------------PULSADORES PRESIONADOS ---------------------*/
		}

		/*----------------- LOGICA DEL PROGRAMA ---------------------*/
		if (start && ready) {
			start = false;
			ready = false;

			if (grabarAudio)

			{

				numGrabacionAudio++;
				grabandoAudio = true;
				HAL_ADC_Start_IT(&hadc1); //Comienza a trabajar el ADC Audio por interrupcion

			}

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

		if(!error)
		{
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
		} else
		{
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

