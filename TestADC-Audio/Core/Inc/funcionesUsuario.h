#ifndef __FUNCIONESUSUARIO_H
#define __FUNCIONESUSUARIO_H

#include "fatfs.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "string.h"
#include "fatfs_sd.h"
#include "stdbool.h"
#include "FIR.h"
#include "make_wav.h"

extern ADC_HandleTypeDef hadc1; //Definida en main.c
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


/**********************PROTOTIPO DE FUNCIONES ****************************/

void EjecutarSetup()  // Contiene el programa que se ejecuta antes del bucle infinito
{
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
}

void EjecutarLoop()
{
	/* Debido a que tantas mustras del ADC no entran en la RAM. Vamos grabando de a bloques del tamaño
			 * 	de adcBuff[] y cuando este se llena, volvamos todo al archivo de texto. la variable adcCount lleva el
			 * 	conteo de cuanto buffer se lleno
			 */

			// Oprimo el pulsador
			if (!start
					&& (HAL_GPIO_ReadPin(Pulsador_GPIO_Port, Pulsador_Pin)
							== GPIO_PIN_RESET)) {
				start = true;
				HAL_Delay(1000);
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
				HAL_ADC_Start_IT(&hadc1);   //ARRANCA EL ADC POR INTERRPUCION

			}

			if (adcCount1 >= adcBuff1Size) { //Si se llenó el buffer 1
				adcCount1 = 0;
				buff1 = false;
				buff2 = true;	//hago que el ADC comience a almacenar en el buff2

				for (int i = 0; i < adcBuff1Size; i++) {

					f_printf(&unfilteredData, "%04u,", adcBuff1[i]); //Guardamos en archivo el buffer 1
				}

			}

			if (adcCount2 >= adcBuff2Size) { //Si se llenó el buffer 2
				adcCount2 = 0;
				buff1 = true;	//hago que el ADC comience a almacenar en el buff1
				buff2 = false;

				for (int i = 0; i < adcBuff2Size; i++) {

					f_printf(&unfilteredData, "%04u,", adcBuff2[i]); //Guardamos en archivo el buffer 2
				}

			}

			if (samples_count >= muestras)// Si se obvtuvieron todas las muestras deseadas
					{

				HAL_ADC_Stop_IT(&hadc1); 				//Detenemos el ADC

				if (buff1 && adcCount1 > 0 && adcCount1 < adcBuff1Size) { //almacenamos lo que quedó del buff 1

					for (int i = 0; i < (adcCount1 - 1); i++) {

						f_printf(&unfilteredData, "%04u,", adcBuff1[i]);
					}
					f_printf(&unfilteredData, "%04u", adcBuff1[adcCount1 - 1]); // Ultima muestra sin coma
				}
				if (buff2 && adcCount2 > 0 && adcCount2 < adcBuff2Size) { //almacenamos lo que quedó del buff 2

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

				if (f_open(&filteredData, "filtered.csv",//Creo el archivo para almacenar muestras filtradas
						FA_CREATE_ALWAYS | FA_READ | FA_WRITE) != FR_OK) {
					memset(buffer, 0, strlen(buffer));
					sprintf(buffer, "ERROR abriendo filtered ");
					CDC_Transmit_FS((uint8_t*) buffer, strlen(buffer));
				}

				HAL_Delay(100);

				if (f_open(&unfilteredData, "unf.csv", FA_READ) != FR_OK) {	//Abro el archivo de las muestras sin filtro
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
						if (bytesLeidos == 0)
							break; /* error or eof */

						switch (buffer[0]) {
						case ',': //Si el char es una coma quiere decir que ya lei el numero completo
							i = 0;
							number = atof(readBuffer); //paso de una cadena de char a un float
							FIRFilter_Update(&filtroPB, number); // Actualizo el filtro FIR con el float
							intNumber = (int) filtroPB.out; //El nuevo valor es la salida del filtro FIR
							if (intNumber < 0)//Por si el filtro devuelve valores negativos
								intNumber = 0;
							sprintf(writeBuffer, "%04d,", intNumber); //Escribo una cadena de char a partir de un int
							f_puts(writeBuffer, &filteredData);
							memset(readBuffer, 0, strlen(readBuffer)); //Vacio los buffer
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
				write_wav_from_csv("filtered.csv", "filt.wav", muestras,
						sampleRate);
				sprintf(buffer, "Done filt");
				CDC_Transmit_FS((uint8_t*) buffer, strlen(buffer));
				memset(buffer, 0, strlen(buffer));
				HAL_Delay(200);

				f_mount(NULL, "", 1); //Desmonto la SD

				memset(buffer, 0, strlen(buffer));
				sprintf(buffer, "\n DONE");
				CDC_Transmit_FS((uint8_t*) buffer, strlen(buffer));
			}
}
#endif /* __FUNCIONESUSUARIO_H */
