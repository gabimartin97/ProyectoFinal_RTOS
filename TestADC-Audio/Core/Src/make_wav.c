/* make_wav.c
 * Creates a WAV file from an array of ints.
 * Output is monophonic, signed 16-bit samples
 * copyright
 * Fri Jun 18 16:36:23 PDT 2010 Kevin Karplus
 * Creative Commons license Attribution-NonCommercial
 *  http://creativecommons.org/licenses/by-nc/3.0/
 */

//#include "stdio.h"
//#include <assert.h>
#include "make_wav.h"
#include "fatfs.h"
#include "fatfs_sd.h"
#include "string.h"
void write_little_endian(int16_t word, int num_bytes, FIL *wav_file) {
	int16_t buf;
	UINT bytesWritten = 0;
	while (num_bytes > 0) {
		buf = word & 0xff;
		//fwrite(&buf, 1,1, wav_file);
		f_write(wav_file, &buf, 1, &bytesWritten);
		//f_printf(wav_file, "%u", buf);
		num_bytes--;
		word >>= 8;
	}
}


void write_wav_from_csv(char *sourceName, char *destName,
		uint32_t num_samples, uint16_t s_rate) {
	FIL src_file; //Archivo de origen
	FIL wav_file; //Archivo destino
	uint16_t sample_rate;
	uint16_t num_channels;
	uint16_t bytes_per_sample;
	uint32_t byte_rate;
	uint32_t samplesCount =0;
	//unsigned long i; /* counter for samples */
	UINT bytesWritten;
	num_channels = 1; /* monoaural */
	bytes_per_sample = 2; //16 bits

	if (s_rate <= 0)
		sample_rate = 44100;
	else
		sample_rate = s_rate;

	byte_rate = sample_rate * num_channels * bytes_per_sample;

	f_open(&src_file, sourceName, FA_READ);
	HAL_Delay(100);
	f_open(&wav_file, destName, FA_CREATE_ALWAYS | FA_READ | FA_WRITE);
	HAL_Delay(100);

	/* --------------------- ESCRIBO TODO EL HEADER DEL FORMATO WAV ---------------- */
	/* write RIFF header */

	//fwrite("RIFF", 1, 4, wav_file);
	f_write(&wav_file, "RIFF", 4, &bytesWritten);
	write_little_endian(36 + bytes_per_sample * num_samples * num_channels, 4,
			&wav_file);
	//fwrite("WAVE", 1, 4, wav_file);
	f_write(&wav_file, "WAVE", 4, &bytesWritten);
	/* write fmt  subchunk */
	//fwrite("fmt ", 1, 4, wav_file);
	f_write(&wav_file, "fmt ", 4, &bytesWritten);
	write_little_endian(16, 4, &wav_file); /* SubChunk1Size is 16 */
	write_little_endian(1, 2, &wav_file); /* PCM is format 1 */
	write_little_endian(num_channels, 2, &wav_file);
	write_little_endian(sample_rate, 4, &wav_file);
	write_little_endian(byte_rate, 4, &wav_file);
	write_little_endian(num_channels * bytes_per_sample, 2, &wav_file); /* block align */
	write_little_endian(8 * bytes_per_sample, 2, &wav_file); /* bits/sample */
	/* --------------------- ESCRIBO TODO EL HEADER DEL FORMATO WAV ---------------- 	 */
	/* write data subchunk */
	//fwrite("data", 1, 4, wav_file);
	f_write(&wav_file, "data", 4, &bytesWritten);

	char readBuffer[4] = { 0 };
	int intNumber = 0;
	int normalizedNumber;
	int i = 0;
	UINT bytesLeidos = 0;
	BYTE buffer[1]; // array de 1, es decir, un solo caracter

	for (;;) {
		f_read(&src_file, buffer, sizeof buffer,	&bytesLeidos);  //Leo  un char
		if (bytesLeidos == 0)
			break; /* error or eof */

		switch (buffer[0]) {
		case ',': //Si el char es una coma quiere decir que ya lei el numero completo
			i = 0;
			intNumber = atoi(readBuffer);
			//Escalo el valor del adc (0 a 4095) a valores int16_t que van desde -32768 a 32767
			normalizedNumber = (((65535)/(4095))*intNumber) - 32768;

			//if(normalizedNumber != 0) //Si el wav arranca con datos 0 se chotea

			write_little_endian((int16_t) (normalizedNumber), bytes_per_sample,
							&wav_file);

			memset(readBuffer, 0, strlen(readBuffer));
			samplesCount++;
			break;
		default: //Si el char no es una coma voy almacenando los digitos del numero en un array
			readBuffer[i++] = buffer[0];
			if (i > strlen(readBuffer))
				i = 0;
			break;

		}
		if (samplesCount >= num_samples)break;
	}
	f_close(&src_file);
	f_close(&wav_file);

}

