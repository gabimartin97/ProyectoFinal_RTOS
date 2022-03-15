/* make_wav.h
 * Fri Jun 18 17:06:02 PDT 2010 Kevin Karplus
 */

#ifndef MAKE_WAV_H
#define MAKE_WAV_H
#include "stdint-gcc.h"
#include "fatfs.h"
#include "fatfs_sd.h"

//Escribe un word de 16 bits en un archivo en formato little endian.
void write_little_endian(int16_t word, int num_bytes, FIL *wav_file);

//Crea un archivo wav de 16 bits a partir de un archivo csv
void write_wav_from_csv(char *sourceName, char *destName,
		uint32_t num_samples, uint16_t s_rate);
#endif
