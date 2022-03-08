/* make_wav.h
 * Fri Jun 18 17:06:02 PDT 2010 Kevin Karplus
 */

#ifndef MAKE_WAV_H
#define MAKE_WAV_H
#include "stdint-gcc.h"
void write_wav(char * filename, uint32_t num_samples, uint16_t * data, uint16_t s_rate);
    /* open a file named filename, write signed 16-bit values as a
        monoaural WAV file at the specified sampling rate
        and close the file
    */
void write_wav_from_csv(char *sourceName, char *destName,
		uint32_t num_samples, uint16_t s_rate);
#endif
