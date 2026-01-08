/*
 * audio_sd.h
 *
 *  Created on: 6 Jan 2026
 *      Author: Kinnu
 *
 */

#ifndef INC_SD_CARD_H_
#define INC_SD_CARD_H_

#define WAV_WRITE_SAMPLE_COUNT 2048

#include <stdint.h>
int sd_card_init();
void start_recording(uint32_t frequency);
void write2wave_file(uint8_t *data, uint16_t data_size);
void stop_recording();


#endif /* INC_SD_CARD_H_ */
