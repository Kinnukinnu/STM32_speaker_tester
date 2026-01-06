/*
 * audio_sd.c
 *
 *  Created on: 6 Jan 2026
 *      Author: Kinnu
 */

#include "sd_card.h"
#include "stdio.h"
#include "fatfs.h"


static FRESULT sd_result;
static FATFS sdCard;
volatile int debug_error_code = 0;

int sd_card_init(){

	//mounting card
	sd_result = f_mount(&sdCard,SDPath, 1);
	debug_error_code = (int)sd_result;// debugging the issue

	if(sd_result != 0){
		printf("error in mounting the sd-card: %d \n", sd_result);


		return 1;
	}
	else
	{
		printf("SD card successfully mounted\n");
		return 0;
	}

}

