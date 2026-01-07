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
static FIL testFile;
volatile int debug_error_code = 0;

int sd_card_init(){

	uint8_t file_name[] = "test.txt";
	uint8_t temp_number;
	uint8_t test_text[] = "Hello I am working";


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
	}

	sd_result = f_open(&testFile ,(void*)file_name, FA_WRITE|FA_CREATE_ALWAYS);
	if(sd_result != 0)
	{
		debug_error_code = 21;
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		HAL_Delay(10000);

		while(1);
	}
	else{
		debug_error_code = 20;
	}

	//writing to the file
	sd_result = f_write(&testFile, (void*)test_text, (UINT)sizeof(test_text), (UINT*)&temp_number);
	if(sd_result != 0)
	{
		debug_error_code = 31;
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		HAL_Delay(5000);
		return 1;
	}
	else
	{
		debug_error_code = 30;



	}
	f_close(&testFile);
	return 0;
}

