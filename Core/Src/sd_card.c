/*
 * audio_sd.c
 * Korjattu versio: Header kirjoitetaan heti alussa
 */

#include "sd_card.h"
#include "stdio.h"
#include "fatfs.h"
#include <string.h> // memcpy varten

// Alustetaan header valmiiksi 44 tavun pituiseksi
static uint8_t wav_file_header[44] = {
    0x52, 0x49, 0x46, 0x46, 0x00, 0x00, 0x00, 0x00, 0x57, 0x41, 0x56, 0x45, 0x66, 0x6d,
    0x74, 0x20, 0x10, 0x00, 0x00, 0x00, 0x01, 0x00, 0x02, 0x00, 0x80, 0xbb, 0x00, 0x00,
    0x00, 0xee, 0x02, 0x00, 0x04, 0x00, 0x10, 0x00, 0x64, 0x61, 0x74, 0x61, 0x00, 0x00,
    0x00, 0x00
};

static FRESULT sd_result;
static FATFS sdCard;
static FIL wavFile;
static uint32_t wav_data_size = 0; // Seurataan pelkän audiodatan määrää

int sd_card_init(){
    // Option 1 = Mount immediately (tämä aiheuttaa virheen 3 jos kortti ei vastaa)
    sd_result = f_mount(&sdCard, SDPath, 0);

    if(sd_result != FR_OK){
        printf("SD Card Mount Failed. Error: %d\n", sd_result);
        return 1;
    }
    else {
        printf("SD Card Mounted Successfully\n");
    }
    return 0;
}

void start_recording(uint32_t frequency)
{
    static char file_name[] = "w_000.wav";
    static uint8_t file_counter = 0;
    UINT bytes_written;

    // 1. Päivitetään headerin taajuustiedot
    uint32_t byte_rate = frequency * 2 * 2; // 16bit stereo

    wav_file_header[24] = (uint8_t)frequency;
    wav_file_header[25] = (uint8_t)(frequency >> 8);
    wav_file_header[26] = (uint8_t)(frequency >> 16);
    wav_file_header[27] = (uint8_t)(frequency >> 24);

    wav_file_header[28] = (uint8_t)byte_rate;
    wav_file_header[29] = (uint8_t)(byte_rate >> 8);
    wav_file_header[30] = (uint8_t)(byte_rate >> 16);
    wav_file_header[31] = (uint8_t)(byte_rate >> 24);

    // 2. Luodaan tiedostonimi
    int temp_counter = file_counter;
    file_name[4] = (temp_counter % 10) + '0';
    temp_counter /= 10;
    file_name[3] = (temp_counter % 10) + '0';
    temp_counter /= 10;
    file_name[2] = (temp_counter % 10) + '0';
    file_counter++;

    printf("Opening file: %s\n", file_name);

    // 3. Luodaan tiedosto
    sd_result = f_open(&wavFile, file_name, FA_WRITE | FA_CREATE_ALWAYS);
    if(sd_result != FR_OK)
    {
        printf("Error creating file: %d\n", sd_result);
        return; // Älä jää jumiin while(1) luuppiin täällä
    }

    // 4. KIRJOITETAAN HEADER HETI!
    // Tämä on tärkeä muutos. Varaamme tiedoston alusta 44 tavua headerille.
    sd_result = f_write(&wavFile, wav_file_header, 44, &bytes_written);
    if(sd_result != FR_OK) {
         printf("Error writing header: %d\n", sd_result);
    }

    // Nollataan laskuri
    wav_data_size = 0;
}

void write2wave_file(uint8_t *data, uint16_t data_size)
{
    UINT bytes_written;

    // Kirjoitetaan VAIN dataa. Header on jo hoidettu.
    sd_result = f_write(&wavFile, data, data_size, &bytes_written);

    if(sd_result != FR_OK)
    {
        printf("Write error: %d\n", sd_result);
    }

    wav_data_size += data_size;
}

void stop_recording()
{
    UINT bytes_written;
    FRESULT res;

    // 1. Lasketaan lopulliset koot
    uint32_t riff_chunk_size = wav_data_size + 36;
    uint32_t data_chunk_size = wav_data_size;

    // 2. Päivitetään header-taulukkoon (muistissa)
    wav_file_header[4] = (uint8_t)riff_chunk_size;
    wav_file_header[5] = (uint8_t)(riff_chunk_size >> 8);
    wav_file_header[6] = (uint8_t)(riff_chunk_size >> 16);
    wav_file_header[7] = (uint8_t)(riff_chunk_size >> 24);

    wav_file_header[40] = (uint8_t)data_chunk_size;
    wav_file_header[41] = (uint8_t)(data_chunk_size >> 8);
    wav_file_header[42] = (uint8_t)(data_chunk_size >> 16);
    wav_file_header[43] = (uint8_t)(data_chunk_size >> 24);

    // 3. Palataan tiedoston alkuun ja kirjoitetaan korjattu header
    res = f_lseek(&wavFile, 0);
    	if(res == FR_OK)
        {
    	    f_write(&wavFile, wav_file_header, 44, &bytes_written);
        }
        else
        {
            printf("Header update failed! Error: %d\n", res);
            // Emme pysäytä koodia tähän, vaan jatkamme sulkemiseen!
        }

        // 3. TÄMÄ ON TÄRKEIN: Sulje tiedosto, tapahtui mitä tahansa
    	res = f_close(&wavFile);

        if(res == FR_OK)
        {
    	    printf("File closed successfully. Size: %lu\n", wav_data_size);
        }
        else
        {
            printf("File close failed! Error: %d\n", res);
            // Jos tämä epäonnistuu, kortti on irti tai rikki
        }

}

