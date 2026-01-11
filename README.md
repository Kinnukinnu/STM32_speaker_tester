## Speaker calibration device

This is a work in progress embedded project to create a speaker calibration tool and an analyzer. The mcu creates a sinewave sweep that is recorded simultaneously to an SD card as a WAVE file. 
After analysing the recorded file, a corrective EQ curve will be provided. In-depth explanation in the documents folder.

Hardware
| Component name | Description |
|----------|-----------|
|STM32 NUCLEO-F411RE| MCU|
|Pmod I2S2 | CS5343 ADC, CS4344 DAC   |
|Pololu-2597 | SD Card breakout board      |
|Wires and Cables| Connections, USB, Audio      |


Development tools
- IDE: STM32cubeMX, STM32cubeIDE
- C language (HAL libraries)

Features
- Full-duplex I2S audio streaming
- DMA and double buffering
- SDIO and FATFS-format for SD card

Current state (JAN 2026)
- Test signal is successfully generated and result recorded to an SD card
- Audio recording is noisy (need to check if problem lies in hardware or firmware)
- I'm researching if it is possible to do the frecuency analysis with the MCU or is there a need for external software
