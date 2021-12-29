# stm32f407vg_SD_CARD_SPI
 
Test project STM32F407VG + SD Card over SPI1.

Generate code in STM32CubeIDE.
SD card connected to SPI1, speed: 2,5MBit/s.
Functions:

1. Read/Write bloks. (Doesn't use FATFS functions).

2. Read/Write files.

3. Read files and folders in dir.

4. Also RTC added in function: Start_Blue_LED_Blink. Made changes in hardware part: Removed R26, connect to VBAT (pin 6) 3.3V battery, and LF cristal and capacitors.

Sourse: https://narodstream.ru/stm-urok-88-sd-spi-fatfs-chast-1/

![alt text](https://github.com/OlegDemk/stm32f407vg_SD_CARD_SPI/blob/main/CD.png)

![alt text](https://github.com/OlegDemk/stm32f407vg_SD_CARD_SPI/blob/main/screenshot_3.png)
