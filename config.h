//Config file for ESP32 NES emulator Corn 2020
#pragma GCC optimize ("O2")
#pragma once 

//Setting FreeRTOS sheduler to 300Hz
#define CONFIG_FREERTOS_HZ 300

//LCD screen & video constants
#define LCD_TYPE_ILI 0
#define LCD_TYPE_ST  1
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240
#define NES_WIDTH 256
#define NES_HEIGHT 240
#define NES_VIS_LINES 224	//This number impacts SPI transfers to the LCD

//Define Display type ILI9341 (currently only ILI9341 works)
#define CONFIG_HW_LCD_TYPE LCD_TYPE_ILI

// NTSC timing defaults if PAL not defined
//#define PAL

//Define to skip menu
//#define SKIP_MENU   

// Sound Settings, Note: Do not assign anything else to pins 25 or 26, those are used by I2S
#define CONFIG_SOUND_ENABLED

//LCD SPI transfer clock rate. With short leads to the display it is possible to run the display at 80MHz however here you can opt to run it slower
#define LCD_SPI_CLOCK_RATE SPI_MASTER_FREQ_80M	//80MHz
//#define LCD_SPI_CLOCK_RATE SPI_MASTER_FREQ_40M	//40MHz

//If defined it allows selection in the menu to scale/stretch frame (X axis only) to full screen (with artifacts in scrolling games).
//It will also be slightly slower with stretch screen enabled due to more data needs to be sent over SPI to the LCD
//0 -> defaults to not stretched, 1 -> defaults to stretched 
#define FULL_SCREEN 1

//If defined then screen interpolation is bilinear instead of nerest neighborhor sampling (looks better but uses a little more CPU)
//This only works in CPU mode and in interlaced mode for the moment
//Speedy implementation idea from the guys hacking the game and watch console processing all 3 color componets at once almost like vector processing
#define BILINEAR

// If defined it will run the SPI transfer to the LCD as a separate task and the value indicates which CPU core 0(faster with volatiles) or 1(faster with OS_SEMAPHORE)
#define RUN_VIDEO_AS_TASK 0

// If defined it will use OS semafores (which seems slower!?) else use simple volatile variables to pass info between tasks
//#define USE_OS_SEMAPHORES 

// If defined rotate display 180 deg
#define ROTATE_LCD_180

// Use DMA to transfer to screen if defined else use CPU (DMA and CPU are about equally fast...)
//#define USE_SPI_DMA

// Turn on various debugging options
//#define NOFRENDO_DEBUG
//#define MEMORY_DEBUG
//#define MAPPER_DEBUG
//#define SHOW_SPI_TRANSFER_TIME	//in micro seconds
//#define SHOW_RENDER_VIDEO_TIME	//in micro seconds

//Define and it will directly boot the ROM from the list
//#define BOOT_ROM 0   

// LCD Settings using VSPI bus
// LCD pin mapping
#define CONFIG_HW_LCD_RESET_GPIO 0
#define CONFIG_HW_LCD_CS_GPIO 5
#define CONFIG_HW_LCD_DC_GPIO 2
#define CONFIG_HW_LCD_MOSI_GPIO 23
#define CONFIG_HW_LCD_CLK_GPIO 18
#define CONFIG_HW_LCD_MISO_GPIO 19
#define CONFIG_HW_LCD_BL_GPIO -1

// SD card pin mapping
#define CONFIG_SD_CARD
#define CONFIG_SD_CS 15
#define CONFIG_SD_MOSI 13
#define CONFIG_SD_SCK 14
#define CONFIG_SD_MISO 12

//----------------------------------
//Enable only one controller type!!!
//----------------------------------

// NES input pin mapping 
//       ___
//DATA  |o o| NC
//LATCH |o o| NC
//CLOCK |o o/ 3V3
//GND   |o_/
//
#define CONFIG_HW_CONTROLLER_NES	//Define to use NES controller
#define CONFIG_HW_NES_DAT 21    //    # MISO
#define CONFIG_HW_NES_LATCH 27  //    # CS
#define CONFIG_HW_NES_CLK 22    //    # CLK

// SNES input pin mapping 
//       _
//3V3   |o|
//CLOCK |o|
//LATCH |o|
//DATA  |o|
//      |-|
//NC    |o|
//NC    |o|
//GND   |o|
//       - 
//
//#define CONFIG_HW_CONTROLLER_SNES	//Define to use SNES controller
#define CONFIG_HW_SNES_DAT 21    //    # MISO
#define CONFIG_HW_SNES_LATCH 27  //    # CS
#define CONFIG_HW_SNES_CLK 22    //    # CLK

// PSX input pin mapping 
//#define CONFIG_HW_CONTROLLER_PSX	//Define to use PSX controller
#define CONFIG_HW_PSX_DAT 21	//    # DATA/MISO
#define CONFIG_HW_PSX_CMD 4		//    # CMD/MOSI
#define CONFIG_HW_PSX_ATT 27	//    # ATT
#define CONFIG_HW_PSX_CLK 22	//    # CLK

// GPIO pin mapping (raw button input)
//#define CONFIG_HW_CONTROLLER_GPIO	//Define to use GPIO pins on the ESP32
#define CONFIG_HW_GPIO_START 21
#define CONFIG_HW_GPIO_SELECT 17
#define CONFIG_HW_GPIO_UP 34
#define CONFIG_HW_GPIO_DOWN 33
#define CONFIG_HW_GPIO_LEFT 39
#define CONFIG_HW_GPIO_RIGHT 32
#define CONFIG_HW_GPIO_B 35
#define CONFIG_HW_GPIO_A 4
#define CONFIG_HW_GPIO_MENU 16	//Also referred to as Button 2
#define CONFIG_HW_GPIO_POWER 27	//Also referred to as Button 1
#define CONFIG_HW_GPIO_TURBO_A 1
#define CONFIG_HW_GPIO_TURBO_B 1
