/*
   This code generates an effect that should pass the 'fancy graphics' qualification
   as set in the comment in the spi_master code.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_system.h>
#include <math.h>
#include "pretty_effect.h"
#include "driver/gpio.h"
#include "charPixels.h"
#include "esp_deep_sleep.h"
#include "menu.h"
#include "Controller.h"
#include "../config.h"

int splashScreenTimer;
int colorCycle=0;
int selectedIdx;
int inputDelay;
int lineMax;
int selRom;

bool xStretch;
bool UpdateMode;

void setBright(int bright){	setBr(bright);}
bool peGetPixel(char peChar, int pe1, int pe2){	return cpGetPixel(peChar, pe1, pe2);}
bool getUpdateMode(){ return UpdateMode;}
bool getXStretch(){	return xStretch;}
void setXStretch(bool str){	xStretch = str;}
void setUpdateMode(bool str){ UpdateMode = str;}
void setLineMax(int lineM){	lineMax = lineM;}
void setSelRom(int selR){ selRom = selR;}
int getSelRom(){ return selRom;}

//!!! Colors repeat after 3Bit(example: 001 = light green, 111 = max green -> 1000 = again light green),
//		 so all values over (dec) 7 start to repeat the color, but they are stored in 5bits!!!
//returns a 16bit rgb Color (1Bit + 15Bit bgr), values for each Color from 0-31
//(MSB=? + 5Bits blue + 5Bits red + 5Bits green)
static inline int rgbColor(int red, int green, int blue)
{
	return 0x8000 + (blue << 10) + (red << 5) + green;
}

// Display noise background
static inline int getNoise()
{
	return rgbColor(rand() % 8, rand() % 8, rand() % 8);
}

//run "boot screen" (intro) and later menu to choose a rom
static inline uint16_t get_bgnd_pixel(int x, int y, int selectedIdx)
{
	if (splashScreenTimer > 0)
	{
		return getNoise();
	}
	else
	{
		return getCharPixel(x, y, colorCycle, selectedIdx);
	}
}

#define DEFAULT_MENU_DELAY 100
// Deal with user activity during boot screen and menu
void handleUserInput()
{
	if (inputDelay > 0)	inputDelay -= 1;
	
	int input = ReadControllerInput();
	
	if (splashScreenTimer > 0)
	{
		if (isAnyPressed(input)) {
			// Immediately cancel splashscreen
			//printf("Cancelling animation\n");
			splashScreenTimer = 0;
			inputDelay = DEFAULT_MENU_DELAY;
		}
	}
	else if (inputDelay <= 0)
	{
		if (isUpPressed(input) && selectedIdx > 0)
		{
			selectedIdx--;
			inputDelay = DEFAULT_MENU_DELAY;
		}
		if (isDownPressed(input) && selectedIdx < lineMax-1)
		{
			selectedIdx++;
			inputDelay = DEFAULT_MENU_DELAY;
		}
		if (isAPressed(input) || isBPressed(input) || isStartPressed(input))
			selRom = selectedIdx;
		/* 
	if (isSelectPressed(input) )
	{
		printf("Entering deep sleep\n");
		esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_AUTO);
		gpio_pullup_dis(CONFIG_HW_GPIO_SELECT);
		gpio_pulldown_en(CONFIG_HW_GPIO_SELECT);
		esp_deep_sleep_enable_ext0_wakeup(CONFIG_HW_GPIO_SELECT, 1);

		vTaskDelay(1000);
		esp_deep_sleep_start();
		printf("Exiting deep sleep\n");
	}
	*/
	}
}

//Instead of calculating the offsets for each pixel we grab, we pre-calculate the valueswhenever a frame changes, then re-use
//these as we go through all the pixels in the frame. This is much, much faster.
/*static int8_t xofs[320], yofs[240];
static int8_t xcomp[320], ycomp[240];*/

//Calculate the pixel data for a set of rows (with implied width of 320). Pixels go in dest, line is the Y-coordinate of the
//first line to be calculated, rowCount is the amount of rows to calculate. Frame increases by one every time the entire image
//is displayed; this is used to go to the next frame of animation.

void drawRows(uint16_t *dest, int y, int rowCount)
{
	if (splashScreenTimer > 0)
		splashScreenTimer--;

	for (int yy = y; yy < y + rowCount; yy++)
	{
		for (int x = 0; x < 320; x++)
		{
			*dest++ = get_bgnd_pixel(x, yy, selectedIdx);
		}
	}
}

// void initGPIO(int gpioNo){
// 	gpio_set_direction(gpioNo, GPIO_MODE_INPUT);
// 	gpio_pulldown_en(gpioNo);
// }

//initialize varibles for "timers" and input, gpios and load picture
esp_err_t menuInit()
{
	splashScreenTimer = 240;
	selectedIdx = 0;
	inputDelay = 0;
	lineMax = 0;
	UpdateMode = 0;	//default to interlaced mode
#ifdef FULL_SCREEN
	xStretch = FULL_SCREEN;
#else
	xStretch = 0;
#endif
	printf("Reading rom list\n");
	initRomList();
	setLineMax(entryCount);
	return ESP_OK;
}
