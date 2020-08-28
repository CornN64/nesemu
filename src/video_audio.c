// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
#include "pretty_effect.h"
#include <math.h>
#include <string.h>
#include "noftypes.h"
#include "bitmap.h"
#include "nofconfig.h"
#include "event.h"
#include "gui.h"
#include "log.h"
#include "nes.h"
#include "nes_apu.h"
#include "nes_pal.h"
#include "nesinput.h"
#include "osd.h"
#include "stdint.h"
#include "driver/i2s.h"
#include "config.h"
#include "spi_lcd.h"
#include "Controller.h"
#include "video_audio.h"

#define AUDIO_SAMPLERATE 15734*2
#define AUDIO_BUFFER_LENGTH 64
#define BITS_PER_SAMPLE 16
// Always bits_per_sample / 8
#define BYTES_PER_SAMPLE 2
#define I2S_DEVICE_ID 0

TimerHandle_t timer;

//Seemingly, this will be called only once. Should call func with a freq of frequency,
int osd_installtimer(int frequency, void *func, int funcsize, void *counter, int countersize)
{
	timer = xTimerCreate("nes", configTICK_RATE_HZ / frequency, pdTRUE, NULL, func);
	xTimerStart(timer, 0);
	printf("Timer install %dHz\n", frequency);
	return 0;
}

/*
** Audio
*/
static int samplesPerPlayback=-1;
static void (*audio_callback)(void *buffer, int length) = NULL;
#if defined(CONFIG_SOUND_ENABLED)
QueueHandle_t queue;
static void *audio_buffer;
#endif

static void do_audio_frame()
{

#if defined(CONFIG_SOUND_ENABLED)
	if (!audio_callback || getVolume() <= 0)
	{
		i2s_zero_dma_buffer(I2S_DEVICE_ID);
		return;
	}
	uint16_t *bufU = (uint16_t *)audio_buffer;
	int16_t *bufS = (int16_t *)audio_buffer;
	int samplesRemaining = samplesPerPlayback;
	int volShift = 8 - getVolume() * 2;	
	while (samplesRemaining)
	{
		int n = AUDIO_BUFFER_LENGTH > samplesRemaining ? samplesRemaining : AUDIO_BUFFER_LENGTH;
		apu_process(audio_buffer, n);
		//if (audio_callback) audio_callback(audio_buffer, n);  //Why does this crash??
		for (int i=0; i < n; i++) {
			int16_t sample = bufS[i];
			uint16_t unsignedSample = sample ^ 0x8000;
			bufU[i] = unsignedSample >> volShift;
		}
		size_t written = -1;
		i2s_write(I2S_DEVICE_ID, audio_buffer, BYTES_PER_SAMPLE * n, &written, portMAX_DELAY);
		samplesRemaining -= n;
	}
#endif
}

void osd_setsound(void (*playfunc)(void *buffer, int length))
{
	//Indicates we should call playfunc() to get more data.
	audio_callback = playfunc;
}

static void osd_stopsound(void)
{
#if defined(CONFIG_SOUND_ENABLED)
	audio_callback = NULL;
	printf("Sound stopped.\n");
	i2s_stop(I2S_DEVICE_ID);
	free(audio_buffer);
#endif
}

static int osd_init_sound(void)
{
#if defined(CONFIG_SOUND_ENABLED)
	audio_buffer = malloc(BYTES_PER_SAMPLE * AUDIO_BUFFER_LENGTH);
	i2s_config_t cfg = {
		.mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN,
		.sample_rate = AUDIO_SAMPLERATE,
		.bits_per_sample = BITS_PER_SAMPLE,
		.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
		.communication_format = I2S_COMM_FORMAT_I2S_MSB,
		.intr_alloc_flags = ESP_INTR_FLAG_INTRDISABLED,
		.dma_buf_count = 8,
		.dma_buf_len = 64,
		.use_apll = false};
	i2s_driver_install(I2S_DEVICE_ID, &cfg, 0, NULL);
	i2s_set_pin(I2S_DEVICE_ID, NULL);
	i2s_set_dac_mode(I2S_DAC_CHANNEL_LEFT_EN);
	i2s_set_sample_rates(I2S_DEVICE_ID, AUDIO_SAMPLERATE);
	samplesPerPlayback = AUDIO_SAMPLERATE / NES_REFRESH_RATE;
	printf("Finished initializing sound\n");

#endif

	audio_callback = NULL;

	return 0;
}

void osd_getsoundinfo(sndinfo_t *info)
{
	info->sample_rate = AUDIO_SAMPLERATE;
	info->bps = BITS_PER_SAMPLE;  // Internal DAC is only 8-bit anyway
}

/*
** Video
*/

static int init(int width, int height);
static void shutdown(void);
static int set_mode(int width, int height);
static void set_palette(rgb_t *pal);
static void clear(uint8 color);
static bitmap_t *lock_write(void);
static void free_write(int num_dirties, rect_t *dirty_rects);
static void custom_blit(bitmap_t *bmp, int num_dirties, rect_t *dirty_rects);
static char fb[1]; //dummy

QueueHandle_t vidQueue;

viddriver_t sdlDriver =
	{
		"Simple DirectMedia Layer", /* name */
		init,						/* init */
		shutdown,					/* shutdown */
		set_mode,					/* set_mode */
		set_palette,				/* set_palette */
		clear,						/* clear */
		lock_write,					/* lock_write */
		free_write,					/* free_write */
		custom_blit,				/* custom_blit */
		false						/* invalidate flag */
};

bitmap_t *myBitmap;

void osd_getvideoinfo(vidinfo_t *info)
{
	info->default_width = SCREEN_WIDTH;
	info->default_height = SCREEN_HEIGHT;
	info->driver = &sdlDriver;
}

/* flip between full screen and windowed */
void osd_togglefullscreen(int code)
{
}

/* initialise video */
static int init(int width, int height)
{
	return 0;
}

static void shutdown(void)
{
}

/* set a video mode */
static int set_mode(int width, int height)
{
	return 0;
}

uint16 myPalette[256];

/* copy nes palette over to hardware */
static void set_palette(rgb_t *pal)
{
	uint16 c;

	int i;

	for (i = 0; i < 256; i++)
	{
		c = (pal[i].b >> 3) + ((pal[i].g >> 2) << 5) + ((pal[i].r >> 3) << 11);
		//myPalette[i] = c;
		myPalette[i] = (c >> 8) | (c << 8);	//swizzle bytes order already here in the palette
	}
}

/* clear all frames to a particular color */
static void clear(uint8 color)
{
	//   SDL_FillRect(mySurface, 0, color);
}

/* acquire the directbuffer for writing */
static bitmap_t *lock_write(void)
{
	//   SDL_LockSurface(mySurface);
	//myBitmap = bmp_createhw((uint8 *)fb, xWidth, yHight, xWidth * 2); //SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_WIDTH*2);
	//return myBitmap;
}

/* release the resource */
static void free_write(int num_dirties, rect_t *dirty_rects)
{
	bmp_destroy(&myBitmap);
}

volatile bitmap_t *_bmp = NULL;	//used to pass info between cores
volatile bool _bmp_inited = false; //used to pass info between cores
static void custom_blit(bitmap_t *bmp, int num_dirties, rect_t *dirty_rects)
{
#ifdef RUN_VIDEO_AS_TASK
	#ifdef USE_OS_SEMAPHORES
		xQueueSend(vidQueue, &bmp, 0);
		do_audio_frame();
	#else
		_bmp = bmp;
		_bmp_inited = true;
		do_audio_frame();
	#endif
#else
	do_audio_frame();
	int x, y, xWidth, yHight;
	xWidth = getXStretch() ? SCREEN_WIDTH : NES_WIDTH;
	yHight = NES_VIS_LINES;
	x = (SCREEN_WIDTH - xWidth) >> 1;
	y = (SCREEN_HEIGHT - yHight) >> 1;
	if (getUpdateMode())
		ili9341_write_Pframe(x, y, xWidth, yHight, (const uint8_t **)bmp->line, getXStretch());
	else
		ili9341_write_Iframe(x, y, xWidth, yHight, (const uint8_t **)bmp->line, getXStretch());
#endif	
}

#ifdef RUN_VIDEO_AS_TASK
static void IRAM_ATTR videoTask(void *arg)
{
#ifdef USE_OS_SEMAPHORES
	int x, y, xWidth, yHight;	
	bitmap_t *bmp = NULL;
	yHight = NES_VIS_LINES;
	y = (SCREEN_HEIGHT - yHight) >> 1;
	
	while (true)
	{
		xWidth = getXStretch() ? SCREEN_WIDTH : NES_WIDTH;
		x = (SCREEN_WIDTH - xWidth) >> 1;
		xQueueReceive(vidQueue, &bmp, portMAX_DELAY);
		if (getUpdateMode())
			ili9341_write_Pframe(x, y, xWidth, yHight, (const uint8_t **)bmp->line, getXStretch());
		else
			ili9341_write_Iframe(x, y, xWidth, yHight, (const uint8_t **)bmp->line, getXStretch());
	}
#else
	int last_ticks;
	int x, y, xWidth, yHight;	
	bitmap_t *bmp = NULL;
	yHight = NES_VIS_LINES;
	y = (SCREEN_HEIGHT - yHight) >> 1;
	last_ticks = nofrendo_ticks;
	
	while (1)
	{
		xWidth = getXStretch() ? SCREEN_WIDTH : NES_WIDTH;
		x = (SCREEN_WIDTH - xWidth) >> 1;
		if (last_ticks != nofrendo_ticks && _bmp_inited)
		{
			last_ticks = nofrendo_ticks;
			if (getUpdateMode())
				ili9341_write_Pframe(x, y, xWidth, yHight, (const uint8_t **)_bmp->line, getXStretch());
			else
				ili9341_write_Iframe(x, y, xWidth, yHight, (const uint8_t **)_bmp->line, getXStretch());
		}
		else vTaskDelay(1);
	}
#endif
}
#endif

/*
** Input
*/

static void osd_initinput()
{
	ControllerInit();
}

void osd_getinput(void)
{
	// Note: These are in the order of PSX controller bitmasks (see controller.c)
	const int ev[16] = {
		event_joypad1_select, 0, 0, event_joypad1_start, event_joypad1_up, event_joypad1_right, event_joypad1_down, event_joypad1_left,
		0, event_hard_reset, 0, event_soft_reset, 0, event_joypad1_a, event_joypad1_b, 0};
	static int oldb = 0xffff;
	int b = ReadControllerInput();
	int chg = b ^ oldb;
	int x;
	oldb = b;
	event_t evh;
	//	printf("Input: %x\n", b);
	for (x = 0; x < 16; x++)
	{
		if (chg & 1)
		{
			evh = event_get(ev[x]);
			if (evh)
				evh((b & 1) ? INP_STATE_BREAK : INP_STATE_MAKE);
		}
		chg >>= 1;
		b >>= 1;
	}
}

static void osd_freeinput(void)
{
}

void osd_getmouse(int *x, int *y, int *button)
{
}

/*
** Shutdown
*/

/* this is at the bottom, to eliminate warnings */
void osd_shutdown()
{
	osd_stopsound();
	osd_freeinput();
}

static int logprint(const char *string)
{
	return printf("%s", string);
}

/*
** Startup
*/

int osd_init()
{
	log_chain_logfunc(logprint);

	if (osd_init_sound())
		return -1;
	printf("free heap after sound init: %d\n", xPortGetFreeHeapSize());
	ili9341_init();
    ili9341_clr();
#ifdef RUN_VIDEO_AS_TASK
	#ifdef USE_OS_SEMAPHORES
	vidQueue = xQueueCreate(1, sizeof(bitmap_t *));
	#endif
	xTaskCreatePinnedToCore(&videoTask, "videoTask", 2048, NULL, 5, NULL, RUN_VIDEO_AS_TASK);	//Start video pump task on core 1 or 0
#endif	
	osd_initinput();
	printf("free heap after input init: %d\n", xPortGetFreeHeapSize());
	return 0;
}
