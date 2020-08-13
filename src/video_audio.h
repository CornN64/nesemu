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



int osd_installtimer(int frequency, void *func, int funcsize, void *counter, int countersize);

/*
** Audio
*/
static void (*audio_callback)(void *buffer, int length);
static void do_audio_frame();
void osd_setsound(void (*playfunc)(void *buffer, int length));
static void osd_stopsound(void);
static int osd_init_sound(void);
void osd_getsoundinfo(sndinfo_t *info);

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
void osd_getvideoinfo(vidinfo_t *info);

/* flip between full screen and windowed */
void osd_togglefullscreen(int code);

/* initialise video */
static int init(int width, int height);
static void shutdown(void);

/* set a video mode */
static int set_mode(int width, int height);

/* copy nes palette over to hardware */
static void set_palette(rgb_t *pal);

/* clear all frames to a particular color */
static void clear(uint8 color);

/* acquire the directbuffer for writing */
static bitmap_t *lock_write(void);

/* release the resource */
static void free_write(int num_dirties, rect_t *dirty_rects);
static void custom_blit(bitmap_t *bmp, int num_dirties, rect_t *dirty_rects);

//This runs on core 1.
static void videoTask(void *arg);

/*
** Input
*/
static void osd_initinput();
void osd_getinput(void);
static void osd_freeinput(void);
void osd_getmouse(int *x, int *y, int *button);

/*
** Shutdown
*/
/* this is at the bottom, to eliminate warnings */
void osd_shutdown();
static int logprint(const char *string);

/*
** Startup
*/
int osd_init();
