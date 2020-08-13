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

#include <Arduino.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include "config.h"
#include "pretty_effect.h"
#include "esp_deep_sleep.h"
#include "Controller.h"

//Bit0 Bit1 Bit2 Bit3 Bit4 Bit5 Bit6 Bit7
//SLCT           STRT UP   RGHT DOWN LEFT
//Bit8 Bit9 Bt10 Bt11 Bt12 Bt13 Bt14 Bt15
//L2   R2   L1   R1    /\   O    X   |_|
// NOTE: These mappings aren't reflected in video_audio.c
// any changes have to be reflected in osd_getinput
// TODO: Make these both work together
#define PSX_SELECT 1
#define PSX_START (1 << 3)
#define PSX_UP (1 << 4)
#define PSX_RIGHT (1 << 5)
#define PSX_DOWN (1 << 6)
#define PSX_LEFT (1 << 7)
#define PSX_L2 (1 << 8)
#define PSX_R2 (1 << 9)
#define PSX_L1 (1 << 10)
#define PSX_R1 (1 << 11)
#define PSX_TRIANGLE (1 << 12)
#define PSX_CIRCLE (1 << 13)
#define PSX_X (1 << 14)
#define PSX_SQUARE (1 << 15)
#define A_BUTTON PSX_CIRCLE
#define B_BUTTON PSX_X
#define TURBO_A_BUTTON PSX_TRIANGLE
#define TURBO_B_BUTTON PSX_SQUARE
#define MENU_BUTTON PSX_L1
#define POWER_BUTTON PSX_R1

#define DELAY() asm("nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;")

#if defined(CONFIG_HW_CONTROLLER_PSX) || defined(CONFIG_HW_CONTROLLER_GPIO) || defined(CONFIG_HW_CONTROLLER_NES) || defined(CONFIG_HW_CONTROLLER_SNES)

int volume, bright;
int inpDelay;
bool shutdown;

#ifdef CONFIG_HW_CONTROLLER_NES
enum NES_Button
{
  NES_A = 0,
  NES_B = 1,
  NES_SELECT = 2,
  NES_START = 3,
  NES_UP = 4,
  NES_DOWN = 5,
  NES_LEFT = 6,
  NES_RIGHT = 7,
};

//This has to be initialized once
void NES_init()
{
  pinMode(CONFIG_HW_NES_DAT, INPUT_PULLUP);	//use pull up to avoid issues if controller is unplugged
  pinMode(CONFIG_HW_NES_LATCH, GPIO_MODE_OUTPUT);
  digitalWrite(CONFIG_HW_NES_LATCH, 0);
  pinMode(CONFIG_HW_NES_CLK, GPIO_MODE_OUTPUT);
  digitalWrite(CONFIG_HW_NES_CLK, 1);
}

int NES_poll()
{
  int buttons = 0;
  digitalWrite(CONFIG_HW_NES_LATCH, 1);
  delayMicroseconds(0);
  digitalWrite(CONFIG_HW_NES_LATCH, 0);
  delayMicroseconds(0);
  for (int i = 0; i < 8; i++)
  {
    buttons |= (digitalRead(CONFIG_HW_NES_DAT) << i);
    digitalWrite(CONFIG_HW_NES_CLK, 0);
    delayMicroseconds(0);
    digitalWrite(CONFIG_HW_NES_CLK, 1);
    delayMicroseconds(0);
  }
  return buttons;
}
#endif

#ifdef CONFIG_HW_CONTROLLER_SNES
enum SNES_Button
{
  SNES_B = 0,
  SNES_Y = 1,
  SNES_SELECT = 2,
  SNES_START = 3,
  SNES_UP = 4,
  SNES_DOWN = 5,
  SNES_LEFT = 6,
  SNES_RIGHT = 7,
  SNES_A = 8,
  SNES_X = 9,
  SNES_L = 10,
  SNES_R = 11,
};

//This has to be initialized once
void SNES_init()
{
  pinMode(CONFIG_HW_SNES_DAT, INPUT_PULLUP);	//use pull up to avoid issues if controller is unplugged
  pinMode(CONFIG_HW_SNES_LATCH, GPIO_MODE_OUTPUT);
  digitalWrite(CONFIG_HW_SNES_LATCH, 0);
  pinMode(CONFIG_HW_SNES_CLK, GPIO_MODE_OUTPUT);
  digitalWrite(CONFIG_HW_SNES_CLK, 1);
}

int SNES_poll()
{
  int buttons = 0;
  digitalWrite(CONFIG_HW_SNES_LATCH, 1);
  delayMicroseconds(0);
  digitalWrite(CONFIG_HW_SNES_LATCH, 0);
  delayMicroseconds(0);
  for (int i = 0; i < 12; i++)
  {
    buttons |= (digitalRead(CONFIG_HW_SNES_DAT) << i);
    digitalWrite(CONFIG_HW_SNES_CLK, 0);
    delayMicroseconds(0);
    digitalWrite(CONFIG_HW_SNES_CLK, 1);
    delayMicroseconds(0);
  }
  return buttons;
}
#endif

#ifdef CONFIG_HW_CONTROLLER_PSX
/* Sends and receives a byte from/to the PSX controller using SPI */
static int psxSendRecv(int send)
{
  int x;
  int ret = 0;
  volatile int delay;

#if 0
  while (1) {
    GPIO.out_w1ts = (1 << CONFIG_HW_PSX_CMD);
    GPIO.out_w1ts = (1 << CONFIG_HW_PSX_CLK);
    GPIO.out_w1tc = (1 << CONFIG_HW_PSX_CMD);
    GPIO.out_w1tc = (1 << CONFIG_HW_PSX_CLK);
  }
#endif

  GPIO.out_w1tc = (1 << CONFIG_HW_PSX_ATT);
  for (delay = 0; delay < 100; delay++);
  for (x = 0; x < 8; x++)
  {
    if (send & 1)
    {
      GPIO.out_w1ts = (1 << CONFIG_HW_PSX_CMD);
    }
    else
    {
      GPIO.out_w1tc = (1 << CONFIG_HW_PSX_CMD);
    }
    DELAY();
    for (delay = 0; delay < 100; delay++);
    GPIO.out_w1tc = (1 << CONFIG_HW_PSX_CLK);
    for (delay = 0; delay < 100; delay++);
    GPIO.out_w1ts = (1 << CONFIG_HW_PSX_CLK);
    ret >>= 1;
    send >>= 1;
    if (GPIO.in & (1 << CONFIG_HW_PSX_DAT))
      ret |= 128;
  }
  return ret;
}

static void psxDone()
{
  DELAY();
  GPIO_REG_WRITE(GPIO_OUT_W1TS_REG, (1 << CONFIG_HW_PSX_ATT));
}
#endif

bool showMenu;
bool getShowMenu()
{
  return showMenu;
}

bool isSelectPressed(int ctl)
{
  return !(ctl & PSX_SELECT);
}
bool isStartPressed(int ctl)
{
  return !(ctl & PSX_START);
}
bool isUpPressed(int ctl)
{
  return !(ctl & PSX_UP);
}
bool isRightPressed(int ctl)
{
  return !(ctl & PSX_RIGHT);
}
bool isDownPressed(int ctl)
{
  return !(ctl & PSX_DOWN);
}
bool isLeftPressed(int ctl)
{
  return !(ctl & PSX_LEFT);
}
bool isAPressed(int ctl)
{
  return !(ctl & A_BUTTON);
}
bool isBPressed(int ctl)
{
  return !(ctl & B_BUTTON);
}
bool isTurboAPressed(int ctl)
{
  return !(ctl & TURBO_A_BUTTON);
}
bool isTurboBPressed(int ctl)
{
  return !(ctl & TURBO_B_BUTTON);
}
bool isMenuPressed(int ctl)
{
  return !(ctl & MENU_BUTTON);
}
bool isPowerPressed(int ctl)
{
  return !(ctl & POWER_BUTTON);
}
bool isAnyDirectionPressed(int ctl)
{
  return isUpPressed(ctl) || isDownPressed(ctl) || isLeftPressed(ctl) || isRightPressed(ctl);
}

bool isAnyActionPressed(int ctl)
{
  return isStartPressed(ctl) || isSelectPressed(ctl) || isMenuPressed(ctl) || isPowerPressed(ctl);
}

bool isAnyFirePressed(int ctl)
{
  return isAPressed(ctl) || isBPressed(ctl) || isTurboAPressed(ctl) || isTurboBPressed(ctl);
}

bool isAnyPressed(int ctl)
{
  return isAnyDirectionPressed(ctl) || isAnyActionPressed(ctl) || isAnyFirePressed(ctl);
}

int turboACounter = 0;
int turboBCounter = 0;
int turboASpeed = 3;
int turboBSpeed = 3;
int MAX_TURBO = 6;
int TURBO_COUNTER_RESET = 210;

int getTurboA() {
  return turboASpeed;
}

int getTurboB() {
  return turboBSpeed;
}

int ReadControllerInput()
{
  int b2b1 = 65535;
  if (inpDelay > 0)
    inpDelay--;

#ifdef CONFIG_HW_CONTROLLER_PSX
  int b1, b2;
  psxSendRecv(0x01);		//wake up
  psxSendRecv(0x42);		//get data
  psxSendRecv(0xff);		//should return 0x5a
  b1 = psxSendRecv(0xff); 	//buttons byte 1
  b2 = psxSendRecv(0xff); 	//buttons byte 2
  psxDone();
  b2b1 = (b2 << 8) | b1;
#endif

#ifdef CONFIG_HW_CONTROLLER_GPIO
  if (gpio_get_level(CONFIG_HW_GPIO_UP))      b2b1 -= PSX_UP;
  if (gpio_get_level(CONFIG_HW_GPIO_DOWN))    b2b1 -= PSX_DOWN;
  if (gpio_get_level(CONFIG_HW_GPIO_RIGHT))   b2b1 -= PSX_RIGHT;
  if (gpio_get_level(CONFIG_HW_GPIO_LEFT))    b2b1 -= PSX_LEFT;
  if (gpio_get_level(CONFIG_HW_GPIO_SELECT))  b2b1 -= PSX_SELECT;
  if (gpio_get_level(CONFIG_HW_GPIO_START))   b2b1 -= PSX_START;
  if (gpio_get_level(CONFIG_HW_GPIO_B))       b2b1 -= B_BUTTON;
  if (gpio_get_level(CONFIG_HW_GPIO_A))       b2b1 -= A_BUTTON;
  if (gpio_get_level(CONFIG_HW_GPIO_TURBO_B)) b2b1 -= TURBO_B_BUTTON;
  if (gpio_get_level(CONFIG_HW_GPIO_TURBO_A)) b2b1 -= TURBO_A_BUTTON;
  if (gpio_get_level(CONFIG_HW_GPIO_MENU))    b2b1 -= MENU_BUTTON;
  if (gpio_get_level(CONFIG_HW_GPIO_POWER))   b2b1 -= POWER_BUTTON;
#endif

#ifdef CONFIG_HW_CONTROLLER_NES
  int Button = NES_poll();
  if (!((Button >> NES_UP) & 1))     b2b1 -= PSX_UP;
  if (!((Button >> NES_DOWN) & 1))   b2b1 -= PSX_DOWN;
  if (!((Button >> NES_RIGHT) & 1)) b2b1 -= PSX_RIGHT;
  if (!((Button >> NES_LEFT) & 1))   b2b1 -= PSX_LEFT;
  if (!((Button >> NES_SELECT) & 1)) b2b1 -= PSX_SELECT;
  if (!((Button >> NES_START) & 1))  b2b1 -= PSX_START;
  if (!((Button >> NES_B) & 1))      b2b1 -= B_BUTTON;
  if (!((Button >> NES_A) & 1))      b2b1 -= A_BUTTON;
  if (!(((Button >> NES_SELECT) | (Button >> NES_LEFT)) & 1)) b2b1 -= MENU_BUTTON;
  if (!(((Button >> NES_SELECT) | (Button >> NES_START)) & 1)) b2b1 -= POWER_BUTTON;
#endif

#ifdef CONFIG_HW_CONTROLLER_SNES
  int Button = SNES_poll();
  if (!((Button >> SNES_UP) & 1))     b2b1 -= PSX_UP;
  if (!((Button >> SNES_DOWN) & 1))   b2b1 -= PSX_DOWN;
  if (!((Button >> SNES_RIGHT) & 1))  b2b1 -= PSX_RIGHT;
  if (!((Button >> SNES_LEFT) & 1))   b2b1 -= PSX_LEFT;
  if (!((Button >> SNES_SELECT) & 1)) b2b1 -= PSX_SELECT;
  if (!((Button >> SNES_START) & 1))  b2b1 -= PSX_START;
  if (!((Button >> SNES_B) & 1))      b2b1 -= B_BUTTON;
  if (!((Button >> SNES_A) & 1))      b2b1 -= A_BUTTON;
  if (!((Button >> SNES_Y) & 1))      b2b1 -= TURBO_B_BUTTON;
  if (!((Button >> SNES_X) & 1))      b2b1 -= TURBO_A_BUTTON;
  if (!((Button >> SNES_R) & 1))      b2b1 -= POWER_BUTTON;
  if (!((Button >> SNES_L) & 1))      b2b1 -= MENU_BUTTON;  
#endif

  if (isMenuPressed(b2b1) && inpDelay == 0)
  {
    showMenu = !showMenu;
    inpDelay = 20;
  }
  
  if (showMenu)
  {
    if (inpDelay == 0)
    {
      if (isUpPressed(b2b1) && volume < 4) volume++;
      if (isDownPressed(b2b1) && volume > 0) volume--;
      if (isRightPressed(b2b1) && bright < 4) bright++;
      if (isLeftPressed(b2b1) && bright > 0)  bright--;
      if (isAPressed(b2b1)) setYStretch(1 - getYStretch());
      if (isBPressed(b2b1)) setXStretch(1 - getXStretch());
      if (isTurboAPressed(b2b1)) turboASpeed = (turboASpeed + 1) % MAX_TURBO;
      if (isTurboBPressed(b2b1)) turboBSpeed = (turboBSpeed + 1) % MAX_TURBO;
      if (isAnyPressed(b2b1)) inpDelay = 15;
    }
  }
  else
  {
    // ! todo: Implement sleep mode for PSX that also works with GPIO code here, disabled for now.
    /*
      if (isPowerPressed(b2b1) && inpDelay > 0)
    	bright = -1;
      if (bright < 0)
      {
    	if (bright == -1 && inpDelay == 0)
    	{
    		esp_sleep_enable_timer_wakeup(1000 * 100);
    		vTaskDelay(100);
    		esp_deep_sleep_start();
    	}

    	if (bright == -1 && inpDelay > 100)
    	{
    		bright = -2;
    		shutdown = 1;
    		esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_AUTO);
    		gpio_pullup_dis(CONFIG_HW_GPIO_POWER);
    		gpio_pulldown_en(CONFIG_HW_GPIO_POWER);
    		esp_deep_sleep_enable_ext0_wakeup(CONFIG_HW_GPIO_POWER, 1);

    		vTaskDelay(1500);
    		esp_deep_sleep_start();
    	}
    	bright = 4;
      }
      inpDelay += 2;
    */
  }
  
  if (!showMenu)
  {
    if (turboASpeed > 0 && isTurboAPressed(b2b1))
    {
      b2b1 |= A_BUTTON;
      if ((turboACounter % (turboASpeed * 2)) == 0)
      {
        b2b1 -= A_BUTTON;
      }
      turboACounter = (turboACounter + 1) % TURBO_COUNTER_RESET; // 30 is the LCM of numers 1 thru 6
    }
    else
    {
      turboACounter = 0;
    }

    if (turboBSpeed > 0 && isTurboBPressed(b2b1))
    {
      b2b1 |= B_BUTTON;
      if ((turboBCounter % (turboBSpeed * 2)) == 0)
      {
        b2b1 -= B_BUTTON;
      }
      turboBCounter = (turboBCounter + 1) % TURBO_COUNTER_RESET; // 30 is the LCM of numers 1 thru 6
    }
    else
    {
      turboBCounter = 0;
    }
  }
  return b2b1;
}

int getBright()
{
  return bright;
}

int getVolume()
{
  return volume;
}

bool getShutdown()
{
  return shutdown;
}

void initGPIO(int gpioNo)
{
  gpio_set_direction(gpioNo, GPIO_MODE_INPUT);
  gpio_pulldown_en(gpioNo);
}

void ControllerInit()
{
  printf("Game controller initalizing\n");

#ifdef CONFIG_HW_CONTROLLER_NES
  NES_init();
  printf("NES Control initated\n");
#endif

#ifdef CONFIG_HW_CONTROLLER_SNES
  SNES_init();
  printf("SNES Control initated\n");
#endif

#ifdef CONFIG_HW_CONTROLLER_GPIO
  initGPIO(CONFIG_HW_GPIO_START);
  initGPIO(CONFIG_HW_GPIO_SELECT);
  initGPIO(CONFIG_HW_GPIO_UP);
  initGPIO(CONFIG_HW_GPIO_DOWN);
  initGPIO(CONFIG_HW_GPIO_LEFT);
  initGPIO(CONFIG_HW_GPIO_RIGHT);
  initGPIO(CONFIG_HW_GPIO_B);
  initGPIO(CONFIG_HW_GPIO_A);
  initGPIO(CONFIG_HW_GPIO_TURBO_B);
  initGPIO(CONFIG_HW_GPIO_TURBO_A);
  initGPIO(CONFIG_HW_GPIO_MENU);
  initGPIO(CONFIG_HW_GPIO_POWER);
  printf("GPIO Control initated\n");
#endif

#ifdef CONFIG_HW_CONTROLLER_PSX
  volatile int delay;
  int t;
  showMenu = 0;
  shutdown = 0;
  gpio_config_t gpioconf[2] = {
    { .pin_bit_mask = (1 << CONFIG_HW_PSX_CLK) | (1 << CONFIG_HW_PSX_CMD) | (1 << CONFIG_HW_PSX_ATT),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_PIN_INTR_DISABLE
    },
    { .pin_bit_mask = (1 << CONFIG_HW_PSX_DAT),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_PIN_INTR_DISABLE
    }
  };
  gpio_config(&gpioconf[0]);
  gpio_config(&gpioconf[1]);

  //Send a few dummy bytes to clean the pipes.
  psxSendRecv(0);
  psxDone();
  for (delay = 0; delay < 500; delay++) DELAY();
  psxSendRecv(0);
  psxDone();
  for (delay = 0; delay < 500; delay++) DELAY();
  //Try and detect the type of controller, so we can give the user some diagnostics.
  psxSendRecv(0x01);
  t = psxSendRecv(0x00);
  psxDone();
  if (t == 0 || t == 0xff)
  {
    printf("No PSX/PS2 controller detected (0x%X). You will not be able to control the game.\n", t);
  }
  else
  {
    printf("PSX controller type 0x%X\n", t);
  }
#endif

  inpDelay = 0;
  volume = 3;
  bright = 4;
}

#else

int ReadControllerInput()
{
  return 0xFFFF;
}

void ControllerInit()
{
  printf("Controller disabled in menuconfig; no input enabled.\n");
}

#endif
