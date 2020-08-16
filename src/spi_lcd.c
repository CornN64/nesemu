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

#include <string.h>
#include <stdio.h>
#include "config.h"
#include "rom/ets_sys.h"
#include "rom/gpio.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/gpio_struct.h"
#include "soc/io_mux_reg.h"
#include "soc/spi_reg.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/periph_ctrl.h"
#include "driver/spi_master.h"
#include "spi_lcd.h"
#include "driver/ledc.h"
#include "nes.h"

#define PIN_NUM_MISO CONFIG_HW_LCD_MISO_GPIO
#define PIN_NUM_MOSI CONFIG_HW_LCD_MOSI_GPIO
#define PIN_NUM_CLK CONFIG_HW_LCD_CLK_GPIO
#define PIN_NUM_CS CONFIG_HW_LCD_CS_GPIO
#define PIN_NUM_DC CONFIG_HW_LCD_DC_GPIO
#define PIN_NUM_RST CONFIG_HW_LCD_RESET_GPIO
#define PIN_NUM_BCKL CONFIG_HW_LCD_BL_GPIO
#define LCD_SEL_CMD() GPIO.out_w1tc = (1 << PIN_NUM_DC)  // Low to send command
#define LCD_SEL_DATA() GPIO.out_w1ts = (1 << PIN_NUM_DC) // High to send data
#define LCD_RST_SET() GPIO.out_w1ts = (1 << PIN_NUM_RST)
#define LCD_RST_CLR() GPIO.out_w1tc = (1 << PIN_NUM_RST)

#define LEDC_LS_TIMER LEDC_TIMER_1
#define LEDC_LS_MODE LEDC_LOW_SPEED_MODE
#define LEDC_LS_CH3_CHANNEL LEDC_CHANNEL_3

#if PIN_NUM_BCKL >= 0
#if CONFIG_HW_INV_BL
#define LCD_BKG_ON() GPIO.out_w1tc = (1 << PIN_NUM_BCKL)  // Backlight ON
#define LCD_BKG_OFF() GPIO.out_w1ts = (1 << PIN_NUM_BCKL) //Backlight OFF
#else
#define LCD_BKG_ON() GPIO.out_w1ts = (1 << PIN_NUM_BCKL)  // Backlight ON
#define LCD_BKG_OFF() GPIO.out_w1tc = (1 << PIN_NUM_BCKL) //Backlight OFF
#endif
#endif

#define SPI_NUM 0x3

#define waitForSPIReady()                                 \
    while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM)) & SPI_USR) \
        ;

ledc_channel_config_t ledc_channel;

#if PIN_NUM_BCKL >= 0
*void initBCKL()
{
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 500,
        .speed_mode = LEDC_LS_MODE,
        .timer_num = LEDC_LS_TIMER};

    ledc_timer_config(&ledc_timer);

    ledc_channel.channel = LEDC_LS_CH3_CHANNEL;
    ledc_channel.duty = 500;
    ledc_channel.gpio_num = PIN_NUM_BCKL;
    ledc_channel.speed_mode = LEDC_LS_MODE;
    ledc_channel.timer_sel = LEDC_LS_TIMER;

    ledc_channel_config(&ledc_channel);
}
#endif

void setBrightness(int bright)
{
    /*int duty=1000;
	if(bright == -2)duty=0;
	if(bright == 0)duty=1000;
	if(bright == 1)duty=2050;
	if(bright == 2)duty=4100;
	if(bright == 3)duty=6150;
	if(bright == 4)duty=8190;
	ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, duty);
    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);*/
    setBright(bright);
}

static void spi_write_byte(const uint8_t data)
{
    SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 0x7, SPI_USR_MOSI_DBITLEN_S);
    WRITE_PERI_REG((SPI_W0_REG(SPI_NUM)), data);
    SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
    waitForSPIReady();
}

static void LCD_WriteCommand(const uint8_t cmd)
{
    LCD_SEL_CMD();
    spi_write_byte(cmd);
}

static void LCD_WriteData(const uint8_t data)
{
    LCD_SEL_DATA();
    spi_write_byte(data);
}

#if 0 /*INIT LCD*/

static void ILI9341_INITIAL()
{
#if PIN_NUM_BCKL >= 0
    LCD_BKG_ON();
#endif
    //------------------------------------Reset Sequence-----------------------------------------//

    LCD_RST_SET();
    ets_delay_us(100000);

    LCD_RST_CLR();
    ets_delay_us(200000);

    LCD_RST_SET();
    ets_delay_us(200000);

#if (CONFIG_HW_LCD_TYPE == LCD_TYPE_ILI)
    //************* Start Initial Sequence **********//
    /* Power contorl B, power control = 0, DC_ENA = 1 */
    LCD_WriteCommand(0xCF);
    LCD_WriteData(0x00);
    LCD_WriteData(0x83);
    LCD_WriteData(0X30);

    /* Power on sequence control,
     * cp1 keeps 1 frame, 1st frame enable
     * vcl = 0, ddvdh=3, vgh=1, vgl=2
     * DDVDH_ENH=1
     */
    LCD_WriteCommand(0xED);
    LCD_WriteData(0x64);
    LCD_WriteData(0x03);
    LCD_WriteData(0X12);
    LCD_WriteData(0X81);

    /* Driver timing control A,
     * non-overlap=default +1
     * EQ=default - 1, CR=default
     * pre-charge=default - 1
     */
    LCD_WriteCommand(0xE8);
    LCD_WriteData(0x85);
    LCD_WriteData(0x01); //i
    LCD_WriteData(0x79); //i

    /* Power control A, Vcore=1.6V, DDVDH=5.6V */
    LCD_WriteCommand(0xCB);
    LCD_WriteData(0x39);
    LCD_WriteData(0x2C);
    LCD_WriteData(0x00);
    LCD_WriteData(0x34);
    LCD_WriteData(0x02);

    /* Pump ratio control, DDVDH=2xVCl */
    LCD_WriteCommand(0xF7);
    LCD_WriteData(0x20);

    /* Driver timing control, all=0 unit */
    LCD_WriteCommand(0xEA);
    LCD_WriteData(0x00);
    LCD_WriteData(0x00);

    /* Power control 1, GVDD=4.75V */
    LCD_WriteCommand(0xC0); //Power control
    LCD_WriteData(0x26);    //i  //VRH[5:0]

    /* Power control 2, DDVDH=VCl*2, VGH=VCl*7, VGL=-VCl*3 */
    LCD_WriteCommand(0xC1); //Power control
    LCD_WriteData(0x11);    //i //SAP[2:0];BT[3:0]

    /* VCOM control 1, VCOMH=4.025V, VCOML=-0.950V */
    LCD_WriteCommand(0xC5); //VCM control
    LCD_WriteData(0x35);    //i
    LCD_WriteData(0x3E);    //i

    /* VCOM control 2, VCOMH=VMH-2, VCOML=VML-2 */
    LCD_WriteCommand(0xC7); //VCM control2
    LCD_WriteData(0xBE);    //i   //»òÕß B1h

    /* Memory access contorl, MX=MY=0, MV=1, ML=0, BGR=1, MH=0 */
    LCD_WriteCommand(0x36); // Memory Access Control
#ifdef ROTATE_LCD_180
    LCD_WriteData(0x28^0xC0);    //i //was 0x48
#else
    LCD_WriteData(0x28);    //i //was 0x48
#endif
    /* Pixel format, 16bits/pixel for RGB/MCU interface */
    LCD_WriteCommand(0x3A);
    LCD_WriteData(0x55);

    /* Frame rate control, f=fosc, 70Hz fps */
    LCD_WriteCommand(0xB1);
    LCD_WriteData(0x00);
    LCD_WriteData(0x1B); //18

    /* Enable 3G, disabled */
    LCD_WriteCommand(0xF2); // 3Gamma Function Disable
    LCD_WriteData(0x08);

    /* Gamma set, curve 1 */
    LCD_WriteCommand(0x26); //Gamma curve selected
    LCD_WriteData(0x01);

    /* Positive gamma correction */
    LCD_WriteCommand(0xE0); //Set Gamma
    LCD_WriteData(0x1F);
    LCD_WriteData(0x1A);
    LCD_WriteData(0x18);
    LCD_WriteData(0x0A);
    LCD_WriteData(0x0F);
    LCD_WriteData(0x06);
    LCD_WriteData(0x45);
    LCD_WriteData(0X87);
    LCD_WriteData(0x32);
    LCD_WriteData(0x0A);
    LCD_WriteData(0x07);
    LCD_WriteData(0x02);
    LCD_WriteData(0x07);
    LCD_WriteData(0x05);
    LCD_WriteData(0x00);

    /* Negative gamma correction */
    LCD_WriteCommand(0XE1); //Set Gamma
    LCD_WriteData(0x00);
    LCD_WriteData(0x25);
    LCD_WriteData(0x27);
    LCD_WriteData(0x05);
    LCD_WriteData(0x10);
    LCD_WriteData(0x09);
    LCD_WriteData(0x3A);
    LCD_WriteData(0x78);
    LCD_WriteData(0x4D);
    LCD_WriteData(0x05);
    LCD_WriteData(0x18);
    LCD_WriteData(0x0D);
    LCD_WriteData(0x38);
    LCD_WriteData(0x3A);
    LCD_WriteData(0x1F);

    /* Column address set, SC=0, EC=0xEF */
    LCD_WriteCommand(0x2A);
    LCD_WriteData(0x00);
    LCD_WriteData(0x00);
    LCD_WriteData(0x00);
    LCD_WriteData(0xEF);

    /* Page address set, SP=0, EP=0x013F */
    LCD_WriteCommand(0x2B);
    LCD_WriteData(0x00);
    LCD_WriteData(0x00);
    LCD_WriteData(0x01);
    LCD_WriteData(0x3f);
    LCD_WriteCommand(0x2C);

    /* Entry mode set, Low vol detect disabled, normal display */
    LCD_WriteCommand(0xB7);
    LCD_WriteData(0x07);

    /* Display function control */
    LCD_WriteCommand(0xB6); // Display Function Control
    LCD_WriteData(0x0A);    //8 82 27
    LCD_WriteData(0x82);
    LCD_WriteData(0x27);
    LCD_WriteData(0x00);

    //LCD_WriteCommand(0xF6); //not there
    //LCD_WriteData(0x01);
    //LCD_WriteData(0x30);

#endif
#if (CONFIG_HW_LCD_TYPE == LCD_TYPE_ST)

    //212
    //122
    LCD_WriteCommand(0x36);
    LCD_WriteData((1 << 5) | (1 << 6)); //MV 1, MX 1

    LCD_WriteCommand(0x3A);
    LCD_WriteData(0x55);

    LCD_WriteCommand(0xB2);
    LCD_WriteData(0x0c);
    LCD_WriteData(0x0c);
    LCD_WriteData(0x00);
    LCD_WriteData(0x33);
    LCD_WriteData(0x33);

    LCD_WriteCommand(0xB7);
    LCD_WriteData(0x35);

    LCD_WriteCommand(0xBB);
    LCD_WriteData(0x2B);

    LCD_WriteCommand(0xC0);
    LCD_WriteData(0x2C);

    LCD_WriteCommand(0xC2);
    LCD_WriteData(0x01);
    LCD_WriteData(0xFF);

    LCD_WriteCommand(0xC3);
    LCD_WriteData(0x11);

    LCD_WriteCommand(0xC4);
    LCD_WriteData(0x20);

    LCD_WriteCommand(0xC6);
    LCD_WriteData(0x0f);

    LCD_WriteCommand(0xD0);
    LCD_WriteData(0xA4);
    LCD_WriteData(0xA1);

    LCD_WriteCommand(0xE0);
    LCD_WriteData(0xD0);
    LCD_WriteData(0x00);
    LCD_WriteData(0x05);
    LCD_WriteData(0x0E);
    LCD_WriteData(0x15);
    LCD_WriteData(0x0D);
    LCD_WriteData(0x37);
    LCD_WriteData(0x43);
    LCD_WriteData(0x47);
    LCD_WriteData(0x09);
    LCD_WriteData(0x15);
    LCD_WriteData(0x12);
    LCD_WriteData(0x16);
    LCD_WriteData(0x19);

    LCD_WriteCommand(0xE1);
    LCD_WriteData(0xD0);
    LCD_WriteData(0x00);
    LCD_WriteData(0x05);
    LCD_WriteData(0x0D);
    LCD_WriteData(0x0C);
    LCD_WriteData(0x06);
    LCD_WriteData(0x2D);
    LCD_WriteData(0x44);
    LCD_WriteData(0x40);
    LCD_WriteData(0x0E);
    LCD_WriteData(0x1C);
    LCD_WriteData(0x18);
    LCD_WriteData(0x16);
    LCD_WriteData(0x19);

#endif

    /* Sleep out */
    LCD_WriteCommand(0x11); //Exit Sleep
    ets_delay_us(100000);
    /* Display on */
    LCD_WriteCommand(0x29); //Display on
    ets_delay_us(100000);
}
//.............LCD API END----------

static void ili_gpio_init()
{
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[PIN_NUM_DC], 2);  //DC PIN
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[PIN_NUM_RST], 2); //RESET PIN
#if PIN_NUM_BCKL >= 0
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[PIN_NUM_BCKL, 2);  //BKL PIN
    WRITE_PERI_REG(GPIO_ENABLE_W1TS_REG, 1 << PIN_NUM_DC | 1 << PIN_NUM_RST | 1 << PIN_NUM_BCKL);
#else
    WRITE_PERI_REG(GPIO_ENABLE_W1TS_REG, 1 << PIN_NUM_DC | 1 << PIN_NUM_RST);
#endif
}

static void spi_master_init()
{
    periph_module_enable(PERIPH_VSPI_MODULE);
    periph_module_enable(PERIPH_SPI_DMA_MODULE);

    ets_printf("lcd spi pin mux init ...\r\n");
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[PIN_NUM_CS], 2);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[PIN_NUM_CLK], 2);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[PIN_NUM_MOSI], 2);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[PIN_NUM_MISO], 2);
    WRITE_PERI_REG(GPIO_ENABLE_W1TS_REG, 1 << PIN_NUM_CS | 1 << PIN_NUM_CLK | 1 << PIN_NUM_MOSI);

    ets_printf("lcd spi signal init\r\n");
    gpio_matrix_in(PIN_NUM_MISO, VSPIQ_IN_IDX, 0);
    gpio_matrix_out(PIN_NUM_MOSI, VSPID_OUT_IDX, 0, 0);
    gpio_matrix_out(PIN_NUM_CLK, VSPICLK_OUT_IDX, 0, 0);
    gpio_matrix_out(PIN_NUM_CS, VSPICS0_OUT_IDX, 0, 0);
    ets_printf("Vspi config\r\n");

    CLEAR_PERI_REG_MASK(SPI_SLAVE_REG(SPI_NUM), SPI_TRANS_DONE << 5);
    SET_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_CS_SETUP);
    CLEAR_PERI_REG_MASK(SPI_PIN_REG(SPI_NUM), SPI_CK_IDLE_EDGE);
    CLEAR_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_CK_OUT_EDGE);
    CLEAR_PERI_REG_MASK(SPI_CTRL_REG(SPI_NUM), SPI_WR_BIT_ORDER);
    CLEAR_PERI_REG_MASK(SPI_CTRL_REG(SPI_NUM), SPI_RD_BIT_ORDER);
    CLEAR_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_DOUTDIN);
    WRITE_PERI_REG(SPI_USER1_REG(SPI_NUM), 0);
    SET_PERI_REG_BITS(SPI_CTRL2_REG(SPI_NUM), SPI_MISO_DELAY_MODE, 0, SPI_MISO_DELAY_MODE_S);
    CLEAR_PERI_REG_MASK(SPI_SLAVE_REG(SPI_NUM), SPI_SLAVE_MODE);

    // WRITE_PERI_REG(SPI_CLOCK_REG(SPI_NUM), (1 << SPI_CLKCNT_N_S) | (1 << SPI_CLKCNT_L_S));//40MHz
    WRITE_PERI_REG(SPI_CLOCK_REG(SPI_NUM), SPI_CLK_EQU_SYSCLK); // 80Mhz

    SET_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_CS_SETUP | SPI_CS_HOLD | SPI_USR_MOSI);
    SET_PERI_REG_MASK(SPI_CTRL2_REG(SPI_NUM), ((0x4 & SPI_MISO_DELAY_NUM) << SPI_MISO_DELAY_NUM_S));
    CLEAR_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_USR_COMMAND);
    SET_PERI_REG_BITS(SPI_USER2_REG(SPI_NUM), SPI_USR_COMMAND_BITLEN, 0, SPI_USR_COMMAND_BITLEN_S);
    CLEAR_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_USR_ADDR);
    SET_PERI_REG_BITS(SPI_USER1_REG(SPI_NUM), SPI_USR_ADDR_BITLEN, 0, SPI_USR_ADDR_BITLEN_S);
    CLEAR_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_USR_MISO);
    SET_PERI_REG_MASK(SPI_USER_REG(SPI_NUM), SPI_USR_MOSI);
    char i;
    for (i = 0; i < 16; ++i)
    {
        WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + (i << 2)), 0);
    }
}
#endif /*INIT LCD*/

#define U16x2toU32(m, l) ((((uint32_t)(l >> 8 | (l & 0xFF) << 8)) << 16) | (m >> 8 | (m & 0xFF) << 8))
#define U16xtoZ16(m) ((uint32_t)((m >> 8 | (m & 0xFF) << 8)))

extern uint16_t myPalette[];

char *menuText[10] = {
    "Brightness 46 0.",
    "Volume     82 9.",
    " .",
    "Turbo  3 $  1 @.",
    " .",
    "These cause lag.",
    "Horiz Scale 1 5.",
    "Vert Scale  3 7.",
    " .",
    "*"};
bool arrow[9][9] = {{0, 0, 0, 0, 0, 0, 0, 0, 0},
                    {0, 0, 0, 0, 1, 0, 0, 0, 0},
                    {0, 0, 0, 1, 1, 1, 0, 0, 0},
                    {0, 0, 1, 1, 1, 1, 1, 0, 0},
                    {0, 1, 1, 1, 1, 1, 1, 1, 0},
                    {0, 0, 0, 1, 1, 1, 0, 0, 0},
                    {0, 0, 0, 1, 1, 1, 0, 0, 0},
                    {0, 0, 0, 1, 1, 1, 0, 0, 0},
                    {0, 0, 0, 0, 0, 0, 0, 0, 0}};
bool buttonA[9][9] = {{0, 0, 0, 0, 0, 0, 0, 0, 0},
                      {0, 0, 1, 1, 1, 1, 0, 0, 0},
                      {0, 1, 1, 0, 0, 1, 1, 0, 0},
                      {1, 1, 0, 1, 1, 0, 1, 1, 0},
                      {1, 1, 0, 1, 1, 0, 1, 1, 0},
                      {1, 1, 0, 0, 0, 0, 1, 1, 0},
                      {1, 1, 0, 1, 1, 0, 1, 1, 0},
                      {0, 1, 0, 1, 1, 0, 1, 0, 0},
                      {0, 0, 1, 1, 1, 1, 0, 0, 0}};
bool buttonB[9][9] = {{0, 0, 0, 0, 0, 0, 0, 0, 0},
                      {0, 0, 1, 1, 1, 1, 0, 0, 0},
                      {0, 1, 0, 0, 0, 1, 1, 0, 0},
                      {1, 1, 0, 1, 1, 0, 1, 1, 0},
                      {1, 1, 0, 0, 0, 1, 1, 1, 0},
                      {1, 1, 0, 1, 1, 0, 1, 1, 0},
                      {1, 1, 0, 1, 1, 0, 1, 1, 0},
                      {0, 1, 0, 0, 0, 1, 1, 0, 0},
                      {0, 0, 1, 1, 1, 1, 0, 0, 0}};
bool disabled[9][9] = {{0, 0, 0, 0, 0, 0, 0, 0, 0},
                       {0, 0, 1, 1, 1, 1, 0, 0, 0},
                       {0, 1, 0, 0, 0, 0, 1, 0, 0},
                       {1, 0, 0, 0, 0, 1, 0, 1, 0},
                       {1, 0, 0, 0, 1, 0, 0, 1, 0},
                       {1, 0, 0, 1, 0, 0, 0, 1, 0},
                       {1, 0, 1, 0, 0, 0, 0, 1, 0},
                       {0, 1, 0, 0, 0, 0, 1, 0, 0},
                       {0, 0, 1, 1, 1, 1, 0, 0, 0}};
bool enabled[9][9] = {{0, 0, 0, 0, 0, 0, 0, 0, 1},
                      {0, 0, 0, 0, 0, 0, 0, 1, 1},
                      {0, 0, 0, 0, 0, 0, 1, 1, 0},
                      {0, 0, 0, 0, 0, 1, 1, 0, 0},
                      {1, 0, 0, 0, 1, 1, 0, 0, 0},
                      {1, 1, 0, 1, 1, 0, 0, 0, 0},
                      {0, 1, 1, 1, 0, 0, 0, 0, 0},
                      {0, 0, 1, 0, 0, 0, 0, 0, 0},
                      {0, 0, 0, 0, 0, 0, 0, 0, 0}};
bool scale[9][9] = {{0, 0, 0, 0, 0, 0, 0, 0, 0},
                    {0, 0, 0, 0, 0, 0, 1, 1, 0},
                    {0, 0, 0, 0, 0, 0, 1, 1, 0},
                    {0, 0, 0, 0, 1, 1, 1, 1, 0},
                    {0, 0, 0, 0, 1, 1, 1, 1, 0},
                    {0, 0, 1, 1, 1, 1, 1, 1, 0},
                    {0, 0, 1, 1, 1, 1, 1, 1, 0},
                    {1, 1, 1, 1, 1, 1, 1, 1, 0},
                    {1, 1, 1, 1, 1, 1, 1, 1, 0}};
static bool lineEnd;
static bool textEnd;
#define BRIGHTNESS '0'
#define A_BUTTON '1'
#define DOWN_ARROW '2'
#define B_BUTTON '3'
#define LEFT_ARROW '4'
#define HORIZ_SCALE '5'
#define RIGHT_ARROW '6'
#define VERT_SCALE '7'
#define UP_ARROW '8'
#define VOL_METER '9'
#define TURBO_A '@'
#define TURBO_B '$'
#define EOL_MARKER '.'
#define EOF_MARKER '*'

#define spiWrite(register, val)                                                                            \
    SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, register, SPI_USR_MOSI_DBITLEN_S); \
    WRITE_PERI_REG((SPI_W0_REG(SPI_NUM)), val);                                                            \
    SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);                                                      \
    waitForSPIReady();

#define RED (31<<11)	
#define GREEN (63<<5)
#define BLUE (0x0F)
#define LBLUE (0x3333)	
#define WHITE (0xFFFF)	
	
static uint16_t IRAM_ATTR renderInGameMenu(int x, int y, uint16_t x1, uint16_t y1, bool xStr)
{
    bool yStr = false;
	
	if (x < 32 || x > 286 || y < 34 || y > 206)
        return x1;	
    else if (x < 40 || x > 280 || y < 38 || y > 202)
        return BLUE;
	
    char actChar = ' ';
    if (y == 38)
        textEnd = 0;
    if (x == 40)
        lineEnd = 0;
    int line = (y - 38) / 18;
    int charNo = (x - 40) / 16;
    int xx = ((x - 40) % 16) / 2;
    int yy = ((y - 38) % 18) / 2;

    actChar = menuText[line][charNo];
    if (actChar == EOL_MARKER)
        lineEnd = 1;
    if (actChar == EOF_MARKER)
        textEnd = 1;
    if (lineEnd || textEnd)
        return BLUE;
    //printf("char %c, x = %d, y = %d{\n",actChar,x,y);
    //color c = [b](0to31)*1 + [g](0to31)*31 + [r] (0to31)*1024 +0x8000 --> x1=y1=c; !?
    switch (actChar)
    {
    case DOWN_ARROW:
        if (arrow[8 - yy][xx])
            return LBLUE;
        break;
    case LEFT_ARROW:
        if (arrow[xx][yy])
            return LBLUE;
        break;
    case RIGHT_ARROW:
        if (arrow[8 - xx][yy])
            return LBLUE;
        break;
    case UP_ARROW:
        if (arrow[yy][xx])
            return LBLUE;
        break;
    case A_BUTTON:
        if (buttonA[yy][xx])
            return LBLUE;
        break;
    case B_BUTTON:
        if (buttonB[yy][xx])
            return LBLUE;
        break;
    case HORIZ_SCALE:
        if (xStr && enabled[yy][xx])
            return GREEN;
        else if (!xStr && disabled[yy][xx])
            return RED;
        break;
    case VERT_SCALE:
		if (yStr && enabled[yy][xx])
            return GREEN;
        else if (!yStr && disabled[yy][xx])
            return RED;
        break;
    case BRIGHTNESS:
        if (scale[yy][xx])
        {
            setBrightness(getBright());
            return xx < (getBright() * 2) ? WHITE : LBLUE;
        }
        break;
    case VOL_METER:
        if (getVolume() == 0 && disabled[yy][xx])
            return RED;

        if (getVolume() > 0 && scale[yy][xx])
            return xx < (getVolume() * 2) ? WHITE : LBLUE;
        break;
    case TURBO_A:
        if (!getTurboA() && disabled[yy][xx])
            return RED;
        if (getTurboA() > 0 && scale[yy][xx])
            return xx < (getTurboA() * 2 - 1) ? WHITE : LBLUE;
        break;
    case TURBO_B:
        if (!getTurboB() && disabled[yy][xx])
            return RED;
        if (getTurboB() > 0 && scale[yy][xx])
            return xx < (getTurboB() * 2 - 1) ? WHITE : LBLUE;
        break;
    default:
        if ((actChar < 47 || actChar > 57) && peGetPixel(actChar, (x - 40) % 16, (y - 38) % 18))
            return WHITE;
        break;
    }
    return BLUE;
}

#ifdef FULL_SCREEN
uint16_t scaleX[SCREEN_WIDTH];
#endif

//***************************
//Clear whole screen to black
//***************************
void IRAM_ATTR ili9341_clr(void)
{
    int i, x, y;
    uint32_t xv, yv;
	uint32_t dc = (1 << PIN_NUM_DC);

	xv = U16x2toU32(0, SCREEN_WIDTH-1);
	yv = U16x2toU32(0, SCREEN_HEIGHT-1);

	waitForSPIReady();	//Wait for previous transfer to finish
	GPIO.out_w1tc = dc;
	spiWrite(7, 0x2a);	// Set column (command 0x2A - col address) to X start
	GPIO.out_w1ts = dc;
	spiWrite(31, xv);
	GPIO.out_w1tc = dc;
	spiWrite(7, 0x2b);	// Set row (command 0x2B - page address) to Y start
	GPIO.out_w1ts = dc;
	spiWrite(31, yv);
	GPIO.out_w1tc = dc;
	spiWrite(7, 0x2c);	// Send memory write command (command 0x2C)
	GPIO.out_w1ts = dc;
	SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 511, SPI_USR_MOSI_DBITLEN_S);

	for (y = 0; y < SCREEN_HEIGHT; y++)
    {
        x = 0;
        while (x < SCREEN_WIDTH)
        {
            // Render 32 pixels, grouped as pairs of 16-bit pixels stored in 32-bit values
            x += 32;
			
			waitForSPIReady();
			
			for (i = 0; i < 16; i++)
			{
				WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + (i << 2)), 0);	//clear to black
			}
            SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
        }
    }
}

static int lastShowMenu = 0;

#ifdef USE_SPI_DMA
extern spi_device_handle_t _spi;
static spi_transaction_t trans;

#ifdef INTERLACED_FRAMES
#define NUM_LINES 1
uint16_t DMA_buf[2][NUM_LINES * SCREEN_WIDTH];	//DMA double buffer for X lines on screen
//**********************************************************************************************************************************
// DMA writes an interlaced frame (even/odd lines) to the TFT one line at the time //Corn 
//**********************************************************************************************************************************
void IRAM_ATTR ili9341_write_frame(const uint16_t xs, const uint16_t ys, const uint16_t width, const uint16_t height, const uint8_t *data[], bool xStr)
{
	if (data == NULL)
		return;
	
	static bool first_frame = 1;
	static bool interlace = 0;
	int act = 0;
	int i, x, y;
	uint16_t x1, y1;
	uint32_t xv, yv;
	uint32_t dc = 1 << PIN_NUM_DC;

    if (getShowMenu() != lastShowMenu)
    {
		ili9341_clr();
    }
	
    lastShowMenu = getShowMenu();

#ifdef SHOW_SPI_TRANSFER_TIME
	uint32_t xStart = xTaskGetTickCount();	//SPI time
#endif

	if (!first_frame) spi_device_polling_end(_spi, &trans);	//make sure previous frame DMA is done

	//setup LCD SPI frame push
	x1 = xs + (width - 1);
	y1 = ys + (height - 1);
	xv = U16x2toU32(xs, x1);
	
	waitForSPIReady();
	GPIO.out_w1tc = dc;
	spiWrite(7, 0x2a);	// Set column (command 0x2a - col address) to X start
	GPIO.out_w1ts = dc;
	spiWrite(31, xv);

	//Transfer from NES palette to RGB565 and move to DMA buffer one lines at the time.
	for (y = ys+interlace; y < height+ys; y+=2)
	{
		if (getShowMenu())
		{
			for (x = 0; x < width; x+=2)
			{
				DMA_buf[act][x] = DMA_buf[act][x+1] = U16xtoZ16(renderInGameMenu(xs ? x+32 : x, y  , 0, 0, xStr));
			}
		}
#ifdef FULL_SCREEN
		else if (xStr)
		{
			for (x = 0; x < width; x++)
			{
				DMA_buf[act][x] = myPalette[data[y][scaleX[x]]];		//Convert from NES palette to RGB(565) format
			}
		}
#endif
		else
		{
			for (x = 0; x < width; x++)
			{
				DMA_buf[act][x] = myPalette[data[y][x]];	//Convert from NES palette to RGB(565) format
			}
		}
		
		if (y!=(ys+interlace)) spi_device_polling_end(_spi, &trans);	//Wait here for previous DMA to finish but skip first line

		yv = U16x2toU32(y, y);		GPIO.out_w1tc = dc;
		spiWrite(7, 0x2b);	// Set row (command 0x2B - page address) to Y start
		GPIO.out_w1ts = dc;
		spiWrite(31, yv);
		GPIO.out_w1tc = dc;
		spiWrite(7, 0x2c);	// Send memory write command
		GPIO.out_w1ts = dc;

		memset(&trans, 0, sizeof(spi_transaction_t));
		trans.user = (void *)1;					//Data mode
		trans.tx_buffer = (uint8_t*)DMA_buf[act];	//finally send the line data
		trans.length = NUM_LINES * width * 16;			//Data length, in bits
		trans.flags = 0;						//SPI_TRANS_USE_TXDATA flag

		//spi_device_polling_transmit(_spi, &trans);
		spi_device_polling_start(_spi, &trans, portMAX_DELAY);
		act ^=1;	//Swap DMA buffers
	}

	interlace ^= 1;
	
	first_frame = 0;
	
#ifdef SHOW_SPI_TRANSFER_TIME
	printf("%d\n", xTaskGetTickCount() - xStart);	//show frame transfer time in ms
#endif
    if (getShutdown())
        setBrightness(getBright());
#if PIN_NUM_BCKL >= 0
    if (getBright() == -1)
        LCD_BKG_OFF();
#endif	
}

#else	//INTERLACED_FRAMES

#define NUM_LINES 6
uint16_t DMA_buf[2][NUM_LINES * SCREEN_WIDTH];	//DMA double buffer for X lines on screen
//**********************************************************************************************************************************
//Using DMA to write a whole frame to the TFT 6 lines at the time //Corn 
//**********************************************************************************************************************************
void IRAM_ATTR ili9341_write_frame(const uint16_t xs, const uint16_t ys, const uint16_t width, uint16_t height, const uint8_t *data[], bool xStr)
{
	if (data == NULL)
		return;
	
	height -= 2;	//need to reduce screen by 2 lines to make it evenly divisable with 6 (assuming 224 is the height)
	
	static bool first_frame = 1;
	int act = 0;
	int i, x, y;
	uint16_t x1, y1;
	uint32_t xv, yv;
	uint32_t dc = 1 << PIN_NUM_DC;

    if (getShowMenu() != lastShowMenu)
    {
		ili9341_clr();
    }
	
    lastShowMenu = getShowMenu();

#ifdef SHOW_SPI_TRANSFER_TIME
	uint32_t xStart = xTaskGetTickCount();	//SPI time
#endif

	if (!first_frame) spi_device_polling_end(_spi, &trans);	//make sure previous frame DMA is done

	//setup LCD SPI frame push
	x1 = xs + (width - 1);
	y1 = ys + (height - 1);
	xv = U16x2toU32(xs, x1);
	yv = U16x2toU32(ys, y1);
	
	waitForSPIReady();
	GPIO.out_w1tc = dc;
	spiWrite(7, 0x2a);	// Set column (command 0x2a - col address) to X start
	GPIO.out_w1ts = dc;
	spiWrite(31, xv);
	GPIO.out_w1tc = dc;
	spiWrite(7, 0x2b);	// Set row (command 0x2B - page address) to Y start
	GPIO.out_w1ts = dc;
	spiWrite(31, yv);
	GPIO.out_w1tc = dc;
	spiWrite(7, 0x2c);	// Send memory write command
	GPIO.out_w1ts = dc;

	//Transfer from NES palette to RGB565 and move to DMA buffer six lines at the time.
	for (y = ys; y < height+ys; y+=NUM_LINES)
	{
		if (getShowMenu())
		{
			for (x = 0; x < width; x+=2)
			{
				DMA_buf[act][x]         = DMA_buf[act][x+1]         = U16xtoZ16(renderInGameMenu(xs ? x+32 : x, y  , 0, 0, xStr));
				DMA_buf[act][x+width]   = DMA_buf[act][x+1+width]   = U16xtoZ16(renderInGameMenu(xs ? x+32 : x, y+1, 0, 0, xStr));
				DMA_buf[act][x+width*2] = DMA_buf[act][x+1+width*2] = U16xtoZ16(renderInGameMenu(xs ? x+32 : x, y+2, 0, 0, xStr));
				DMA_buf[act][x+width*3] = DMA_buf[act][x+1+width*3] = U16xtoZ16(renderInGameMenu(xs ? x+32 : x, y+3, 0, 0, xStr));
				DMA_buf[act][x+width*4] = DMA_buf[act][x+1+width*4] = U16xtoZ16(renderInGameMenu(xs ? x+32 : x, y+4, 0, 0, xStr));
				DMA_buf[act][x+width*5] = DMA_buf[act][x+1+width*5] = U16xtoZ16(renderInGameMenu(xs ? x+32 : x, y+5, 0, 0, xStr));
			}
		}
#ifdef FULL_SCREEN
		else if (xStr)
		{
			for (x = 0; x < width; x++)
			{
				DMA_buf[act][x]                = myPalette[data[y][scaleX[x]]];		//Convert from NES palette to RGB(565) format
				DMA_buf[act][x+SCREEN_WIDTH]   = myPalette[data[y+1][scaleX[x]]];	//Convert from NES palette to RGB(565) format
				DMA_buf[act][x+SCREEN_WIDTH*2] = myPalette[data[y+2][scaleX[x]]];	//Convert from NES palette to RGB(565) format
				DMA_buf[act][x+SCREEN_WIDTH*3] = myPalette[data[y+3][scaleX[x]]];	//Convert from NES palette to RGB(565) format
				DMA_buf[act][x+SCREEN_WIDTH*4] = myPalette[data[y+4][scaleX[x]]];	//Convert from NES palette to RGB(565) format
				DMA_buf[act][x+SCREEN_WIDTH*5] = myPalette[data[y+5][scaleX[x]]];	//Convert from NES palette to RGB(565) format
			}
		}
#endif
		else
		{
			for (x = 0; x < width; x++)
			{
				DMA_buf[act][x]             = myPalette[data[y][x]];	//Convert from NES palette to RGB(565) format
				DMA_buf[act][x+NES_WIDTH]   = myPalette[data[y+1][x]];	//Convert from NES palette to RGB(565) format
				DMA_buf[act][x+NES_WIDTH*2] = myPalette[data[y+2][x]];	//Convert from NES palette to RGB(565) format
				DMA_buf[act][x+NES_WIDTH*3] = myPalette[data[y+3][x]];	//Convert from NES palette to RGB(565) format
				DMA_buf[act][x+NES_WIDTH*4] = myPalette[data[y+4][x]];	//Convert from NES palette to RGB(565) format
				DMA_buf[act][x+NES_WIDTH*5] = myPalette[data[y+5][x]];	//Convert from NES palette to RGB(565) format
			}
		}
		
		if (y!=ys) spi_device_polling_end(_spi, &trans);	//Wait here for previous DMA to finish but skip first line

		memset(&trans, 0, sizeof(spi_transaction_t));
		trans.user = (void *)1;					//Data mode
		trans.tx_buffer = (uint8_t*)DMA_buf[act];	//finally send the line data
		trans.length = NUM_LINES * width * 16;			//Data length, in bits
		trans.flags = 0;						//SPI_TRANS_USE_TXDATA flag

		//spi_device_polling_transmit(_spi, &trans);
		spi_device_polling_start(_spi, &trans, portMAX_DELAY);
		act ^=1;	//Swap DMA buffers
	}
	
	first_frame = 0;
#ifdef SHOW_SPI_TRANSFER_TIME
	printf("%d\n", xTaskGetTickCount() - xStart);	//show frame transfer time in ms
#endif
    if (getShutdown())
        setBrightness(getBright());
#if PIN_NUM_BCKL >= 0
    if (getBright() == -1)
        LCD_BKG_OFF();
#endif	
}
#endif	//INTERLACED_FRAMES

#else //USE_SPI_DMA

#ifdef INTERLACED_FRAMES
//**********************************************************************************************************************************
//The CPU writes an interlaced frame (even/odd lines) to the TFT in 32 16bit pixel chunks making sure to minimize wating time //Corn 
//**********************************************************************************************************************************
void IRAM_ATTR ili9341_write_frame(const uint16_t xs, const uint16_t ys, const uint16_t width, const uint16_t height, const uint8_t *data[], bool xStr)
{
    static bool interlace = 0;
	int i, x, y;
    uint16_t x1, y1, evenPixel, oddPixel;
    uint32_t xv, yv;
	uint32_t dc = 1 << PIN_NUM_DC;
    uint32_t temp[16];
    
	if (data == NULL)
        return;

    if (getShowMenu() != lastShowMenu)
    {
		ili9341_clr();
    }
	
    lastShowMenu = getShowMenu();

#ifdef SHOW_SPI_TRANSFER_TIME
	uint32_t xStart = xTaskGetTickCount();	//SPI time
#endif
	
	//setup LCD SPI pixel push
	x1 = xs + (width - 1);
	y1 = ys + (height - 1);
	xv = U16x2toU32(xs, x1);
	
	waitForSPIReady();	//Wait for previous transfer to finish
	GPIO.out_w1tc = dc;
	spiWrite(7, 0x2a);	// Set column (command 0x2A - col address) to X start
	GPIO.out_w1ts = dc;
	spiWrite(31, xv);

	for (y = ys+interlace; y < height+ys; y+=2)
    {
		waitForSPIReady();	//Wait for previous transfer to finish
		yv = U16x2toU32(y, y);
		GPIO.out_w1tc = dc;
		spiWrite(7, 0x2b);	// Set row (command 0x2B - page address) to Y start
		GPIO.out_w1ts = dc;
		spiWrite(31, yv);
		GPIO.out_w1tc = dc;
		spiWrite(7, 0x2c);	// Send memory write command (command 0x2C)
		GPIO.out_w1ts = dc;
		SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 511, SPI_USR_MOSI_DBITLEN_S);

		x = 0;
        while (x < width)
        {
            // Render 32 pixels, grouped as pairs of 16-bit pixels stored in 32-bit values
			if (getShowMenu())
			{
				for (i = 0; i < 16; i++)
                {
                    evenPixel = oddPixel = renderInGameMenu(xs ? x+32 : x, y, 0, 0, xStr);
					temp[i] = U16x2toU32(evenPixel, oddPixel);
					x+=2;
                }
			}
#ifdef FULL_SCREEN
			else if (xStr)			
			{
				//Color from palette is already byte swizzled
				temp[0] =  myPalette[data[y][scaleX[x++]]] | (myPalette[data[y][scaleX[x++]]] << 16);
				temp[1] =  myPalette[data[y][scaleX[x++]]] | (myPalette[data[y][scaleX[x++]]] << 16);
				temp[2] =  myPalette[data[y][scaleX[x++]]] | (myPalette[data[y][scaleX[x++]]] << 16);
				temp[3] =  myPalette[data[y][scaleX[x++]]] | (myPalette[data[y][scaleX[x++]]] << 16);
				temp[4] =  myPalette[data[y][scaleX[x++]]] | (myPalette[data[y][scaleX[x++]]] << 16);
				temp[5] =  myPalette[data[y][scaleX[x++]]] | (myPalette[data[y][scaleX[x++]]] << 16);
				temp[6] =  myPalette[data[y][scaleX[x++]]] | (myPalette[data[y][scaleX[x++]]] << 16);
				temp[7] =  myPalette[data[y][scaleX[x++]]] | (myPalette[data[y][scaleX[x++]]] << 16);
				temp[8] =  myPalette[data[y][scaleX[x++]]] | (myPalette[data[y][scaleX[x++]]] << 16);
				temp[9] =  myPalette[data[y][scaleX[x++]]] | (myPalette[data[y][scaleX[x++]]] << 16);
				temp[10] = myPalette[data[y][scaleX[x++]]] | (myPalette[data[y][scaleX[x++]]] << 16);
				temp[11] = myPalette[data[y][scaleX[x++]]] | (myPalette[data[y][scaleX[x++]]] << 16);
				temp[12] = myPalette[data[y][scaleX[x++]]] | (myPalette[data[y][scaleX[x++]]] << 16);
				temp[13] = myPalette[data[y][scaleX[x++]]] | (myPalette[data[y][scaleX[x++]]] << 16);
				temp[14] = myPalette[data[y][scaleX[x++]]] | (myPalette[data[y][scaleX[x++]]] << 16);
				temp[15] = myPalette[data[y][scaleX[x++]]] | (myPalette[data[y][scaleX[x++]]] << 16);
			}
#endif			
			else
			{	//Color from palette is already byte swizzled
				temp[0] =  myPalette[data[y][x++]] | (myPalette[data[y][x++]] << 16);
				temp[1] =  myPalette[data[y][x++]] | (myPalette[data[y][x++]] << 16);
				temp[2] =  myPalette[data[y][x++]] | (myPalette[data[y][x++]] << 16);
				temp[3] =  myPalette[data[y][x++]] | (myPalette[data[y][x++]] << 16);
				temp[4] =  myPalette[data[y][x++]] | (myPalette[data[y][x++]] << 16);
				temp[5] =  myPalette[data[y][x++]] | (myPalette[data[y][x++]] << 16);
				temp[6] =  myPalette[data[y][x++]] | (myPalette[data[y][x++]] << 16);
				temp[7] =  myPalette[data[y][x++]] | (myPalette[data[y][x++]] << 16);
				temp[8] =  myPalette[data[y][x++]] | (myPalette[data[y][x++]] << 16);
				temp[9] =  myPalette[data[y][x++]] | (myPalette[data[y][x++]] << 16);
				temp[10] = myPalette[data[y][x++]] | (myPalette[data[y][x++]] << 16);
				temp[11] = myPalette[data[y][x++]] | (myPalette[data[y][x++]] << 16);
				temp[12] = myPalette[data[y][x++]] | (myPalette[data[y][x++]] << 16);
				temp[13] = myPalette[data[y][x++]] | (myPalette[data[y][x++]] << 16);
				temp[14] = myPalette[data[y][x++]] | (myPalette[data[y][x++]] << 16);
				temp[15] = myPalette[data[y][x++]] | (myPalette[data[y][x++]] << 16);
            }
	
			//Special trick here so we can patially fill the SPI TX buffer and start the transfer early //Corn
			taskDISABLE_INTERRUPTS();	//globally disable all maskable interrupts
            
			waitForSPIReady();	//Wait for previous transfer to finish
			
            WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + 0), temp[0]);
            SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);	//Start SPI transfer early and continue to fill the SPI TX buffer to save time
            WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + 4), temp[1]);
            WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + 8), temp[2]);
            WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + 12), temp[3]);
            WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + 16), temp[4]);
            WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + 20), temp[5]);
            WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + 24), temp[6]);
            WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + 28), temp[7]);
            WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + 32), temp[8]);
            WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + 36), temp[9]);
            WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + 40), temp[10]);
            WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + 44), temp[11]);
            WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + 48), temp[12]);
            WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + 52), temp[13]);
            WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + 56), temp[14]);
            WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + 60), temp[15]);
			
			taskENABLE_INTERRUPTS();	//globally enable all maskable interrupts
        }
    }

	interlace ^= 1;
	
#ifdef SHOW_SPI_TRANSFER_TIME
	printf("%d\n", xTaskGetTickCount() - xStart);	//show frame transfer time in ms
#endif
	 
    if (getShutdown())
        setBrightness(getBright());
#if PIN_NUM_BCKL >= 0
    if (getBright() == -1)
        LCD_BKG_OFF();
#endif
}

#else	//INTERLACED_FRAMES

//**********************************************************************************************************************************
//The CPU writes the whole frame to the TFT in 32 16bit pixel chunks making sure to minimize wating time //Corn 
//**********************************************************************************************************************************
void IRAM_ATTR ili9341_write_frame(const uint16_t xs, const uint16_t ys, const uint16_t width, const uint16_t height, const uint8_t *data[], bool xStr)
{
    int i, x, y;
    uint16_t x1, y1, evenPixel, oddPixel;
    uint32_t xv, yv;
	uint32_t dc = 1 << PIN_NUM_DC;
    uint32_t temp[16];
    
	if (data == NULL)
        return;

    if (getShowMenu() != lastShowMenu)
    {
		ili9341_clr();
    }
	
    lastShowMenu = getShowMenu();

#ifdef SHOW_SPI_TRANSFER_TIME
	uint32_t xStart = xTaskGetTickCount();	//SPI time
#endif
	
	//setup LCD SPI pixel push
	x1 = xs + (width - 1);
	y1 = ys + (height - 1);
	xv = U16x2toU32(xs, x1);
	yv = U16x2toU32(ys, y1);
	
	waitForSPIReady();	//Wait for previous transfer to finish
	GPIO.out_w1tc = dc;
	spiWrite(7, 0x2a);	// Set column (command 0x2A - col address) to X start
	GPIO.out_w1ts = dc;
	spiWrite(31, xv);
	GPIO.out_w1tc = dc;
	spiWrite(7, 0x2b);	// Set row (command 0x2B - page address) to Y start
	GPIO.out_w1ts = dc;
	spiWrite(31, yv);
	GPIO.out_w1tc = dc;
	spiWrite(7, 0x2c);	// Send memory write command (command 0x2C)
	GPIO.out_w1ts = dc;
	SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 511, SPI_USR_MOSI_DBITLEN_S);

	for (y = ys; y < height+ys; y++)
    {
        x = 0;
        while (x < width)
        {
            // Render 32 pixels, grouped as pairs of 16-bit pixels stored in 32-bit values
			if (getShowMenu())
			{
				for (i = 0; i < 16; i++)
                {
                    evenPixel = oddPixel = renderInGameMenu(xs ? x+32 : x, y, 0, 0, xStr);
					temp[i] = U16x2toU32(evenPixel, oddPixel);
					x+=2;
                }
			}
#ifdef FULL_SCREEN
			else if (xStr)			
			{
				//Color from palette is already byte swizzled
				temp[0] =  myPalette[data[y][scaleX[x++]]] | (myPalette[data[y][scaleX[x++]]] << 16);
				temp[1] =  myPalette[data[y][scaleX[x++]]] | (myPalette[data[y][scaleX[x++]]] << 16);
				temp[2] =  myPalette[data[y][scaleX[x++]]] | (myPalette[data[y][scaleX[x++]]] << 16);
				temp[3] =  myPalette[data[y][scaleX[x++]]] | (myPalette[data[y][scaleX[x++]]] << 16);
				temp[4] =  myPalette[data[y][scaleX[x++]]] | (myPalette[data[y][scaleX[x++]]] << 16);
				temp[5] =  myPalette[data[y][scaleX[x++]]] | (myPalette[data[y][scaleX[x++]]] << 16);
				temp[6] =  myPalette[data[y][scaleX[x++]]] | (myPalette[data[y][scaleX[x++]]] << 16);
				temp[7] =  myPalette[data[y][scaleX[x++]]] | (myPalette[data[y][scaleX[x++]]] << 16);
				temp[8] =  myPalette[data[y][scaleX[x++]]] | (myPalette[data[y][scaleX[x++]]] << 16);
				temp[9] =  myPalette[data[y][scaleX[x++]]] | (myPalette[data[y][scaleX[x++]]] << 16);
				temp[10] = myPalette[data[y][scaleX[x++]]] | (myPalette[data[y][scaleX[x++]]] << 16);
				temp[11] = myPalette[data[y][scaleX[x++]]] | (myPalette[data[y][scaleX[x++]]] << 16);
				temp[12] = myPalette[data[y][scaleX[x++]]] | (myPalette[data[y][scaleX[x++]]] << 16);
				temp[13] = myPalette[data[y][scaleX[x++]]] | (myPalette[data[y][scaleX[x++]]] << 16);
				temp[14] = myPalette[data[y][scaleX[x++]]] | (myPalette[data[y][scaleX[x++]]] << 16);
				temp[15] = myPalette[data[y][scaleX[x++]]] | (myPalette[data[y][scaleX[x++]]] << 16);
			}
#endif			
			else
			{	//Color from palette is already byte swizzled
				temp[0] =  myPalette[data[y][x++]] | (myPalette[data[y][x++]] << 16);
				temp[1] =  myPalette[data[y][x++]] | (myPalette[data[y][x++]] << 16);
				temp[2] =  myPalette[data[y][x++]] | (myPalette[data[y][x++]] << 16);
				temp[3] =  myPalette[data[y][x++]] | (myPalette[data[y][x++]] << 16);
				temp[4] =  myPalette[data[y][x++]] | (myPalette[data[y][x++]] << 16);
				temp[5] =  myPalette[data[y][x++]] | (myPalette[data[y][x++]] << 16);
				temp[6] =  myPalette[data[y][x++]] | (myPalette[data[y][x++]] << 16);
				temp[7] =  myPalette[data[y][x++]] | (myPalette[data[y][x++]] << 16);
				temp[8] =  myPalette[data[y][x++]] | (myPalette[data[y][x++]] << 16);
				temp[9] =  myPalette[data[y][x++]] | (myPalette[data[y][x++]] << 16);
				temp[10] = myPalette[data[y][x++]] | (myPalette[data[y][x++]] << 16);
				temp[11] = myPalette[data[y][x++]] | (myPalette[data[y][x++]] << 16);
				temp[12] = myPalette[data[y][x++]] | (myPalette[data[y][x++]] << 16);
				temp[13] = myPalette[data[y][x++]] | (myPalette[data[y][x++]] << 16);
				temp[14] = myPalette[data[y][x++]] | (myPalette[data[y][x++]] << 16);
				temp[15] = myPalette[data[y][x++]] | (myPalette[data[y][x++]] << 16);
            }
	
			//Special trick here so we can patially fill the SPI TX buffer and start the transfer early //Corn
			taskDISABLE_INTERRUPTS();	//globally disable all maskable interrupts
            
			waitForSPIReady();	//Wait for previous transfer to finish
			
            WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + 0), temp[0]);
            SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);	//Start SPI transfer early and continue to fill the SPI TX buffer to save time
            WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + 4), temp[1]);
            WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + 8), temp[2]);
            WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + 12), temp[3]);
            WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + 16), temp[4]);
            WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + 20), temp[5]);
            WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + 24), temp[6]);
            WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + 28), temp[7]);
            WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + 32), temp[8]);
            WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + 36), temp[9]);
            WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + 40), temp[10]);
            WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + 44), temp[11]);
            WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + 48), temp[12]);
            WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + 52), temp[13]);
            WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + 56), temp[14]);
            WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + 60), temp[15]);
			
			taskENABLE_INTERRUPTS();	//globally enable all maskable interrupts
        }
    }

#ifdef SHOW_SPI_TRANSFER_TIME
	printf("%d\n", xTaskGetTickCount() - xStart);	//show frame transfer time in ms
#endif
	 
    if (getShutdown())
        setBrightness(getBright());
#if PIN_NUM_BCKL >= 0
    if (getBright() == -1)
        LCD_BKG_OFF();
#endif
}
#endif //INTERLACED_FRAMES
#endif	//USE_SPI_DMA

#ifdef FULL_SCREEN
void precalculateLookupTables()
{
    for (int i = 0; i < SCREEN_WIDTH; i++)
    {
        scaleX[i] = round(i * NES_WIDTH / SCREEN_WIDTH);
    }
}
#endif

void ili9341_init()
{
    lineEnd = textEnd = 0;
    //spi_master_init();
    //ili_gpio_init();
    //ILI9341_INITIAL();

#if PIN_NUM_BCKL >= 0
    LCD_BKG_ON();
    initBCKL();
#endif

#ifdef FULL_SCREEN	
    precalculateLookupTables();
#endif
}
