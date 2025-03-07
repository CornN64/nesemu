/* SPI Master example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "pretty_effect.h"
#include "driver/ledc.h"
#include "esp_spiffs.h"
#include "menu.h"
#include "../config.h"

/*
 This code displays some fancy graphics on the 320x240 LCD on an ESP-WROVER_KIT board.
 This example demonstrates the use of both spi_device_transmit as well as
 spi_device_queue_trans/spi_device_get_trans_result and pre-transmit callbacks.

 Some info about the ILI9341/ST7789V: It has an C/D line, which is connected to a GPIO here. It expects this
 line to be low for a command and high for data. We use a pre-transmit callback here to control that
 line: every transaction has as the user-definable argument the needed state of the D/C line and just
 before the transaction is sent, the callback will set this line to the correct state.
*/

#define PIN_NUM_MISO CONFIG_HW_LCD_MISO_GPIO
#define PIN_NUM_MOSI CONFIG_HW_LCD_MOSI_GPIO
#define PIN_NUM_CLK CONFIG_HW_LCD_CLK_GPIO
#define PIN_NUM_CS CONFIG_HW_LCD_CS_GPIO

#define PIN_NUM_DC CONFIG_HW_LCD_DC_GPIO
#define PIN_NUM_RST CONFIG_HW_LCD_RESET_GPIO
#define PIN_NUM_BCKL CONFIG_HW_LCD_BL_GPIO

#define LEDC_LS_TIMER LEDC_TIMER_1
#define LEDC_LS_MODE LEDC_HIGH_SPEED_MODE
#define LEDC_LS_CH3_CHANNEL LEDC_CHANNEL_3

#define LCD_BKG_ON() GPIO.out_w1tc = (1 << PIN_NUM_BCKL)  // Backlight ON
#define LCD_BKG_OFF() GPIO.out_w1ts = (1 << PIN_NUM_BCKL) //Backlight OFF
//To speed up transfers, every SPI transfer sends a bunch of rows. This define specifies how many. More means more memory use,
//but less overhead for setting up / finishing transfers. Make sure 240 is dividable by this.
#define FRAMEBUFFER_HEIGHT 4

#define NO_ROM_SELECTED -1

/*
 The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct.
*/
typedef struct
{
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

/*typedef enum
{
    LCD_TYPE_ILI = 1,
    LCD_TYPE_ST,
    LCD_TYPE_MAX,
} type_lcd_t;*/

//Place data into DRAM. Constant data gets placed into DROM by default, which is not accessible by DMA.
DRAM_ATTR static const lcd_init_cmd_t st_init_cmds[] = {
    /* Memory Data Access Control, MX=MV=1, MY=ML=MH=0, RGB=0 */
    {0x36, {(1 << 5) | (1 << 6)}, 1},
    /* Interface Pixel Format, 16bits/pixel for RGB/MCU interface */
    {0x3A, {0x55}, 1},
    /* Porch Setting */
    {0xB2, {0x0c, 0x0c, 0x00, 0x33, 0x33}, 5},
    /* Gate Control, Vgh=13.65V, Vgl=-10.43V */
    {0xB7, {0x45}, 1},
    /* VCOM Setting, VCOM=1.175V */
    {0xBB, {0x2B}, 1},
    /* LCM Control, XOR: BGR, MX, MH */
    {0xC0, {0x2C}, 1},
    /* VDV and VRH Command Enable, enable=1 */
    {0xC2, {0x01, 0xff}, 2},
    /* VRH Set, Vap=4.4+... */
    {0xC3, {0x11}, 1},
    /* VDV Set, VDV=0 */
    {0xC4, {0x20}, 1},
    /* Frame Rate Control, 60Hz, inversion=0 */
    {0xC6, {0x0f}, 1},
    /* Power Control 1, AVDD=6.8V, AVCL=-4.8V, VDDS=2.3V */
    {0xD0, {0xA4, 0xA1}, 1},
    /* Positive Voltage Gamma Control */
    {0xE0, {0xD0, 0x00, 0x05, 0x0E, 0x15, 0x0D, 0x37, 0x43, 0x47, 0x09, 0x15, 0x12, 0x16, 0x19}, 14},
    /* Negative Voltage Gamma Control */
    {0xE1, {0xD0, 0x00, 0x05, 0x0D, 0x0C, 0x06, 0x2D, 0x44, 0x40, 0x0E, 0x1C, 0x18, 0x16, 0x19}, 14},
    /* Sleep Out */
    {0x11, {0}, 0x80},
    /* Display On */
    {0x29, {0}, 0x80},
    {0, {0}, 0xff}};

#define ILI9341_FRAMERATE_61_HZ 0x1F
#define ILI9341_FRAMERATE_63_HZ 0x1E
#define ILI9341_FRAMERATE_65_HZ 0x1D
#define ILI9341_FRAMERATE_68_HZ 0x1C
#define ILI9341_FRAMERATE_70_HZ 0x1B
#define ILI9341_FRAMERATE_73_HZ 0x1A
#define ILI9341_FRAMERATE_76_HZ 0x19
#define ILI9341_FRAMERATE_79_HZ 0x18
#define ILI9341_FRAMERATE_83_HZ 0x17
#define ILI9341_FRAMERATE_86_HZ 0x16
#define ILI9341_FRAMERATE_90_HZ 0x15
#define ILI9341_FRAMERATE_95_HZ 0x14
#define ILI9341_FRAMERATE_100_HZ 0x13
#define ILI9341_FRAMERATE_106_HZ 0x12
#define ILI9341_FRAMERATE_112_HZ 0x11
#define ILI9341_FRAMERATE_119_HZ 0x10
	
DRAM_ATTR static const lcd_init_cmd_t ili_init_cmds[] = {
    /* Power control A, Vcore=1.6V, DDVDH=5.6V */
    {0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},
    /* Power contorl B, power control = 0, DC_ENA = 1 */
    //{0xCF, {0x00, 0x83, 0X30}, 3},
    {0xCF, {0x00, 0xC1, 0X30}, 3},
    /* Power on sequence control,
     * cp1 keeps 1 frame, 1st frame enable
     * vcl = 0, ddvdh=3, vgh=1, vgl=2
     * DDVDH_ENH=1
     */
    {0xED, {0x64, 0x03, 0X12, 0X81}, 4},
    /* Driver timing control A,
     * non-overlap=default +1
     * EQ=default - 1, CR=default
     * pre-charge=default - 1
     */
    //{0xE8, {0x85, 0x01, 0x79}, 3},
    {0xE8, {0x85, 0x01, 0x78}, 3},
    /* Pump ratio control, DDVDH=2xVCl(0x20) or 3XVCI(0x30) only for ili9241 */
    {0xF7, {0x30}, 1},
    /* Driver timing control, all=0 unit */
    {0xEA, {0x00, 0x00}, 2},
    /* Power control 1, GVDD=4.75V */
    //{0xC0, {0x26}, 1},
    {0xC0, {0x23}, 1},
    /* Power control 2, DDVDH=VCl*2, VGH=VCl*7, VGL=-VCl*3 */
    //{0xC1, {0x11}, 1},
    {0xC1, {0x10}, 1},
    /* VCOM control 1, VCOMH=4.025V, VCOML=-0.950V */
    //{0xC5, {0x35, 0x3E}, 2},
    {0xC5, {0x3E, 0x28}, 2},
    /* VCOM control 2, VCOMH=VMH-2, VCOML=VML-2 */
    //{0xC7, {0xBE}, 1},
    {0xC7, {0x86}, 1},
    /* Memory access contorl, MX=MY=0, MV=1, ML=0, BGR=1, MH=0 */
#ifdef ROTATE_LCD_180
    {0x36, {0x28^0xC0}, 1},
#else
    {0x36, {0x28}, 1},
#endif
	/* Pixel format, 16bits/pixel for RGB/MCU interface */
    {0x3A, {0x55}, 1},
    /* Frame rate control*/
    {0xB1, {0x00, ILI9341_FRAMERATE_119_HZ}, 2},	//Note that this refers to the ILI internal update rate not the actual pushed frames rate
    /* Enable 3G, disabled */
    {0xF2, {0x08}, 1},
    /* Gamma set, curve 1 */
    {0x26, {0x01}, 1},
    /* Positive gamma correction */
    //{0xE0, {0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0X87, 0x32, 0x0A, 0x07, 0x02, 0x07, 0x05, 0x00}, 15},
    {0xE0, {0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00}, 15},
    /* Negative gamma correction */
    //{0XE1, {0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D, 0x38, 0x3A, 0x1F}, 15},
    {0XE1, {0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F}, 15},
    /* Column address set, SC=0, EC=0xEF */
    {0x2A, {0x00, 0x00, 0x00, 0xEF}, 4},
    /* Page address set, SP=0, EP=0x013F */
    {0x2B, {0x00, 0x00, 0x01, 0x3f}, 4},
    /* Memory write */
    //{0x2C, {0}, 0},
    /* Entry mode set, Low vol detect disabled, normal display */
    {0xB7, {0x07}, 1},
    /* Display function control */
    //{0xB6, {0x0A, 0x82, 0x27, 0x00}, 4},
    {0xB6, {0x08, 0x82, 0x27}, 3},
    /* Sleep out */
    {0x11, {0}, 0x80},
    /* Display on */
    {0x29, {0}, 0x80},
    {0, {0}, 0xff},
};

//Send a command to the LCD. Uses spi_device_transmit, which waits until the transfer is complete.
void lcd_cmd(spi_device_handle_t spi, const uint8_t cmd)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));           //Zero out the transaction
    t.length = 8;                       //Command is 8 bits
    t.tx_buffer = &cmd;                 //The data is the cmd itself
    t.user = (void *)0;                 //D/C needs to be set to 0
    ret = spi_device_transmit(spi, &t); //Transmit!
    assert(ret == ESP_OK);              //Should have had no issues.
}

//Send data to the LCD. Uses spi_device_transmit, which waits until the transfer is complete.
void lcd_data(spi_device_handle_t spi, const uint8_t *data, int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len == 0)
        return;                         //no need to send anything
    memset(&t, 0, sizeof(t));           //Zero out the transaction
    t.length = len * 8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer = data;                 //Data
    t.user = (void *)1;                 //D/C needs to be set to 1
    ret = spi_device_transmit(spi, &t); //Transmit!
    assert(ret == ESP_OK);              //Should have had no issues.
}

//This function is called (in irq context!) just before a transmission starts. It will
//set the D/C line to the value indicated in the user field.
void IRAM_ATTR lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc = (int)t->user;
    gpio_set_level(PIN_NUM_DC, dc);
}

uint32_t lcd_get_id(spi_device_handle_t spi)
{
    //get_id cmd
    lcd_cmd(spi, 0x04);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8 * 3;
    t.flags = SPI_TRANS_USE_RXDATA;
    t.user = (void *)1;

    esp_err_t ret = spi_device_transmit(spi, &t);
    assert(ret == ESP_OK);

    return *(uint32_t *)t.rx_data;
}

//Initialize the display
void lcd_init(spi_device_handle_t spi)
{
    int cmd = 0;
    const lcd_init_cmd_t *lcd_init_cmds;

    //Initialize non-SPI GPIOs
    gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
#if PIN_NUM_RST >= 0
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
#endif
    //PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[27], PIN_FUNC_GPIO);
#if PIN_NUM_BCKL >= 0
    gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);
#endif

    //Reset the display
#if PIN_NUM_RST >= 0
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(100 / portTICK_RATE_MS);
#endif

    //detect LCD type
    uint32_t lcd_id = lcd_get_id(spi);
    int lcd_detected_type = 0;
    int lcd_type;

    //    printf("LCD ID: %08X\n", lcd_id);
    //    if ( lcd_id == 0 ) {
    //zero, ili
    lcd_detected_type = LCD_TYPE_ILI;
    /*        printf("ILI9341 detected.\n");
    } else {
        // none-zero, ST
        lcd_detected_type = LCD_TYPE_ST;
        printf("ST7789V detected.\n");
    }

#ifdef CONFIG_LCD_TYPE_AUTO
    lcd_type = lcd_detected_type;
#elif defined( CONFIG_LCD_TYPE_ST7789V )
    printf("kconfig: force CONFIG_LCD_TYPE_ST7789V.\n");
    lcd_type = LCD_TYPE_ST;
#elif defined( CONFIG_LCD_TYPE_ILI9341 )
    printf("kconfig: force CONFIG_LCD_TYPE_ILI9341.\n");*/
    lcd_type = LCD_TYPE_ILI;
    //#endif
    if (lcd_type == LCD_TYPE_ST)
    {
        printf("LCD ST7789V initialization.\n");
        lcd_init_cmds = st_init_cmds;
    }
    else
    {
        printf("LCD ILI9341 initialization.\n");
        lcd_init_cmds = ili_init_cmds;
    }

    //Send all the commands
    while (lcd_init_cmds[cmd].databytes != 0xff)
    {
        lcd_cmd(spi, lcd_init_cmds[cmd].cmd);
        lcd_data(spi, lcd_init_cmds[cmd].data, lcd_init_cmds[cmd].databytes & 0x1F);
        if (lcd_init_cmds[cmd].databytes == 0x80)
        {
            vTaskDelay(100 / portTICK_RATE_MS);
        }
        cmd++;
    }
    printf("LCD initialization complete.\n");

    ///Enable backlight
    //gpio_set_level(17, 1);
}

//To send a set of lines we have to send a command, 2 data bytes, another command, 2 more data bytes and another command
//before sending the line data itself; a total of 6 transactions. (We can't put all of this in just one transaction
//because the D/C line needs to be toggled in the middle.)
//This routine queues these commands up so they get sent as quickly as possible.
static void send_lines(spi_device_handle_t spi, int ypos, uint16_t *linedata)
{
    esp_err_t ret;
    int x;
    //Transaction descriptors. Declared static so they're not allocated on the stack; we need this memory even when this
    //function is finished because the SPI driver needs access to it even while we're already calculating the next line.
    static spi_transaction_t trans[6];

    //In theory, it's better to initialize trans and data only once and hang on to the initialized
    //variables. We allocate them on the stack, so we need to re-init them each call.
    for (x = 0; x < 6; x++)
    {
        memset(&trans[x], 0, sizeof(spi_transaction_t));
        if ((x & 1) == 0)
        {
            //Even transfers are commands
            trans[x].length = 8;
            trans[x].user = (void *)0;
        }
        else
        {
            //Odd transfers are data
            trans[x].length = 8 * 4;
            trans[x].user = (void *)1;
        }
        trans[x].flags = SPI_TRANS_USE_TXDATA;
    }
    trans[0].tx_data[0] = 0x2A;                               //Column Address Set
    trans[1].tx_data[0] = 0;                                  //Start Col High
    trans[1].tx_data[1] = 0;                                  //Start Col Low
    trans[1].tx_data[2] = (SCREEN_WIDTH) >> 8;                //End Col High
    trans[1].tx_data[3] = (SCREEN_WIDTH) & 0xff;              //End Col Low
    trans[2].tx_data[0] = 0x2B;                               //Page address set
    trans[3].tx_data[0] = ypos >> 8;                          //Start page high
    trans[3].tx_data[1] = ypos & 0xff;                        //start page low
    trans[3].tx_data[2] = (ypos + FRAMEBUFFER_HEIGHT) >> 8;   //end page high
    trans[3].tx_data[3] = (ypos + FRAMEBUFFER_HEIGHT) & 0xff; //end page low
    trans[4].tx_data[0] = 0x2C;                               //memory write
    trans[5].tx_buffer = linedata;                            //finally send the line data
    trans[5].length = SCREEN_WIDTH * 2 * 8 * FRAMEBUFFER_HEIGHT;       //Data length, in bits
    trans[5].flags = 0;                                       //undo SPI_TRANS_USE_TXDATA flag

    //Queue all transactions.
    for (x = 0; x < 6; x++)
    {
        ret = spi_device_queue_trans(spi, &trans[x], portMAX_DELAY);
        assert(ret == ESP_OK);
    }

    //When we are here, the SPI driver is busy (in the background) getting the transactions sent. That happens
    //mostly using DMA, so the CPU doesn't have much to do here. We're not going to wait for the transaction to
    //finish because we may as well spend the time calculating the next line. When that is done, we can call
    //send_line_finish, which will wait for the transfers to be done and check their status.
}

static void send_line_finish(spi_device_handle_t spi)
{
    spi_transaction_t *rtrans;
    esp_err_t ret;
    //Wait for all 6 transactions to be done and get back the results.
    for (int x = 0; x < 6; x++)
    {
        ret = spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
        assert(ret == ESP_OK);
        //We could inspect rtrans now if we received any info back. The LCD is treated as write-only, though.
    }
}

//Show menu until a Rom is selected
//Because the SPI driver handles transactions in the background, we can calculate the next line
//while the previous one is being sent.
static char filename[FILENAME_LENGTH];

static char* selectRomFromMenu(spi_device_handle_t spi)
{
    uint16_t *lines[2];
    //Allocate memory for the pixel buffers
    for (int i = 0; i < 2; i++)
    {
        lines[i] = heap_caps_malloc(SCREEN_WIDTH * FRAMEBUFFER_HEIGHT * sizeof(uint16_t), MALLOC_CAP_DMA);
        assert(lines[i] != NULL);
    }
    //Double-buffer indexes (sending vs active/drawing)
    int sendingBufferIdx = -1;
    int activeBufferIdx = 0;

    do {
        for (int y = 0; y < SCREEN_HEIGHT; y += FRAMEBUFFER_HEIGHT)
        {
            //Draw a framebuffer's worth of graphics
            drawRows(lines[activeBufferIdx], y, FRAMEBUFFER_HEIGHT);
            handleUserInput();
            //Finish up the sending process of the previous line, if any
            if (sendingBufferIdx != -1)
                send_line_finish(spi);
            //Swap sending_line and calc_line
            sendingBufferIdx = activeBufferIdx;
            activeBufferIdx = 1 - activeBufferIdx;
            //Send the line we currently calculated.
            send_lines(spi, y, lines[sendingBufferIdx]);
            //The line set is queued up for sending now; the actual sending happens in the
            //background. We can go on to calculate the next line set as long as we do not
            //touch line[sending_line]; the SPI sending process is still reading from that.
        }

        vTaskDelay(150 / portTICK_RATE_MS);
    
	} while ( getSelRom() == NO_ROM_SELECTED );
		
	filename[0] = '\0';
	strcat(filename, "/spiffs/");
	strcat(filename, menuEntries[getSelRom()].fileName);
	for (int i = 0; i < 2; i++)
	{
		heap_caps_free(lines[i]);
	}
	return filename;
}

ledc_channel_config_t ledc_channel;

void initBl()
{
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_3_BIT,
        .freq_hz = 5 * 1000 * 1000,
        .speed_mode = LEDC_LS_MODE,
        .timer_num = LEDC_LS_TIMER};

    ledc_timer_config(&ledc_timer);

    ledc_channel.channel = LEDC_LS_CH3_CHANNEL;
    //ledc_channel.duty       = 2000;
    ledc_channel.gpio_num = PIN_NUM_BCKL;
    ledc_channel.speed_mode = LEDC_LS_MODE;
    ledc_channel.timer_sel = LEDC_LS_TIMER;

    #if PIN_NUM_BCKL >= 0
    ledc_channel_config(&ledc_channel);
    #endif
}

void setBr(int bright)
{
    int duty;
	
    switch (bright)
	{
		case -2:
			duty = 0; break;
		case 0:
			duty = 1; break;
		case 1:
			duty = 2; break;
		case 2:
			duty = 3; break;
		case 3:
			duty = 4; break;
		case 4:
			duty = 5; break;
		default:
			duty = 1000;
	}
	
    #if PIN_NUM_BCKL >= 0
    ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, duty);
    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
    #endif
}

spi_device_handle_t _spi;

char* runMenu()
{
    printf("Init SPI bus\n");
    esp_err_t ret;
    
	spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 6 * SCREEN_WIDTH * 2 + 8,
        .flags = 0,
        .intr_flags = 0
		};
    
	spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,		
        .clock_speed_hz = LCD_SPI_CLOCK_RATE,
        .duty_cycle_pos = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .input_delay_ns = 0,	
        .mode = 0,                               //SPI mode 0
        .spics_io_num = PIN_NUM_CS,              //CS pin
        .queue_size = 7,                         //We want to be able to queue 7 transactions at a time
        .pre_cb = lcd_spi_pre_transfer_callback, //Specify pre-transfer callback to handle D/C line
        .flags = SPI_DEVICE_NO_DUMMY
		};
  
	//Initialize the SPI bus
    ret = spi_bus_initialize(VSPI_HOST, &buscfg, 1);
    printf("Init SPI bus completed\n");
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    printf("Adding screen device\n");
    ret = spi_bus_add_device(VSPI_HOST, &devcfg, &_spi);
    ESP_ERROR_CHECK(ret);
    printf("Adding screen completed\n");
    //Initialize the LCD
    lcd_init(_spi);
    printf("Start boot menu init\n");
    //Initialize the menu screen
    ret = menuInit();
    printf("Finished boot menu init\n");
    ESP_ERROR_CHECK(ret);
    printf("No errors, carry on\n");    

    initBl();
    setBr(2);
    //Show splash screen and prompt user to select something
    printf("Showing boot menu\n");
#ifdef BOOT_ROM
    setSelRom(BOOT_ROM);	//Auto start ROM from list
#else
    setSelRom(NO_ROM_SELECTED);	//Wait for selection
#endif
    char* filename = selectRomFromMenu(_spi);
    printf("Menu selection is %s\n", filename);
    return filename;
}
