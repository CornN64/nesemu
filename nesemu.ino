/* Corn 2020
**
** Permission to use, copy, modify, and/or distribute this software for
** any purpose with or without fee is hereby granted, provided that the
** above copyright notice and this permission notice appear in all copies.
**
** THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
** WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR
** BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES
** OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
** WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,
** ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
** SOFTWARE.
*/
//#pragma GCC diagnostic warning "-fpermissive"
 
#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_partition.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "esp_spiffs.h"
//#include "esp_int_wdt.h"
//#include "esp_task_wdt.h"
#include "esp_bt.h"
#include "soc/rtc.h"
//#include "driver/spi_master.h"
//#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "driver/gpio.h"
#include "sdmmc_cmd.h"
#include "nvs_flash.h"
#include "src/Controller.h"
#include "src/nofrendo.h"
#include "src/menu.h"
#include "src/config.h"

#ifdef SKIP_MENU
extern char *selectedRomFilename = "/spiffs/unknown.nes";
#else
extern char *selectedRomFilename;
#endif

#define ASSERT_ESP_OK(returnCode, message)                        \
  if (returnCode != ESP_OK)                                       \
  {                                                               \
    printf("%s. (%s)\n", message, esp_err_to_name(returnCode));   \
    return returnCode;                                            \
  }

esp_err_t event_handler(void *ctx, system_event_t *event)
{
  return ESP_OK;
}

#ifdef CONFIG_SD_CARD
esp_err_t registerSdCard()
{
  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  // host.command_timeout_ms=200;
  // host.max_freq_khz = SDMMC_FREQ_PROBING;
  sdspi_slot_config_t slot_config = SDSPI_SLOT_CONFIG_DEFAULT();
  slot_config.gpio_miso =  (gpio_num_t)CONFIG_SD_MISO;
  slot_config.gpio_mosi =  (gpio_num_t)CONFIG_SD_MOSI;
  slot_config.gpio_sck =   (gpio_num_t)CONFIG_SD_SCK;
  slot_config.gpio_cs =    (gpio_num_t)CONFIG_SD_CS;
  slot_config.dma_channel = 2; //2
  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
    .format_if_mount_failed = false,
    .max_files = 2
  };

  sdmmc_card_t *card;
  esp_err_t ret = esp_vfs_fat_sdmmc_mount("/spiffs", &host, &slot_config, &mount_config, &card);	//!TODO: Evil hack... don't use spiffs here!
  ASSERT_ESP_OK(ret, "Failed to mount SD card");

  // Card has been initialized, print its properties
  sdmmc_card_print_info(stdout, card);

  return ret;
}

#else

esp_err_t registerSpiffs()
{
  esp_vfs_spiffs_conf_t conf = {
    .base_path = "/spiffs",
    .partition_label = NULL,
    .max_files = 5,
    .format_if_mount_failed = false
  };

  esp_err_t ret = esp_vfs_spiffs_register(&conf);

  ASSERT_ESP_OK(ret, "Failed to mount SPIFFS partition.");
  struct stat st;
  if (stat(ROM_LIST, &st) != 0)
  {
    printf("Cannot locate rom list in %s", ROM_LIST);
  }
  return ret;
}
#endif

void setup()
{
  rtc_clk_cpu_freq_set(RTC_CPU_FREQ_240M);  //make sure we run at full tilt ;)
  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT); //Drop Bluetooth and get some memory back

#ifdef CONFIG_SD_CARD
  vTaskDelay(300 / portTICK_RATE_MS); //a small delay to let SD card power up
  registerSdCard();
#else
  registerSpiffs();
#endif

  ControllerInit();
}

void esp_wake_deep_sleep()
{
  esp_restart();
}

void loop() {
#ifndef SKIP_MENU
  selectedRomFilename = runMenu();
#endif

  printf("NoFrendo start!\n");
  nofrendo_main(0, NULL);
  printf("Ops...NoFrendo died?!\n");
  //asm("break.n 1");
}
