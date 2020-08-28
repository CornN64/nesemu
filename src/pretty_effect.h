#pragma once
#include <stdint.h>
#include "esp_err.h"


/**
 * @brief Calculate and disaply a set of rows to the screen
 *
 * @param dest Destination for the pixels. Assumed to be LINECT * 320 16-bit pixel values.
 * @param y Starting y coordinate of the chunk of lines.
 * @param rowCount Amount of lines to calculate
 */
void drawRows(uint16_t *dest, int y, int rowCount);

void handleUserInput();

/**
 * @brief Initialize the boot screen and menu
 *
 * @return ESP_OK on success, an error from the jpeg decoder otherwise.
 */
esp_err_t menuInit();

bool peGetPixel(char peChar, int pe1, int pe2);

void setLineMax(int lineM);

int getSelRom();

void setSelRom(int selR);

bool getUpdateMode();

bool getXStretch();

void setXStretch(bool str);

void setUpdateMode(bool str);

void setBright(int bright);