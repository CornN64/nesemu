#include <string.h>
#include "freertos/FreeRTOS.h"
#include "menu.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_partition.h"
#include "nvs_flash.h"
#include "charData.h"
#include "iconData.h"
#include "pretty_effect.h"

int yLineEnded;
int charOff;
int change;

bool cpGetPixel(char cpChar, int cp1, int cp2)
{
	return getPixel(cpChar, cp1, cp2);
}

//Load Rom list from flash partition to char array(lines), init some variables for printing rom list
void initRomList()
{
	FILE *romsFile = fopen(ROM_LIST, "r");
	char line[64];
	entryCount = -1;
	while (fgets(line, 64, romsFile) != NULL)
	{
		if (line[0] == '*' && line[1] == '#' && line[2] == '*')
		{
			break;
		}
		else
		{
			entryCount++;
		}
	}
	menuEntries = (MenuEntry *)malloc(entryCount * sizeof(MenuEntry));
	rewind(romsFile);
	fgets(line, 64, romsFile); // Skip first line
	for (int i = 0; i < entryCount; i++)
	{
		menuEntries[i].entryNumber = -1;
		menuEntries[i].icon = 'E';
		strcpy(menuEntries[i].name,     "?                 ");
		strcpy(menuEntries[i].fileName, "?                 ");
		fscanf(romsFile, "%d.\t%c\t%[^\t]\t%[^\n\r]\n",
			   &menuEntries[i].entryNumber,
			   &menuEntries[i].icon,
			   menuEntries[i].name,
			   menuEntries[i].fileName);
		for (int j = FILENAME_LENGTH; j > 0; j--) 
		{
			if (menuEntries[i].fileName[j] < ' ') {
				menuEntries[i].fileName[j] = '\0';
			}
		}
		printf("Read entry number %d; icon %c; name %s; file %s\n",
			   menuEntries[i].entryNumber,
			   menuEntries[i].icon,
			   menuEntries[i].name,
			   menuEntries[i].fileName);
	}
	fclose(romsFile);
	printf("Read %d rom entries\n", entryCount);
}

//get depending on x/y value the actual char from lines array
//depending on x/y/actChar return color/black if char value is true/false(charData.c)
//depending on x/y/actChar read and return icon pixel color(iconData.c)
int getCharPixel(int x, int y, int change, int choosen)
{
	int page = choosen / 13;
	int line = ((y - 3) / 18) + 13 * page;
	int charNo = (x - 26) / 16;
	char actChar;

	if (line >= entryCount) return 0x0000;
	
	if (x >= 7 && x <= 22)
	{
		if (line == choosen)
			return getIconPixel(menuEntries[line].icon, x - 7, (y - 5) % 18, change);
		else
			return 0x0000;
	}
	
	actChar = menuEntries[line].name[charNo];
	
	if (actChar < ' ')
	{
		return 0x0000;
	}
	
	if (getPixel(actChar, (x - 26) % 16, (y - 3) % 18) == 1)
		return 0x001F;
	return 0x0000;
}

void freeRomList()
{
	free(menuEntries);
}
