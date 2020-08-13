#ifndef PSXCONTROLLER_H
#define PSXCONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

int ReadControllerInput();
void ControllerInit();
bool getShowMenu();
int getBright();
int getVolume();
bool getShutdown();
bool isSelectPressed(int ctl);
bool isStartPressed(int ctl);
bool isUpPressed(int ctl);
bool isRightPressed(int ctl);
bool isDownPressed(int ctl);
bool isLeftPressed(int ctl);
bool isAPressed(int ctl);
bool isBPressed(int ctl);
bool isTurboAPressed(int ctl);
bool isTurboBPressed(int ctl);
bool isMenuPressed(int ctl);
bool isPowerPressed(int ctl);
bool isAnyPressed(int ctl);
bool isAnyFirePressed(int ctl);
int getTurboA();
int getTurboB();

#ifdef __cplusplus
}
#endif

#endif
