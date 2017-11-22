#ifndef DISPLAY_H
#define DISPLAY_H

#include "stm32f2xx_hal.h"
#include "cmsis_os.h"

#define     ALLPORTS        0xFF
#define     DEFAULT_PWM     1.0
#define     DEFAULT_CURRENT 0.1
#define		  n_of_ports		24
#define		  PWM_ON_VALUE	255
#define     UID_LENGTH    26
#define     NUM_LEDS 14
#define     MAX_DISPLAYS 4              // Maximum number of allowed displays
#define     DISPLAY_TIMEOUT (10000)    // 10 second display timeout

struct display {
  char display[UID_LENGTH];
  uint8_t type;
  uint32_t lastSeen;
} displays[MAX_DISPLAYS];

uint8_t displayCount = 0;

static const uint8_t ds1_DigitLookup[10] =
{
0x7B, 0x48, 0x3D, 0x6D, 0x4E, 0x67, 0x77, 0x49, 0x7F, 0x4F
};

static const uint8_t ds2_DigitLookup[10] =
{
0x77, 0x14, 0x3B, 0x3E, 0x5C, 0x6E, 0x6F, 0x34, 0x7F, 0x7C
};

uint32_t displayBuffer;

enum registerType {
  REGISTER,
  UPDATE
};

enum displayType {
  INDICATOR,
  SHUTDOWN
};

/*
enum leds_indicator {
  ALARM,
  DETECTOR,
  DISCHARGE,
  LOW_PRESSURE,
  SYS_FAULT,
  ISOLATED,
  NORMAL,
  PWR_FAULT,
  POWER_ON
};

enum leds_shutdown {
  ALARM,
  SHUTDOWN,
  DISCHARGE,
  LOW_PRESSURE,
  SYS_FAULT,
  ISOLATED,
  NORMAL,
  PWR_FAULT,
  POWER_ON
};

enum btn_indicator {
  ISOL_RESET,
  TEST,
  DIM
};

enum btn_shutdown {
  SD_EXTEND,
  TEST,
  DIM
};
*/

int registerDisplay(char *uid, uint8_t type);
uint8_t renewDisplay(uint8_t id);
uint8_t checkForLostDisplay(void);
void updateDisplays(void);
void powerLedOn(void);




#endif
