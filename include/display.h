#ifndef DISPLAY_H
#define DISPLAY_H

#include "stm32f2xx_hal.h"
#include "cmsis_os.h"

#define     ALLPORTS        0xFF
#define     DEFAULT_PWM     1.0
#define     DEFAULT_CURRENT 0.1
#define		n_of_ports		24
#define		PWM_ON_VALUE	255
#define     NUM_LEDS 14

static const uint8_t ds1_DigitLookup[10] =
{
0x7B, 0x48, 0x3D, 0x6D, 0x4E, 0x67, 0x77, 0x49, 0x7F, 0x4F
};

static const uint8_t ds2_DigitLookup[10] =
{
0x77, 0x14, 0x3B, 0x3E, 0x5C, 0x6E, 0x6F, 0x34, 0x7F, 0x7C
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

#endif
