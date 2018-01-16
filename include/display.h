#ifndef DISPLAY_H
#define DISPLAY_H

#include "stm32f2xx_hal.h"
#include "cmsis_os.h"

#define     ALLPORTS          0xFF
#define     DEFAULT_PWM       1.0
#define     DEFAULT_CURRENT   0.1
#define		  n_of_ports		    24
#define		  PWM_ON_VALUE	    255
#define     UID_LENGTH        26
#define     NUM_LEDS          14
#define     NUM_ALL_LEDS      24
#define     MAX_DISPLAYS      4              // Maximum number of allowed displays
#define     DISPLAY_TIMEOUT   (10000)    // 10 second display timeout
#define     TENS_DIGIT_OFFSET 9         //Offset to first LED segment in tens digit
#define     ONES_DIGIT_OFFSET 16        //Offset to first LED segment in ones digit
#define     BUZZER            24

struct display {
  char display[UID_LENGTH];
  uint8_t type;
  uint32_t lastSeen;

} displays[MAX_DISPLAYS];

static uint8_t displayCount = 0;

static const uint8_t ds1_DigitLookup[10] =
{
0x7B, 0x48, 0x3D, 0x6D, 0x4E, 0x67, 0x77, 0x49, 0x7F, 0x4F
};

static const uint8_t ds2_DigitLookup[10] =
{
0x77, 0x14, 0x3B, 0x3E, 0x5C, 0x6E, 0x6F, 0x34, 0x7F, 0x7C
};

typedef struct {
  uint32_t ledStateBuffer;  //24 bit led state buffer (on/off)
  uint32_t group_pwm;       //4 x 8 bit PWM Value for LED groups (Red, Amber, Green, 7-Seg)
  uint32_t group_iref;      //4 x 8 bit PWM Value for LED groups (Red, Amber, Green, 7-Seg)
  uint8_t dim;
} dispdata;

//typedef struct {
//  uint32_t buffer;
//  uint8_t pwm_red;
//  uint8_t pwm_amber;
//  uint8_t pwm_green;
//  uint8_t pwm_seg;
//  uint8_t iref_red;
//  uint8_t iref_amber;
//  uint8_t iref_green;
//  uint8_t iref_seg;
//} dispdata;

dispdata display;

uint32_t displayBuffer;

enum registerType {
  REGISTER,
  UPDATE
};

enum displayType {
  TYPE_INDICATOR,
  TYPE_SHUTDOWN
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
*/
enum leds_shutdown {
  LED_ALARM,
  LED_SHUTDOWN,
  LED_DISCHARGE,
  LED_LOW_PRESSURE,
  LED_SYS_FAULT,
  LED_ISOLATED,
  LED_NORMAL,
  LED_PWR_FAULT,
  LED_POWER_ON
};
/*
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
void updateDisplays(dispdata data);
void powerLedOn(void);
void set7Seg(uint8_t value);
uint8_t activeDisplayCount(void);
void updateDigits(uint8_t counter);

#endif
