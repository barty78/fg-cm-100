/*
 * display.c
 *
 *  Created on: 15 Nov.,2017
 *      Author: Peter Bartlett <peter@masters-young.com.au>
 */

///////////////////////////////////////////////////////////////////////////////
//
// display.c
//
//  DESCRIPTION: Display Panel Routines
//
//  CREATED:     15 NOV 2017
//  AUTHOR:      Peter Bartlett <peter@masters-young.com.au>
//
///////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <math.h>

#include "global.h"
#include "display.h"
#include "comms.h"
#include "packets.h"

extern dispdata display;

/**
 *  Register Display
 * @param uid - 96bit UID from STM32 in display - 27 chars long inc. \0
 * @param type - 0 = indicator type, 1 = shutdown type.  Determined by connector pin on front label
 * @return index of display, -1 if not added
 */
int registerDisplay(char *uid, uint8_t type)
{
//  taskENTER_CRITICAL();
  if (displayCount < MAX_DISPLAYS)
    {
      for (int i = 0; i < displayCount; i++)
        {
          if (strncmp(uid, displays[i].display, UID_LENGTH) == 0) return i;
        }

      memcpy(&displays[displayCount].display, uid, UID_LENGTH);
      displays[displayCount].type = type;
      displays[displayCount].lastSeen = HAL_GetTick();
      displayCount++;
      display.ledStateBuffer = 1 << LED_POWER_ON;
      return displayCount - 1;   // Display registered

    }
//  taskEXIT_CRITICAL();
  return -1;       // Display not registered, already using max displays
}

/**
 *
 */
void initDisplay()
{
  display.ledStateBuffer = 0;
  display.group_pwm = 0xFFFFFFFF;
  display.group_iref = 0x77777777;
  display.dim = 0;
}

void dimDisplay(void)
{
  uint32_t tmp = 0;
  if (!display.dim)
    {
//      volatile char b0 = ((display.group_pwm & 0xFF) >> 0 / 2);
//      volatile char b1 = ((display.group_pwm & 0xFF00) >> 8 / 2);
//      volatile char b2 = ((display.group_pwm & 0xFF0000) >> 16 / 2);
//      volatile char b3 = ((display.group_pwm & 0xFF000000) >> 24 / 2);

      display.group_pwm = 0x1F1F1F1F;
//      for (int i = 0; i < 4; i++)
//        {
//          uint32_t val = (display.group_pwm & (1 << i));
//        }

      display.dim = 1;
    } else {
        display.group_pwm = 0xFFFFFFFF;
        display.dim = 0;
    }

}

/**
 *
 * @param id
 */
uint8_t renewDisplay(uint8_t id)
{
  if (id > MAX_DISPLAYS || id > displayCount) return 1;
  displays[id].lastSeen = HAL_GetTick();
}

/**
 * Check for lost display
 * @return
 */
uint8_t checkForLostDisplay(void)
{

  for (int i = 0; i < displayCount; i++)
    {
      if (HAL_GetTick() > (displays[i].lastSeen + DISPLAY_TIMEOUT))
        {
          if (displayCount > 0)
            {
              displayCount--;
            }
          return 1;
        }
    }

  return 0;
}

/**
 * Pack display data and send
 */
void updateDisplays(dispdata data)
{
  char ledMsg[RESPONSE_BUFFER_LENGTH];

  sprintf(ledMsg, ">,17,%06X,%08X,%08X,",data.ledStateBuffer, data.group_pwm, data.group_iref);
  sendResponse(ledMsg);
}

/**
 * @brief This function takes a counter integer value and updates the display with the counter value
 * @param counter - Integer counter to display on the 7-segment displays
 */
void updateDigits(uint8_t counter)
{
  uint8_t tens, ones;
  char digits[3];
  uint16_t tmp;

  itoa10(counter, digits, 3);                                       //Convert integer counter to ascii digits
  tens = ds1_DigitLookup[(char)digitsToInt(digits, 0, 1, 10)];      //Lookup LED segment states for character to display - tens digit
  ones = ds2_DigitLookup[(char)digitsToInt(digits, 1, 1, 10)];      //Lookup LED segment states for character to display - ones digit

  tmp = display.ledStateBuffer & 0x01FF;                            //Copy non-digit led state bits
  display.ledStateBuffer = 0;                                       //Clear state variable
  display.ledStateBuffer |= tmp;                                    //Restore non-digit led state bits
  display.ledStateBuffer |= (tens << TENS_DIGIT_OFFSET);            //Mask in tens digit led state bits
  display.ledStateBuffer |= (ones << ONES_DIGIT_OFFSET);            //Mask in ones digit led state bits
//  updateDisplays(display);                                          // Send the msg to the display
}

void refresh(void)
{
  updateDisplays(display);
}

uint8_t activeDisplayCount(void)
{
  return displayCount;
}

void ledToggle(uint8_t led)
{
  display.ledStateBuffer ^= (1 << led);
}

void ledOff(uint8_t led)
{
  display.ledStateBuffer &= ~(1 << led);
}

void ledOn(uint8_t led)
{
  display.ledStateBuffer |= (1 << led);
}

//void buzzerToggle(void)
//{
//  display.ledStateBuffer ^= (1 << BUZZER);
//}
//
//void buzzerOn(void)
//{
//  display.ledStateBuffer |= (1 << BUZZER);
//}
//
//void buzzerOff(void)
//{
//  display.ledStateBuffer &= ~(1 << BUZZER);
//}

//void powerLedOn(void)
//{
//  writeMessage(">,12,08,77,FF,3E\n");
//}


