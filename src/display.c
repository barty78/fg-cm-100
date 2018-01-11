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
      return displayCount - 1;   // Display registered

    }
//  taskEXIT_CRITICAL();
  return -1;       // Display not registered, already using max displays
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
 * Update display
 */
void updateDisplays(void)
{

}

uint8_t activeDisplayCount(void)
{
  return displayCount;
}

void powerLedOn(void)
{
  writeMessage(">,12,08,77,FF,3E\n");
}


