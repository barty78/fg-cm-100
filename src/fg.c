//==============================================================================
//
// FILE:        $Id: fg.c 6490 2017-11-23 03:07:40Z peter $
//
// CREATED BY:  Peter Bartlett
//              Masters and Young Pty Ltd
//              Unit 2, 5 Booran Drive
//              Woodridge  QLD  4114
//              25/06/2015
//
// All copyright, confidential information, patents, design rights and all other
// intellectual property rights of whatsoever nature contained herein are and
// shall remain the sole and exclusive property of Masters and Young Pty. Ltd.
//
//==============================================================================


/**
 * @file
 *
 * @brief
 *
 * $Date: 2017-11-23 13:07:40 +1000 (Thurs, 23 Nov 2017) $
 * $Rev:  $
 */


#include <stdlib.h>
#include <math.h>

#include "fg.h"
#include "display.h"
#include "threads.h"
#include "buttons.h"
#include "comms.h"

//
// Local Globals
//

fg_config_t l_config;

fg_config_t *l_fg_config;
fg_alarm_state_t *l_fg_alarm_state;

static fg_state_t  l_curr_state    = fg_state_ready;    //!< The current state of the HAS state machine.

static uint8_t l_health_checked    = 0;
static uint8_t l_supply_ok_tick_cnt  = 0;
//static ubcd4_t l_supply_voltage    = {.uint16 = 0x0000};

static uint8_t l_diag_start      = 0;
static uint8_t l_diag_stop       = 0;

static uint32_t buttonPressedTick;
uint8_t pushButtonsThread;
uint8_t prevButton = 0;

uint8_t buttons;
//uint8_t heldButtonCount;

//
// Flags and counters used for low/critical voltage detection.
// Note:
//  - They are not initialised here, they are initialised in has_init().
//  - They need to be done in has_init() to restore the system to a known state after recovering from low system voltage.
//  - It is NOT done here as well as then it will be done in 2 places, and at some point someone will only update 1 of them ... (me and Murphy are well acquainted ...)
//
static int16_t  l_batt_low_counter;
static int16_t  l_batt_crit_counter;
static uint8_t  l_batt_low_flag;
static uint8_t  l_batt_crit_flag;

//
// Strings for state machine states.
//

//
// Local Prototypes
//

static fg_state_t fg_handle_disabled(fg_state_t prev_state);
static fg_state_t fg_handle_ready(fg_state_t prev_state);
static fg_state_t fg_handle_armed(fg_state_t prev_state);
static fg_state_t fg_handle_start(fg_state_t prev_state);
static fg_state_t fg_handle_alarm(fg_config_t *config, fg_state_t prev_state, uint32_t *sd_countdn);
static fg_state_t fg_handle_fault(fg_state_t prev_state);
static fg_state_t fg_handle_isolated(fg_state_t prev_state);
static fg_state_t fg_handle_shutdown_initiate(fg_state_t prev_state, uint32_t *sd_countdn);
static fg_state_t fg_handle_shutdown_pending(fg_state_t prev_state, uint32_t *sd_countdn);
static fg_state_t fg_handle_shutdown_active(fg_state_t prev_state);


void fg_init(fg_config_t **config )
{
  //need to (re)initialise all local variables here.
    l_curr_state  = fg_state_ready;
    l_diag_start  = 0;
    l_diag_stop   = 0;

    l_config.factory.system_type = fg_shutdown;
    l_config.factory.shutdown_timeout = 30;

    *config = &l_config;

    l_fg_config = (fg_config_t *) BKPSRAM_BASE;
    l_fg_alarm_state = (fg_alarm_state_t *) BKPSRAM_BASE + sizeof(fg_config_t);


    volatile fg_config_t ret = *(fg_config_t*) l_fg_config;

    l_fg_config->factory.shutdown_timeout = 50;
    l_fg_config->factory.system_type = fg_state_shutdown_active;

//    fg_config_t *BKPSRAM_CONFIG = (fg_config_t *)BKPSRAM_BASE;
//    BKPSRAM_CONFIG->factory.shutdown_timeout = 30;
//    BKPSRAM_CONFIG->factory.system_type = fg_shutdown;

    l_fg_alarm_state->state = fg_state_alarm;

//    fg_alarm_state_t *BKPSRAM_ALARM_STATE = (fg_alarm_state_t *)BKPSRAM_BASE + sizeof(fg_config_t);
//    BKPSRAM_ALARM_STATE->state = isolated;

    //TODO - Just to test mem offset is correct.
//    ERROR_STATE = BLINK_TIMEOUT_ERROR;

//    volatile uint8_t tmp = sizeof(fg_config_t);

}

/**
 * @brief
 *
 */
fg_state_t fg_get_state(void)
{
  return (l_curr_state);
}

void buttonPressedHandler(osEvent evt, fg_state_t state)
{
  char msg[32];
  uint8_t pushedButton = prevButton;

  if (evt.status == osEventMessage)
    {
      taskENTER_CRITICAL();
      pushedButton = evt.value.v;

      taskEXIT_CRITICAL();
#ifdef DEBUG
//      sprintf(msg, "Recv - Button %02X\n", pushedButton);
      writeMessage(msg);
#endif
    }

  if (pushedButton != prevButton)                               // Button changed
    {
      if (pushedButton != no_button) {                          // Button changed and didn't go back to default - no buttons
          buttonPressedTick = HAL_GetTick();
          if (pushedButton == dec)
            {
#ifdef DEBUG
//              writeMessage("Dec...\n");
#endif
              //TODO - Implement decrement fn handler
            }
          if (pushedButton == inc)
            {
#ifdef DEBUG
//              writeMessage("Inc...\n");
#endif
              //TODO - Implement increment fn handler
            }
      } else {
          buttonPressedTick = 0;
      }
      prevButton = pushedButton;
    } else {
        if (buttonPressedTick != 0)
          {
            if (pushedButton == sys_reset && (buttonPressedTick + SYSTEM_RESET_BUTTON_HOLD_DELAY < HAL_GetTick()))
              {
                //TODO - Go to appropriate state - Isolate/Reset
#ifdef DEBUG
//                writeMessage("Isolate\n");
#endif
                buttonPressedTick = 0;
              }
          }
    }
}

/**
 * @brief
 *
 */
void fg_run_state_machine(fg_config_t *config)
{
  static uint32_t      sd_countdn       = 0xFFFFFFFF;
  static fg_state_t    prev_state       = fg_state_num_states;
  fg_state_t           next_state       = fg_state_num_states; // Doesn't need to be initialised. Done so to keep compiler happy.

  // Check for button presses
  osEvent evt = osMessageGet(buttonQID, 10);
  buttonPressedHandler(evt, l_curr_state);

  //
  // Handle each of the states accordingly.
  //
  switch (l_curr_state)
  {
    case fg_state_disabled:
      next_state = fg_handle_disabled(prev_state);
      break;
    case fg_state_ready:
      next_state = fg_handle_ready(prev_state);
      break;
    case fg_state_armed:
      next_state = fg_handle_armed(prev_state);
      break;
    case fg_state_start:
      next_state = fg_handle_start(prev_state);
      break;
    case fg_state_alarm:
      next_state = fg_handle_alarm(config, prev_state, &sd_countdn);
      break;
    case fg_state_fault:
      next_state = fg_handle_fault(prev_state);
      break;
    case fg_state_isolated:
      next_state = fg_handle_isolated(prev_state);
      break;
    case fg_state_shutdown_pending:
      next_state = fg_handle_shutdown_pending(prev_state, &sd_countdn);
      break;
    case fg_state_shutdown_active:
      next_state = fg_handle_shutdown_active(prev_state);
      break;

    default:
      break;
  }

  prev_state = l_curr_state;
  l_curr_state = next_state;

}


fg_state_t fg_handle_disabled(fg_state_t prev_state)
{
  if (fg_handle_disabled != prev_state) {
        //Single execution code goes here, first entry into this state
    }

  return (fg_state_disabled);
}

fg_state_t fg_handle_ready(fg_state_t prev_state)
{
  if (fg_handle_ready != prev_state) {
        //Single execution code goes here, first entry into this state
    }

  return (fg_state_ready);
}

fg_state_t fg_handle_armed(fg_state_t prev_state)
{
  if (fg_handle_armed != prev_state) {
        //Single execution code goes here, first entry into this state
    }

  return (fg_state_armed);
}

fg_state_t fg_handle_start(fg_state_t prev_state)
{
  return (fg_state_start);
}

fg_state_t fg_handle_alarm(fg_config_t *config, fg_state_t prev_state, uint32_t *sd_countdn)
{
  if (config->factory.system_type == fg_shutdown)
    {
      *sd_countdn = config->factory.shutdown_timeout;
      return (fg_state_shutdown_pending);
    }

  return (fg_state_alarm);
}

fg_state_t fg_handle_fault(fg_state_t prev_state)
{
  if (fg_handle_fault != prev_state) {
        //Single execution code goes here, first entry into this state
    }

  return (fg_state_fault);
}

fg_state_t fg_handle_isolated(fg_state_t prev_state)
{
  if (fg_handle_isolated != prev_state) {
    //Single execution code goes here, first entry into this state
}

  return (fg_state_isolated);
}

fg_state_t fg_handle_shutdown_initiate(fg_state_t prev_state, uint32_t *sd_countdn)
{

}

fg_state_t fg_handle_shutdown_pending(fg_state_t prev_state, uint32_t *sd_countdn)
{
  if (fg_handle_shutdown_pending != prev_state) {
      //Single execution code goes here, first entry into this state
  }

  *sd_countdn--;
  /*osEvent evt = osMessagePut(displayDigitsQID, (uint8_t)sd_countdn, 0);
  if (evt.status == osEventMessage)
    {

    }
*///  set7Seg(*sd_countdn); // Update the display with the count
  if (0 == *sd_countdn)
    {
      return (fg_state_shutdown_active);
    }
  return (fg_state_shutdown_pending);
}

fg_state_t fg_handle_shutdown_active(fg_state_t prev_state)
{
  if (fg_handle_shutdown_active != prev_state)
    {

    }

  return (fg_state_shutdown_active);
}
