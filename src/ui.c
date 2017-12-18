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

#include "ui.h"

//
// Local Globals
//




//
// Flags and counters used for low/critical voltage detection.
// Note:
//  - They are not initialised here, they are initialised in has_init().
//  - They need to be done in has_init() to restore the system to a known state after recovering from low system voltage.
//  - It is NOT done here as well as then it will be done in 2 places, and at some point someone will only update 1 of them ... (me and Murphy are well acquainted ...)
//

//
// Strings for state machine states.
//

static ui_state_t l_curr_state = ui_state_disabled;  //!< The current state of the UI state machine.
static uint32_t   l_timeout_ticks;            //!< UI timeout in ticker ticks.


//
// Local Prototypes
//


void ui_init()
{
  l_curr_state = ui_state_disabled;
}

/**
 * @brief
 *
 */
void ui_run_state_machine(fg_config_t *config)
{

  static ui_state_t    prev_state       = ui_state_num_states;
  ui_state_t           next_state       = ui_state_num_states; // Doesn't need to be initialised. Done so to keep compiler happy.

  if (config->factory.shutdown_timeout == 0){

  }
  //
  // Handle each of the states accordingly.
  //
  switch (l_curr_state)
  {
    case ui_state_disabled:
//      next_state = fg_handle_disabled(prev_state);
      break;

    default:
      break;
  }

  prev_state = l_curr_state;
  l_curr_state = next_state;

}
