//==============================================================================
//
// FILE:        $Id: fg.h 6037 2017-11-23 03:55:11Z peter $
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
 * $Date: 2017-11-23 13:55:11 +1000 (Thurs, 23 Nov 2017) $
 * $Rev:  $
 */

#ifndef UI_H
#define UI_H

#include "fg.h"

#define UI_TIMEOUT_MS     (UI_TIMEOUT_MINS * 60UL * 1000UL)
#define UI_TIMEOUT_TICKS    (UI_TIMEOUT_MS / TICKER_MS)

#define UI_SET_HOLD_SECS    5UL
#define UI_SET_HOLD_MS      (UI_SET_HOLD_SECS * 1000UL)
#define UI_SET_HOLD_TICKS   (UI_SET_HOLD_MS / TICKER_MS)

#define UI_SET_TIMEOUT_SECS   30UL
#define UI_SET_TIMEOUT_MS   (UI_SET_TIMEOUT_SECS * 1000UL)
#define UI_SET_TIMEOUT_TICKS  (UI_SET_TIMEOUT_MS / TICKER_MS)

#define UI_DISP_UPDATE_MS   1000
#define UI_DISP_UPDATE_TICKS  (UI_DISP_UPDATE_MS / TICKER_MS)

#define UI_DISP_BATT_MS     3000
#define UI_DISP_BATT_TICKS    (UI_DISP_BATT_MS / TICKER_MS)

typedef enum {
  ui_state_disabled = 0,
  ui_state_lights_out,
  ui_state_disp_time,
  ui_state_disp_time_rel,
  ui_state_set_time_flash,
  ui_state_disp_conf1,
  ui_state_disp_conf1_rel,
  ui_state_set_conf1_flash,
  ui_state_disp_conf2,
  ui_state_disp_conf2_rel,
  ui_state_set_conf2_flash,
  ui_state_disp_bat1,
  ui_state_disp_bat1_rel,
  ui_state_disp_bat2,
  ui_state_disp_bat2_rel,
  ui_state_disp_diag1,
  ui_state_disp_diag1_rel,
  ui_state_disp_diag1_flash,
  ui_state_disp_diag2,
  ui_state_disp_diag2_rel,

  ui_state_disp_diag3,
  ui_state_disp_diag3_rel,
  ui_state_disp_diag4,
  ui_state_disp_diag4_rel,

  ui_state_set_cancel,
  ui_state_set_hold,
  ui_state_done_hold,

  ui_state_set_ones,
  ui_state_set_ones_rel,
  ui_state_set_tens,
  ui_state_set_tens_rel,
  ui_state_set_hund,
  ui_state_set_hund_rel,
  ui_state_set_thou,
  ui_state_set_thou_rel,

  ui_state_inc,
  ui_state_dec,

  ui_state_num_states
} ui_state_t;

void ui_init(void);

void ui_run_state_machine(fg_config_t  *config);

#endif
