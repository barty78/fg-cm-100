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

#ifndef FG_H
#define FG_H

#include "stm32f2xx_hal.h"
#include "cmsis_os.h"

typedef enum {
  fg_state_disabled = 0,
  fg_state_ready,
  fg_state_start,
  fg_state_armed,
  fg_state_isolated,
  fg_state_armed_powerloss,
  fg_state_isolated_powerloss,
  fg_state_alarm,
  fg_state_fault,
  fg_state_shutdown_pending,
  fg_state_shutdown_active,
  fg_state_num_states
} fg_state_t;

typedef enum {
  fg_truck_type_12V = 0,
  fg_truck_type_24V,
  fg_truck_type_num_types
} fg_truck_type_t;

typedef enum {
  fg_indicate = 0,
  fg_shutdown
} fg_system_type_t;

typedef struct {
  uint8_t             num_batts;
//  has_batt_config_t   bat_config[2];
  fg_system_type_t    system_type;
  uint32_t            shutdown_timeout;
  //uint8_t       config_enable;
} fg_factory_config_t;

typedef struct {
  uint32_t            tmp;
} fg_user_config_t;

typedef struct {
  fg_factory_config_t   factory;
  fg_user_config_t      user;
} fg_config_t;

void fg_init(fg_config_t **config);

fg_state_t fg_get_state(void);

void fg_run_state_machine(fg_config_t  *config);


#endif
