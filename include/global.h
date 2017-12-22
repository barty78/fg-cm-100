#ifndef GLOBAL_H
#define GLOBAL_H

#include "stm32f2xx_hal.h"
#include "cmsis_os.h"

// Version Definitions
#define HARDWARE_ID "CM-100"
#define FIRMWARE_VERSION "001"
#define SVN_RELEASE "571"  // $Revision:$

#define RS485

#define ID_STRING_LENGTH	24
static char	id_string[ID_STRING_LENGTH+1];

// Debug Definitions
#define CHECK_STACK 1                               // Global Thread Stack debugging flag, (Set to 1 to enable stack monitoring and overflow exceptions)
#define CHECK_THREADS 1                             // Monitor Thread loop durations

#define BUTTON_HOLD_DELAY                 (2000)    // 2 seconds Button Hold Delay
#define SYSTEM_ISOLATE_BUTTON_HOLD_DELAY  (7000)    // 7 second hold delay for system isolation
#define SYSTEM_RESET_BUTTON_HOLD_DELAY    (7000)    // 7 second hold delay for system isolation

// Battery-Backed SRAM variables
#define ERROR_STATE                       (*(__IO uint32_t *) (BKPSRAM_BASE + sizeof(fg_config_t)))

// Error Definitions
#define SYSTEM_ERROR               0x0001
#define THREAD_ERROR               0x0002
#define STACK_OVERFLOW_ERROR       0x0003
#define USER_RESET_ERROR           0x0004
#define WRITEMESSAGE_TIMEOUT_ERROR 0x0005
#define READPACKET_TIMEOUT_ERROR   0x0006
#define PARSEPACKET_TIMEOUT_ERROR  0x0007
#define READIO_TIMEOUT_ERROR       0x0008
#define WRITEIO_TIMEOUT_ERROR      0x0009
#define UNDERVOLTAGE_ERROR         0x000A
#define SURGEPROTECTION_ERROR      0x000B
#define PWR_5_12_24_VOLTAGE_ERROR  0x000C
#define MONITOR_TIMEOUT_ERROR      0x000D
#define I2CREAD_TIMEOUT_ERROR      0x000E
#define I2CWRITE_TIMEOUT_ERROR     0x000F
#define DISPLAY_TIMEOUT_ERROR	     0x0010
#define BLINK_TIMEOUT_ERROR        0x0011

// Alarm Definitions
#define NORMAL                     0x0001
#define ISOLATED                   0x0002
#define FAULT                      0x0003
#define FAULT_DET                  0x0004
#define FAULT_LP                   0x0005
#define FAULT_CYLDIS               0x0006
#define FAULT_SOL                  0x0007
#define FAULT_DISP                 0x0008
#define ALARM                      0x0009
#define SHDN                       0x000A


#define THREAD_WATCHDOG_DELAY (60000)  // 60 seconds Shutdown Delay, (system will be shutdown when any thread watchdog flag is not reset within this period)

#define STM32_UUID ((uint32_t *)0x1FFF7A10)  // STM32F205 Unique Identifier Address

uint8_t flagFirmwareReset;

typedef enum {
  fg_truck_type_12V = 0,
  fg_truck_type_24V,
  fg_truck_type_num_types
} fg_truck_type_t;

typedef enum {
  fg_indicate = 0,
  fg_shutdown
} fg_system_type_t;

typedef enum {
  normal,
  isolated,
  fault,
  alarm
} fg_alarm_state_type_t;

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

typedef struct {
  fg_alarm_state_type_t      state;
} fg_alarm_state_t;

extern fg_config_t *BKPSRAM;
// Global debugging flag. Only set when testing code. Do NOT set in production code!
#ifdef STMDEBUG
#define DEBUG 1
#define NUCLEO 1          // Global Nucleo Development Board flag. Indicates that code is being executed on Nucleo Dev Board. Only set when testing code. Do NOT set in production code!
#else
#define DEBUG 0
#define NUCLEO 0         // Global Nucleo Development Board flag. Indicates that code is being executed on Nucleo Dev Board. Only set when testing code. Do NOT set in production code!
#endif

#if CHECK_STACK == 1
// #define configCHECK_FOR_STACK_OVERFLOW 1
// #define INCLUDE_uxTaskGetStackHighWaterMark 1

UBaseType_t blinkThreadStackHighWaterMark;

UBaseType_t uiThreadStackHighWaterMark, writeMessageThreadStackHighWaterMark, readPacketThreadStackHighWaterMark,
			parsePacketThreadStackHighWaterMark, readIOThreadStackHighWaterMark, writeIOThreadStackHighWaterMark,
			monitorThreadStackHighWaterMark;

 void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed portCHAR *pcTaskName);
#endif

void delay(uint32_t period);
uint8_t delayedResponse(uint8_t condition, uint8_t* flag, uint8_t* flagMsg, uint32_t* start, uint32_t* end, uint32_t* send, uint32_t delay, uint32_t delayMsg, char* message);
void initSystem();
void firmwareReset(uint16_t error);
/*
 Exported macro ------------------------------------------------------------
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
 Exported functions ------------------------------------------------------- */

#endif
