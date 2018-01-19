#ifndef BUTTONS_H
#define BUTTONS_H

#define NUM_BUTTONS     3

typedef enum {
  no_button = 0,
  button1 = 1,
  button2 = 2,
  button3 = 4
} fg_button_type_t;

typedef enum {
  inc = 3,
  sd_test = 3,          // Define two button hold for system function - Shutdown Test
  dim = 4,
  sys_reset = 5,        // Define two button hold for system function - Reset
  sys_isolate = 5,      // Define two button hold for system function - Isolate
  dec = 6
} fg_button_fn_t;

typedef enum {
  shortPress = 0,
  longPress = 1
} fg_press_type_t;

typedef enum {
  button1short,
  button1long,
  button2short,
  button2long,
  button3short,
  button3long
} fg_button_press_type_t;

typedef struct {
  fg_button_type_t button;
  fg_press_type_t  press;
} fg_button_t;

#endif
