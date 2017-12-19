#ifndef BUTTONS_H
#define BUTTONS_H

typedef enum {
  button1 = 1,
  button2 = 2,
  button3 = 4
} fg_button_type_t;

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
