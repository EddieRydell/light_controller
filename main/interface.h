#ifndef LAB06_INTERFACE_H
#define LAB06_INTERFACE_H

#include "my_lcd/my_lcd.h"

extern TFT_t display_config;
extern uint32_t directory_index;

void init_interface();

void init_filenames();

void draw_interface();

const char* interface_get_selected_filename();

#endif //LAB06_INTERFACE_H
