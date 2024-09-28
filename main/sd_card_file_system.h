#ifndef LAB06_SD_CARD_FILE_SYSTEM_H
#define LAB06_SD_CARD_FILE_SYSTEM_H

#define MOUNT_POINT "/sdcard"

#include <stdio.h>

// Function to list the contents of a directory on the sd card specified by path
int32_t list_directory(const char* path);

// Populates directory_names with all the directories inside a path
void get_directory_names(const char* path, char** directory_names);

// Initialize SD card and mount filesystem
void init_sd_card();

#endif //LAB06_SD_CARD_FILE_SYSTEM_H
