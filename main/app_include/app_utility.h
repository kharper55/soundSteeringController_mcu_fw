#ifndef APP_UTILITY_H
#define APP_UTILITY_H

#ifdef __cplusplus
extern "C" {
#endif

// Includes 
#include <stdio.h>
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <string.h>

#include "rom/crc.h"            // For crc32 calculations

// Pin Defines

// Settings

// Macros

// Typedefs

// Static functions

// User functions
uint32_t app_compute_crc32(char * str, int data_len);

#ifdef __cplusplus
}
#endif

#endif  // APP_UTILITY_H