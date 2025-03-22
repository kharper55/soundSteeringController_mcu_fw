/*===================================================================================================
    File: app_utility.h
    Author: Kevin Harper
    Date: 01/2024
    Details: Various function prototypes, constants, preprocessor defs/macros which are used
             generally and not for any one processor peripheral in particular.

    Written using ESP-IDF v5.1.1 API. Built in 03/2025 using v5.1.2
//==================================================================================================*/

#ifndef APP_UTILITY_H
#define APP_UTILITY_H

#ifdef __cplusplus
extern "C" {
#endif

// Includes 
#include <stdio.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_mac.h" // TOOL CHANGE INCLUSION 03/10/2025
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <string.h>

#include <ctype.h>

#include "rom/crc.h"            // For crc32 calculations

// Pin Defines

// Settings

// Macros

// Typedefs

// Static functions

// User functions
uint32_t app_compute_crc32_str(char * str, int data_len);
uint32_t app_compute_crc32_bytes(uint8_t * bytes, int data_len);
//uint8_t hex2dec(uint8_t data);
uint8_t hex2dec(char data);
//int hex2dec(char hex[]);
uint8_t concat_hex_chars(char high, char low);

#ifdef __cplusplus
}
#endif

#endif  // APP_UTILITY_H