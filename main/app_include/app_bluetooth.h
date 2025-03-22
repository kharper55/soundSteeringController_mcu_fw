/*===================================================================================================
    File: app_bluetooth.h
    Author: Kevin Harper
    Date: 01/2024
    Details: Function prototypes, constants, preprocessor defs/macros for Bluetooth peripheral
             using Bluedroid stack. All BT and networking features should be implemented with the
             ___ core (?).

    Written using ESP-IDF v5.1.1 API. Built in 03/2025 using v5.1.2
//==================================================================================================*/

/*
ESP-IDF currently supports two host stacks. The Bluedroid based stack (default) supports classic 
Bluetooth as well as BLE. On the other hand, Apache NimBLE based stack is BLE only. For users to make a choice:
    - For usecases involving classic Bluetooth as well as BLE, Bluedroid should be used.
    - For BLE-only usecases, using NimBLE is recommended. It is less demanding in terms of 
    code footprint and runtime memory, making it suitable for such scenarios.
*/

#ifndef APP_BLUETOOTH_H
#define APP_BLUETOOTH_H

#ifdef __cplusplus
extern "C" {
#endif

// Includes
//#include "driver/bt.h" // IDF provided driver interface. (Include errors... investigate the makefile and cmake/json stuff)

// See https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/bluetooth/esp_hidd.html
// HID Host and Device link?

// Pin Defines

// Settings

// Macros

// Typedefs

// User functions


#ifdef __cplusplus
}
#endif

#endif  // APP_BLUETOOTH_H

/*========================================= END FILE ============================================*/