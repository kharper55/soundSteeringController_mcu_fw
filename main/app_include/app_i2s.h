/*===================================================================================================
    File: app_i2s.h
    Author: Kevin Harper
    Date: 01/2024
    Details: Function prototypes, constants, preprocessor defs/macros for the I2S (digital audio 
             protocol) hardware peripheral. 

    Written using ESP-IDF v5.1.1 API. Built in 03/2025 using v5.1.2
//==================================================================================================*/

#ifndef APP_I2S_H
#define APP_I2S_H

#ifdef __cplusplus
extern "C" {
#endif

// Includes
#include "driver/i2s_std.h" // Interface for I2S peripheral hardware driver
#include "app_gpio.h" /* Include for now as I2S pins are simply initialized as GPIO...*/

// Pin Defines
#define I2S_SDA_PIN             GPIO_NUM_25
#define I2S_BCLK_PIN            GPIO_NUM_26
#define I2S_WS_PIN              GPIO_NUM_27

// Settings

// Macros

// Typedefs

// Static functions

// User functions
esp_err_t app_i2s_init(void);

#ifdef __cplusplus
}
#endif

#endif  // APP_I2S_H

/*========================================= END FILE ============================================*/