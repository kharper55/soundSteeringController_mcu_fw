#ifndef APP_I2S_H
#define APP_I2S_H

#ifdef __cplusplus
extern "C" {
#endif

// Includes
#include "driver/i2s_std.h"
#include "app_gpio.h" /* Include for now as I2S pins are simply initialized as GPIO*/

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