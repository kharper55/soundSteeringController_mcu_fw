/*===================================================================================================
    File: app_i2s.c
    Author: Kevin Harper
    Date: 01/2024
    Details: Function definitions for the I2S (digital audio protocol) hardware peripheral. 

    Written using ESP-IDF v5.1.1 API. Built in 03/2025 using v5.1.2
//==================================================================================================*/

#include "app_include/app_i2s.h" // Function prototypes, constants, preprocessor defs/macros, typedefs

esp_err_t app_i2s_init(void) {
    // i2s output as gpio for now
    gpio_reset_pin(I2S_SDA_PIN);
    gpio_set_direction(I2S_SDA_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(I2S_SDA_PIN, 0);

    gpio_reset_pin(I2S_BCLK_PIN);
    gpio_set_direction(I2S_BCLK_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(I2S_BCLK_PIN, 0);

    gpio_reset_pin(I2S_WS_PIN);
    gpio_set_direction(I2S_WS_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(I2S_WS_PIN, 0);

    return ESP_OK;
}

/*========================================= END FILE ============================================*/