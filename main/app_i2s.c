/*===================================================================================================
    File: app_i2s.c
    Author: Kevin Harper
    Date: 01/2024
    Details: Function definitions for the I2S (digital audio protocol) hardware peripheral. 

    Written using ESP-IDF v5.1.1 API. Built in 03/2025 using v5.1.2
//==================================================================================================*/

#include "app_include/app_i2s.h" // Function prototypes, constants, preprocessor defs/macros, typedefs

/*---------------------------------------------------------------
    xxxxxxx
---------------------------------------------------------------*/
static i2s_chan_handle_t tx_chan; // TX channel handle

esp_err_t app_i2s_init(const char *TAG) {
    esp_err_t ret;

    // Channel configuration: Auto-select port, Master role
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);

    // Allocate only TX channel
    ret = i2s_new_channel(&chan_cfg, &tx_chan, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2S TX channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // Define standard I2S configuration
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(44100),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,       // Disable MCLK explicitly
            .bclk = I2S_BCLK_PIN,
            .ws   = I2S_WS_PIN,
            .dout = I2S_SDA_PIN,
            .din  = I2S_GPIO_UNUSED,       // TX only, disable input
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false
            }
        }
    };

    // Initialize TX channel in standard mode
    ret = i2s_channel_init_std_mode(tx_chan, &std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init I2S STD mode: %s", esp_err_to_name(ret));
        return ret;
    }

    // Enable TX channel
    ret = i2s_channel_enable(tx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2S TX channel: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2S initialized with MCLK disabled");
    
    return ESP_OK;
}

/*========================================= END FILE ============================================*/