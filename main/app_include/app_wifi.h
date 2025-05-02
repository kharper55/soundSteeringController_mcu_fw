/*===================================================================================================
    File: app_wifi.h
    Author: Kevin Harper
    Date: 04/2025
    Details: Function prototypes, constants, preprocessor defs/macros for WiFi (802.11) peripheral
             All BT and networking features should be implemented with the PRO CPU core. 
             Make sure to use sdkConfig ESP IDF tool to include the driver interfaces in your project.
             
    Written using ESP-IDF v5.1.1 API. Built in 03/2025 using v5.1.2
//==================================================================================================*/

#ifndef APP_WIFI_H
#define APP_WIFI_H

#ifdef __cplusplus
extern "C" {
#endif

// Includes
#include "app_utility.h" // for esp_err_t
#include "esp_wifi.h"

// Pin Defines

// Settings

// Macros

// Typedefs

// General parameters for the FreeRTOS WiFi task
typedef struct {
    char * TAG;
    wifi_mode_t mode;
    //adc_oneshot_unit_handle_t * handle;
    //adc_cali_handle_t * cali_handle;
    //adc_unit_t unit;
    //adc_channel_t channel;
    //adc_atten_t atten;
    //adc_filter_t * filt;
    //int delay_ms;
    //int * vraw;
    //int * vcal;
    //int * vfilt;
} wifiParams_t;

// User functions
esp_err_t app_wifi_deinit(void);
esp_err_t app_wifi_init(wifi_mode_t mode);

/*
Typical Pattern: Wi-Fi Provisioning Flow

    1. Boot into AP mode if no known Wi-Fi credentials.
    2. Host a captive portal (ESP-IDF supports this).
    3. User connects to ESP32 AP, enters Wi-Fi credentials.
    4. ESP32 saves credentials in NVS and reboots into STA mode.
    5. ESP32 connects to the user’s Wi-Fi and serves the web page there.

ESP-IDF provides Wi-Fi provisioning libraries (wifi_provisioning component) with:

    - BLE or SoftAP-based setup
    - Captive portal
    - Secure credential storage
*/

#ifdef __cplusplus
}
#endif

#endif  // APP_WIFI_H

/*========================================= END FILE ============================================*/