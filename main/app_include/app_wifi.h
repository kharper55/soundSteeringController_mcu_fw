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
#include "nvs_flash.h"
#include "netif/ethernet.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/netif.h"
#include "lwip/lwip_napt.h"
#include "lwip/dns.h"

// Pin Defines

// Settings

#define APP_WIFI_MODE WIFI_MODE_APSTA /*WIFI_MODE_AP, WIFI_MODE_APSTA, WIFI_MODE_STA*/
#define USE_NAT_IF_APSTA 1

// ESP32 SoftAP Credentials
#define AP_WIFI_SSID             "SoundsteeringAP"
#define AP_WIFI_PASS             "11111111"
#define AP_WIFI_CHANNEL          0
#define AP_MAX_STA_CONN          4

// User WiFi credentials (STA/APSTA mode)
#define STA_WIFI_SSID            "TMOBILE-6CB9" // Fetch user credentials via captive protal if necessary
#define STA_WIFI_PASS            "jizz1234"
#define STA_WIFI_MAX_RETRY       10
#define STA_WIFI_CONNECTED_BIT   BIT0
#define STA_WIFI_FAIL_BIT        BIT1

#define MY_DNS_IP_ADDR           0x08080808 // 8.8.8.8

// Macros

// Typedefs
typedef struct {
    bool NAT_enabled;
    wifi_mode_t mode;
} wifi_sta_context_t;

// General parameters for the FreeRTOS WiFi task
typedef struct {
    char * TAG;
    wifi_mode_t mode;
} wifiParams_t;

// User functions
esp_err_t app_wifi_deinit(void);
esp_err_t app_wifi_init(const char * TAG, wifi_mode_t mode, bool NATen);
esp_err_t app_nvs_init(const char * TAG);

#ifdef __cplusplus
}
#endif

#endif  // APP_WIFI_H

/*========================================= END FILE ============================================*/