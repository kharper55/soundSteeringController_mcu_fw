/*===================================================================================================
    File: app_bluetooth.c
    Author: Kevin Harper
    Date: 01/2024
    Details: Function and global definitions for ADC peripheral FreeRTOS tasks.

    Written using ESP-IDF v5.1.1 API. Built in 03/2025 using v5.1.2
//==================================================================================================*/

#include "app_include/app_bt.h" // Function prototypes, constants, preprocessor defs/macros, typedefs

// Callback for A2DP events
static void a2dp_sink_cb(const char * TAG, esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param) {
    switch (event) {
        case ESP_A2D_CONNECTION_STATE_EVT:
            ESP_LOGI(TAG, "A2DP connection state: %d", param->conn_stat.state);
            break;
        case ESP_A2D_AUDIO_STATE_EVT:
            ESP_LOGI(TAG, "A2DP audio state: %d", param->audio_stat.state);
            break;
        default:
            break;
    }
}

// Callback for received audio data
void audio_data_callback(const char * TAG, const uint8_t *data, uint32_t len) {
    ESP_LOGI(TAG, "Received audio data: %" PRIu32 " bytes", len);
    // Process the audio data (e.g., send to I2S, DAC, etc.)
}

esp_err_t app_bt_init(const char *TAG) {
    esp_err_t ret;

    // Initialize NVS (Non-Volatile Storage)
    ESP_LOGI(TAG, "Initializing NVS");
    ret = app_nvs_init(TAG);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init failed: 0x%x", ret);
        return ret;
    }

    ESP_LOGI(TAG, "Initializing BT controller");
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_bt_controller_init failed: 0x%x", ret);
        return ret;
    }

    // cONTROLLERN EEDS TO HAVE BTDM ENABLED IN SDK CONFIG
    ESP_LOGI(TAG, "Enabling BT controller");
    ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);  // Or try ESP_BT_MODE_CLASSIC_BT
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_bt_controller_enable failed: 0x%x", ret);
        return ret;
    }

    ESP_LOGI(TAG, "Initializing Bluedroid stack");
    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_bluedroid_init failed: 0x%x", ret);
        return ret;
    }

    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_bluedroid_enable failed: 0x%x", ret);
        return ret;
    }

    ESP_LOGI(TAG, "Registering A2DP callbacks");
    ret = esp_a2d_register_callback(a2dp_sink_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_a2d_register_callback failed: 0x%x", ret);
        return ret;
    }

    ret = esp_a2d_sink_register_data_callback(audio_data_callback);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_a2d_sink_register_data_callback failed: 0x%x", ret);
        return ret;
    }

    ESP_LOGI(TAG, "Initializing A2DP Sink");
    ret = esp_a2d_sink_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_a2d_sink_init failed: 0x%x", ret);
        return ret;
    }

    ESP_LOGI(TAG, "Setting BT name and scan mode");
    esp_bt_dev_set_device_name("A2DP_Sink_Device");

    ret = esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_bt_gap_set_scan_mode failed: 0x%x", ret);
        return ret;
    }

    ESP_LOGI(TAG, "Bluetooth initialization completed");
    return ESP_OK;
}

  /*
    /// Discoverability and Connectability mode
    typedef enum {
        ESP_BT_NON_CONNECTABLE,             // Non-connectable 
        ESP_BT_CONNECTABLE,                 //< Connectable 
    } esp_bt_connection_mode_t;

    typedef enum {
        ESP_BT_NON_DISCOVERABLE,            //!< Non-discoverable 
        ESP_BT_LIMITED_DISCOVERABLE,        //!< Limited Discoverable 
        ESP_BT_GENERAL_DISCOVERABLE,        //!< General Discoverable 
    } esp_bt_discovery_mode_t;
    */

// There is one host, ESP-Bluedroid, supporting Classic Bluetooth in IDF.    

    // ESP32 Bluetooth Classic Stack Architecture
    // 1. Application (top layer - UI)
    // 2. Host (ESP-Bluedroid)
    // 3. ESP Bluetooth Controller (API/peripherals and drivers)

    // ESP32 BLE Stack Architecture
    // 1. Application
    // 2. Profiles (ESP-BLE-MESH, BluFi, HID...)
    // 3. Hosts (ESP-Bluedroid/ESP-NimBLE)
    // 4. ESP Bluetooth Controller (API/peripherals and drivers)

    // BT HCI - Host Controller Interface a standardized protocol that enables 
    //  communication between a Bluetooth host and a Bluetooth controller
    // BTDM - Bluetooth Dual-Mode. Capable of Classic and BLE
    // A2DP - Advanced Audio Distribution Profile

/*========================================= END FILE ============================================*/