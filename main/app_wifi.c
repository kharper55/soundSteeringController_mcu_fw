/*===================================================================================================
    File: app_wifi.c
    Author: Kevin Harper
    Date: 04/2025
    Details: Function and global definitions for WiFi-enabling peripheral FreeRTOS tasks.

    Many ideas in this source code were adapted from https://github.com/Daniel-Wait/ESP32_wifi_nat/tree/main
    Mainly sequencing and argument declaration for all the IDF functions

    Written using ESP-IDF v5.1.1 API. Built in 03/2025 using v5.1.2
//==================================================================================================*/

#include "app_include/app_wifi.h" // Function prototypes, constants, preprocessor defs/macros, typedefs

static EventGroupHandle_t sta_evt_grp;
static int retry_num = 0;

/*---------------------------------------------------------------
    Initializes Non-Volatile Storage peripheral drivers
    for storage of RF PHY calibration constants
---------------------------------------------------------------*/
esp_err_t app_nvs_init(const char * TAG) {
    /* Initialize NVS — it is used to store PHY calibration data */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

/*---------------------------------------------------------------
    Initializes Network Address Port Translation (NAPT) to enable
    routing of traffic from devices connected to the ESP32's
    Wi-Fi SoftAP (AP interface) through the Station (STA) interface,
    which is connected to an upstream network (e.g., internet).

    Note:
    - NAPT is enabled only on the AP interface.
    - STA is set as the default route for outbound traffic.
    - Both AP and STA interfaces must be initialized and up.
    - CONFIG_LWIP_IP_FORWARD and CONFIG_LWIP_L2_TO_L3_COPY must be
      enabled in the project configuration.
---------------------------------------------------------------*/
static esp_err_t app_esp_ip_napt_init(void) {
    esp_err_t err = ESP_OK;

    esp_netif_t* ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    if (ap_netif == NULL) {
        ESP_LOGE("NAPT", "Failed to get AP interface.");
        return ESP_FAIL;
    }

    if (!esp_netif_is_netif_up(ap_netif)) {
        ESP_LOGE("NAPT", "AP netif not up yet.");
        return ESP_FAIL;
    }

    esp_netif_t* sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (sta_netif == NULL) {
        ESP_LOGE("NAPT", "Failed to get STA interface.");
        return ESP_FAIL;
    }

    if (!esp_netif_is_netif_up(sta_netif)) {
        ESP_LOGE("NAPT", "STA netif not up yet.");
        return ESP_FAIL;
    }

    err = esp_netif_napt_enable(ap_netif);
    if (err != ESP_OK) {
        ESP_LOGE("NAPT", "Failed to enable NAPT: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }

    esp_netif_ip_info_t ap_ip_info;
    err = esp_netif_get_ip_info(ap_netif, &ap_ip_info);
    if (err == ESP_OK) {
        ESP_LOGI("NAPT", "NAPT enabled on AP with IP: " IPSTR, IP2STR(&ap_ip_info.ip));
    } else {
        ESP_LOGW("NAPT", "Failed to get AP IP info: %s", esp_err_to_name(err));
    }

    err = esp_netif_set_default_netif(sta_netif);
    if (err == ESP_OK) {
        esp_netif_ip_info_t sta_ip_info;
        if (esp_netif_get_ip_info(sta_netif, &sta_ip_info) == ESP_OK) {
            ESP_LOGI("NAPT", "STA set as default route. STA IP: " IPSTR, IP2STR(&sta_ip_info.ip));
        }
    } else {
        ESP_LOGW("NAPT", "Failed to set STA as default netif.");
        return ESP_FAIL;
    }

    return ESP_OK;
}

/*---------------------------------------------------------------
    Event handler for Wi-Fi SoftAP events. This function conforms
    to the standard ESP-IDF event handler signature.

    Handles:
    - WIFI_EVENT_AP_STACONNECTED: Logs when a station connects to the AP.
    - WIFI_EVENT_AP_STADISCONNECTED: Logs when a station disconnects from the AP. 
---------------------------------------------------------------*/
static void ap_wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data) {

    // Handle the event that a STA connected to the AP
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI("AP", "Station "MACSTR" joined, AID=%d.", MAC2STR(event->mac), event->aid);
    } 

    // Handle the event that a STA disconnected from the AP
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI("AP", "Station "MACSTR" left, AID=%d.", MAC2STR(event->mac), event->aid);
    } 
}

/*---------------------------------------------------------------
    Event handler for Station (STA) mode events and IP events.
    Handles connection state changes and IP acquisition.
    Handles events related to the ESP32 connecting to an external 
    Wi-Fi network as a client. This function conforms
    to the standard ESP-IDF event handler signature.
---------------------------------------------------------------*/
static void sta_wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data) {
    
    wifi_sta_context_t * ctx = (wifi_sta_context_t *)arg;

    // Wait until app_wifi_init() reaches start phase, at which point the 
    // WIFI_EVENT_STA_START event will be generated.
    // This event handler is technically part of the WIFI STA initialization procedure
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI("STA", "Attempting to connect to the AP.");
        ESP_ERROR_CHECK(esp_wifi_connect()); // Connect the STA to the AP
    } 

    // Handle the event that the STA disconnects from an AP
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW("STA","Lost connection to the AP.");
        // Retry to connect to the AP
        if (retry_num < STA_WIFI_MAX_RETRY) {
            ESP_ERROR_CHECK(esp_wifi_connect()); // Connect the STA to the AP
            retry_num++;
            ESP_LOGI("STA", "Retrying to connect to the AP (%d/%d attempts).", retry_num, STA_WIFI_MAX_RETRY);
        } else {
            xEventGroupSetBits(sta_evt_grp, STA_WIFI_FAIL_BIT);
            ESP_LOGW("STA","Connecting to the AP failed. Number of reconnect attempts (%d) exceeded.", STA_WIFI_MAX_RETRY);
        }
    } 

    // Handle the event that an IP address has successfully been acquired via 
    // Dynamic Host Configuration Protocol (DCHP)
    // Also used to initialize DNS relay and NAPT when NAT is enabled
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI("STA", "DHCP-granted IP!");
    
        // Get DNS server info from lwIP (index 0)
        const ip_addr_t* lwip_dns_ip = dns_getserver(0);
        esp_netif_dns_info_t dns_info = { 0 };
    
        if (lwip_dns_ip && lwip_dns_ip->type == IPADDR_TYPE_V4) {
            memcpy(&dns_info.ip.u_addr.ip4, &lwip_dns_ip->u_addr.ip4, sizeof(esp_ip4_addr_t));
            dns_info.ip.type = IPADDR_TYPE_V4;
            ESP_LOGI("STA", "Using IPv4 DNS server: " IPSTR, IP2STR(&dns_info.ip.u_addr.ip4));
        } else {
            ESP_LOGW("STA", "Invalid or missing DNS info from STA interface.");
            return;
        }
    
        retry_num = 0;
        xEventGroupSetBits(sta_evt_grp, STA_WIFI_CONNECTED_BIT);
    
        if (ctx && ctx->NAT_enabled && ctx->mode == WIFI_MODE_APSTA) {
            // Set DNS for AP interface (i.e. DHCP clients on ESP32 AP)
            esp_netif_t* ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
            if (ap_netif) {
                ESP_ERROR_CHECK(esp_netif_dhcps_stop(ap_netif)); // Need to stop the dhcp server before modifying
    
                // Tell DHCP server to use custom DNS (value = 2)
                uint8_t opt_val = 2;
                ESP_ERROR_CHECK(esp_netif_dhcps_option(ap_netif,
                    ESP_NETIF_OP_SET, ESP_NETIF_DOMAIN_NAME_SERVER,
                    &opt_val, sizeof(opt_val)));
    
                ESP_ERROR_CHECK(esp_netif_set_dns_info(ap_netif, ESP_NETIF_DNS_MAIN, &dns_info));
                ESP_ERROR_CHECK(esp_netif_dhcps_start(ap_netif));
                ESP_LOGI("STA", "DNS propagated to AP clients.");
            }
    
            // Set DNS for STA interface (optional, for consistency)
            esp_netif_t* sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
            if (sta_netif) {
                ESP_ERROR_CHECK(esp_netif_set_dns_info(sta_netif, ESP_NETIF_DNS_MAIN, &dns_info));
                ESP_LOGI("STA", "DNS reaffirmed for STA interface.");
            }
    
            // Start NAPT
            ESP_ERROR_CHECK(app_esp_ip_napt_init());
        }
    }
}

/*---------------------------------------------------------------
    Deinitialize Wi-Fi and free associated resources.

    This function disconnects from any connected Wi-Fi networks, 
    stops the Wi-Fi driver, and deinitializes it. Useful when 
    transitioning to a different Wi-Fi mode, shutting down 
    networking to save power, or cleaning up before reboot/reset.

    Returns:
        ESP_OK on success,
        ESP_ERR_WIFI_NOT_INIT if Wi-Fi is not initialized,
        or another esp_err_t code on failure.
---------------------------------------------------------------*/
esp_err_t app_wifi_deinit(void) {

    esp_err_t ret = ESP_OK;

    // Disconnect WiFi station from the AP. Returns ESP_OK, ESP_FAIL, or ESP_ERR_WIFI_NOT_INIT
    ret = esp_wifi_disconnect();
    if (ret != ESP_OK) return ret; // Error handling should take place upon function return

    // Stop WiFi and free the appropriate control block. Returns ESP_OK or ESP_ERR_WIFI_NOT_INIT
    ret = esp_wifi_stop(); 
    if (ret != ESP_OK) return ret;
    
    // Free all resource allocated in esp_wifi_init() and stop WiFi task. Returns ESP_OK or ESP_ERR_WIFI_NOT_INIT
    ret = esp_wifi_deinit();

    return ret;
}

/*---------------------------------------------------------------
    Initialize WiFi in AP, STA, or APSTA mode.

    AP Mode - Create own Wi-Fi network (SSID), acting as a hotspot.
              Other devices (like phones or laptops) can connect directly to the ESP32.
    STA Mode - Connect to an existing Wi-Fi network (e.g., your home router) 
               and behaves like a typical device
---------------------------------------------------------------*/
esp_err_t app_wifi_init(const char * TAG, wifi_mode_t mode, bool NATen) {

    esp_err_t ret = ESP_OK;
    wifi_init_config_t wfcfg = WIFI_INIT_CONFIG_DEFAULT(); // for esp_wifi_init()
    //esp_event_loop_args_t event_loop_args; // for esp_event_loop_create()
    //esp_event_loop_handle_t event_loop;    // for esp_event_loop_create()

    // Check user input mode selection
    if (!(mode == WIFI_MODE_AP || mode == WIFI_MODE_STA || mode == WIFI_MODE_APSTA)) return ESP_FAIL;

    // Initialize NVS as it is used to store RF calibration data
    ret = app_nvs_init(TAG);
    if (ret != ESP_OK) return ret;

    ip_addr_t dnsserver;
    sta_evt_grp = xEventGroupCreate();

    // ============ 1. Wi-Fi/LwIP Init Phase ============ //

     // Create an LwIP core task and initialize LwIP-related work.
    ret = esp_netif_init();
    if (ret != ESP_OK) return ret;

    // Create a system Event task and initialize an application event's callback function
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK) return ret;

    // Create default WIFI AP/STA. In case of any init error this API aborts. Returns pointer to esp-netif instance
    esp_netif_t *ap_netif;
    esp_netif_t *sta_netif;

    // Must create both for APSTA mode
    if (mode == WIFI_MODE_AP || mode == WIFI_MODE_APSTA) { // If APSTA, it will already have STA configured by this point
        ap_netif = esp_netif_create_default_wifi_ap();
    }
    if (mode == WIFI_MODE_STA || mode == WIFI_MODE_APSTA) {
        sta_netif = esp_netif_create_default_wifi_sta();
    } 

    // Initialize WiFi Allocate resource for WiFi driver, such as WiFi control structure, RX/TX buffer, WiFi NVS 
    // structure etc. This also starts WiFi task.
    ret = esp_wifi_init(&wfcfg);
    if (ret != ESP_OK) return ret;
    
    if (mode == WIFI_MODE_AP || mode == WIFI_MODE_APSTA) {
        esp_wifi_set_ps(WIFI_PS_NONE); // Disable power save in AP. WIFI_PS_MIN_MODEM - WIFI_PS_MAX_MODEM  - Update the power save config accordingly after esp_wifi_set_mode() if you're switching modes dynamically
    }

    /* Station Event Handler Register*/
    esp_event_handler_instance_t sta_any_id;
    esp_event_handler_instance_t sta_got_ip;
    if (mode == WIFI_MODE_STA || mode == WIFI_MODE_APSTA) {
        wifi_sta_context_t *sta_ctx = malloc(sizeof(wifi_sta_context_t)); // dangling.. pass its own pointer loc as a member?
        if (!sta_ctx) return ESP_ERR_NO_MEM;
        sta_ctx->NAT_enabled = NATen;
        sta_ctx->mode = mode;
        ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                            ESP_EVENT_ANY_ID,
                                                            &sta_wifi_event_handler,
                                                            (void *)sta_ctx,
                                                            &sta_any_id));
        ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                            IP_EVENT_STA_GOT_IP,
                                                            &sta_wifi_event_handler,
                                                            (void *)sta_ctx,
                                                            &sta_got_ip));
    }

    /* Access Point Event Handler Register*/
    esp_event_handler_instance_t ap_any_id;
    esp_event_handler_instance_t ap_got_ip;
    if (mode == WIFI_MODE_AP || mode == WIFI_MODE_APSTA) {
        ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                            ESP_EVENT_ANY_ID,
                                                            &ap_wifi_event_handler,
                                                            NULL,
                                                            &ap_any_id));
        ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                            IP_EVENT_AP_STAIPASSIGNED,
                                                            &ap_wifi_event_handler,
                                                            NULL,
                                                            &ap_got_ip));
    }

    // ============ 2. Configuration Phase ============ //

    /* Station Wifi Config*/
    wifi_config_t sta_wifi_config = {
        .sta = {
            .ssid = STA_WIFI_SSID,
            .password = STA_WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	        .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };

    /* Access Point Wifi Config*/
    wifi_config_t ap_wifi_config = {
        .ap = {
            .ssid = AP_WIFI_SSID,
            .ssid_len = strlen(AP_WIFI_SSID),
            .channel = AP_WIFI_CHANNEL,
            .password = AP_WIFI_PASS,
            .max_connection = AP_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    if (strlen(AP_WIFI_PASS) == 0) {
        ap_wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    // Configure the Wi-Fi mode as AP/STA or both
    ret = esp_wifi_set_mode(mode); // (WIFI_MODE_STA/WIFI_MODE_AP) to 
    if (ret != ESP_OK) return ret;
    
    // Always configure both for APSTA mode
    if (mode == WIFI_MODE_STA || mode == WIFI_MODE_APSTA) {
        ret = esp_wifi_set_config(WIFI_IF_STA, &sta_wifi_config);
        if (ret != ESP_OK) return ret;
    } 
    if (mode == WIFI_MODE_AP || mode == WIFI_MODE_APSTA) { // If APSTA, it will already have STA configured by this point
        ret = esp_wifi_set_config(WIFI_IF_AP, &ap_wifi_config);
        if (ret != ESP_OK) return ret;
    }
 
    // ============ 3. Start Phase ============ //

    // Start the Wi-Fi driver which posts WIFI_EVENT_STA_START to the event task; then, the event 
    // task will do some common things and will call the application event callback function.
    ret = esp_wifi_start(); 
    if (ret != ESP_OK) return ret;

    switch(mode) {
        case WIFI_MODE_AP:
            ESP_LOGI(TAG, "Wi-Fi started in Access Point mode. SSID: %s, Channel: %d", 
                     AP_WIFI_SSID, AP_WIFI_CHANNEL);
            break;
        case WIFI_MODE_STA:
            ESP_LOGI(TAG, "Wi-Fi started in Station mode. Attempting to connect to SSID: %s", 
                     STA_WIFI_SSID);
            break;
        case WIFI_MODE_APSTA:
            ESP_LOGI(TAG, "Wi-Fi started in AP+STA mode. AP SSID: %s, STA SSID: %s, Channel: %d", 
                     AP_WIFI_SSID, STA_WIFI_SSID, AP_WIFI_CHANNEL);
            break;
        default:
            break;
    }

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) 
    or connection failed for the maximum number of retries (WIFI_FAIL_BIT). */
    if (mode == WIFI_MODE_STA || mode == WIFI_MODE_APSTA) {
        EventBits_t bits = xEventGroupWaitBits(sta_evt_grp,
                STA_WIFI_CONNECTED_BIT | STA_WIFI_FAIL_BIT,
                pdFALSE,
                pdFALSE,
                portMAX_DELAY); // Wait indefinitely for connection or failure

        /* xEventGroupWaitBits() returns the bits before the call returned, 
        hence we can test which event actually happened. */
        if (bits & STA_WIFI_CONNECTED_BIT) {
            ESP_LOGI(TAG, "Connected to AP with SSID:%s", STA_WIFI_SSID);
        } else {
            ESP_LOGI(TAG, "Failed to connect to AP with SSID:%s", STA_WIFI_SSID);
        }
    } else {
        ESP_LOGI(TAG, "Running in AP mode, no need to block for connection.");
    }

    /* The event will not be processed after unregister */
    //ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, ap_any_id));
    //ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, ap_got_ip));
    //vEventGroupDelete(sta_evt_grp);

    // ============ 4. Wi-Fi Connect Phase ============ //

    // Wi-Fi driver will start the internal scan/connection process. If the internal scan/connection process is successful, 
    // the WIFI_EVENT_STA_CONNECTED will be generated. The Wi-Fi connection may fail because, for example, the password is 
    // wrong, or the AP is not found. In a case like this, WIFI_EVENT_STA_DISCONNECTED will arise and the reason for such
    // a failure will be provided. For handling events that disrupt Wi-Fi connection, please refer to phase 5. 

    // Connect WiFi station to the AP.
    //ret = esp_wifi_connect(); // this is now called in the appropriate handler
    //if (ret != ESP_OK) return ret;

    return ret;
}