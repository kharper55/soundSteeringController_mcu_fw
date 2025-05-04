/*===================================================================================================
    File: app_wifi.c
    Author: Kevin Harper
    Date: 04/2025
    Details: Function and global definitions for WiFi-enabling peripheral FreeRTOS tasks.

    Written using ESP-IDF v5.1.1 API. Built in 03/2025 using v5.1.2
//==================================================================================================*/

#include "app_include/app_wifi.h" // Function prototypes, constants, preprocessor defs/macros, typedefs

static EventGroupHandle_t sta_evt_grp;
static int retry_num = 0;

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

    // Try to get the AP interface handle
    esp_netif_t* ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    if (ap_netif == NULL) {
        ESP_LOGE("NAPT", "Failed to get AP interface.");
        return ESP_FAIL;
    }

    // Verify the AP interace is up
    if (!esp_netif_is_netif_up(ap_netif)) {
        ESP_LOGE("NAPT", "AP netif not up yet.");
        return ESP_FAIL;
    }

    // Try to get the STA interface handle
    esp_netif_t* sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (ap_netif == NULL) {
        ESP_LOGE("NAPT", "Failed to get STA interface.");
        return ESP_FAIL;
    }

    // Verify the STA interace is up
    if (!esp_netif_is_netif_up(sta_netif)) {
        ESP_LOGE("NAPT", "STA netif not up yet.");
        return ESP_FAIL;
    }

    // Enable NAPT only if getting the interface succeeds
    err = esp_netif_napt_enable(ap_netif);
    if (err != ESP_OK) {
        ESP_LOGE("NAPT", "Failed to enable NAPT: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }

    // Proceed to get IP info if NAPT was enabled
    esp_netif_ip_info_t ip_info;
    err = esp_netif_get_ip_info(ap_netif, &ip_info);
    if (err == ESP_OK) {
        ESP_LOGI("NAPT", "NAPT enabled on SoftAP interface with IP: " IPSTR, IP2STR(&ip_info.ip));
    } else {
        ESP_LOGW("NAPT", "Failed to get AP IP info: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }

    // Set default route. Some LwIP setups require this
    err = esp_netif_set_default_netif(sta_netif); 
    if (err == ESP_OK) {
        ESP_LOGI("NAPT", "STA interface set for default route." IPSTR, IP2STR(&ip_info.ip));
    } else {
        ESP_LOGW("NAPT", "Failed to set STA interface as default route.");
        return ESP_FAIL;
    }

    return err;
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

        // Cast event data to IP event structure to access assigned IP info
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI("STA", "DHCP-granted IP:" IPSTR, IP2STR(&event->ip_info.ip));
        retry_num = 0; // Reset counter upon successful IP acquisition

        // Signal that STA WIFI is connected and has IP
        xEventGroupSetBits(sta_evt_grp, STA_WIFI_CONNECTED_BIT);

        if (ctx && ctx->NAT_enabled && ctx->mode == WIFI_MODE_APSTA) {
            // Retrieve current DNS server from LwIP
            const ip_addr_t* lwip_dns_ip = dns_getserver(0);

            // Prepare esp_netif_dns_info_t structure to set DNS on AP interface
            esp_netif_dns_info_t dns_info = { 0 }; 
            
            // Copy DNS IP based on whether it's IPv4 or IPv6
            dns_info.ip.type = lwip_dns_ip->type;
            if (lwip_dns_ip->type == IPADDR_TYPE_V4) {
                dns_info.ip.u_addr.ip4.addr = lwip_dns_ip->u_addr.ip4.addr;
            } else if (lwip_dns_ip->type == IPADDR_TYPE_V6) {
                memcpy(&dns_info.ip.u_addr.ip6, &lwip_dns_ip->u_addr.ip6, sizeof(ip6_addr_t));
            }

            // Get the handle to the AP network interface
            esp_netif_t * ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");

            if (ap_netif) {
                // Set the DNS server info for the AP interface so clients can resolve domains
                ESP_ERROR_CHECK(esp_netif_set_dns_info(ap_netif, ESP_NETIF_DNS_MAIN, &dns_info));
            } else {
                ESP_LOGW("STA", "Failed to get AP netif handle for DNS setup.");
            }

            // If in APSTA mode and NAT is enabled, propagate DNS to AP and start NAPT
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

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

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