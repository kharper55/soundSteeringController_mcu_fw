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
    generic wifi event handler signature
---------------------------------------------------------------*/
static void app_esp_ip_napt_init(void) {

    // Ensure the network interface is created and initialized before enabling NAPT
    esp_netif_ip_info_t ip_info;
    esp_netif_t * ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");

    // Enable NAPT (Network Address Port Translation) with the IP address pointer
    if (ap_netif != NULL && esp_netif_is_netif_up(ap_netif)) {
        ESP_ERROR_CHECK(esp_netif_napt_enable(ap_netif));
        ESP_ERROR_CHECK(esp_netif_get_ip_info(ap_netif, &ip_info));
        ESP_LOGI("NAPT", "NAPT enabled on SoftAP interface with IP: " IPSTR, IP2STR(&ip_info.ip));
    } else {
        ESP_LOGW("NAPT", "AP netif not up yet, deferring NAPT enable...");
    }

    // Set default route. Some LwIP setups require this
    esp_netif_t * sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    ESP_ERROR_CHECK(esp_netif_get_ip_info(sta_netif, &ip_info));
    ESP_ERROR_CHECK(esp_netif_set_default_netif(sta_netif)); 
    ESP_LOGI("NAPT", "STA set to default interface route with IP: " IPSTR, IP2STR(&ip_info.ip));

    // Note that both CONFIG_LWIP_IP_FORWARD and CONFIG_LWIP_L2_TO_L3_COPY options
    // need to be enabled in system configuration for the NAPT to work on ESP platform

    /*
    Confirm also:

    You're calling app_esp_ip_napt_init() after the AP and STA are fully initialized.
    You only enable NAPT on the AP interface, not the STA.
    You are not setting the IP manually if you're letting the DHCP server assign it.
    */

}

/*---------------------------------------------------------------
    generic wifi event handler signature
---------------------------------------------------------------*/
static void ap_wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data) {
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI("AP", "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI("AP", "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } //else if (event_base == IP_EVENT && event_id == IP_EVENT_AP_STAIPASSIGNED) {
      //  ESP_LOGI("AP", "Station got IP from DHCP");
      //  app_esp_ip_napt_init(); // Enable NAPT here
    //}
}

/*---------------------------------------------------------------
    generic wifi event handler signature
---------------------------------------------------------------*/
static void sta_wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (retry_num < STA_WIFI_MAX_RETRY) {
            esp_wifi_connect();
            retry_num++;
            ESP_LOGI("STA", "retry to connect to the AP");
        } else {
            xEventGroupSetBits(sta_evt_grp, STA_WIFI_FAIL_BIT);
        }
        ESP_LOGI("STA","connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI("STA", "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        retry_num = 0;
        xEventGroupSetBits(sta_evt_grp, STA_WIFI_CONNECTED_BIT);
        // Enable DNS (offer) for dhcp server
        //dhcps_offer_t dhcps_dns_value = OFFER_DNS;
        //dhcps_set_option_info(6, &dhcps_dns_value, sizeof(dhcps_dns_value)); // deprecated
        // After STA is connected and got IP:
        const ip_addr_t* lwip_dns_ip = dns_getserver(0);  // From LwIP

        esp_netif_dns_info_t dns_info = { 0 };

        // Copy using correct field
        dns_info.ip.type = lwip_dns_ip->type;

        if (lwip_dns_ip->type == IPADDR_TYPE_V4) {
            dns_info.ip.u_addr.ip4.addr = lwip_dns_ip->u_addr.ip4.addr;
        } else if (lwip_dns_ip->type == IPADDR_TYPE_V6) {
            memcpy(&dns_info.ip.u_addr.ip6, &lwip_dns_ip->u_addr.ip6, sizeof(ip6_addr_t));
        }
        esp_netif_t* ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
        ESP_ERROR_CHECK(esp_netif_set_dns_info(ap_netif, ESP_NETIF_DNS_MAIN, &dns_info));  

        app_esp_ip_napt_init(); // should happen after STA has an IP, or it may forward incorrectly.
    }
}

/*---------------------------------------------------------------
    esp_err_t app_wifi_deinit(void)
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
    esp_err_t app_wifi_init(wifi_mode_t mode)

    AP Mode - Create own Wi-Fi network (SSID), acting as a hotspot.
              Other devices (like phones or laptops) can connect directly to the ESP32.
    STA Mode - Connect to an existing Wi-Fi network (e.g., your home router) 
               and behaves like a typical device
    NAT - Network Area Translation. Need to route traffic from connected devices on
          AP to the internet via station connection
---------------------------------------------------------------*/
esp_err_t app_wifi_init(const char * TAG, wifi_mode_t mode) {

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

    /* Station Event Handler Register*/
    esp_event_handler_instance_t sta_any_id;
    esp_event_handler_instance_t sta_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &sta_wifi_event_handler,
                                                        NULL,
                                                        &sta_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &sta_wifi_event_handler,
                                                        NULL,
                                                        &sta_got_ip));

    /* Access Point Event Handler Register*/
    esp_event_handler_instance_t ap_any_id;
    esp_event_handler_instance_t ap_got_ip;
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

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             AP_WIFI_SSID, AP_WIFI_PASS, AP_WIFI_CHANNEL);
    

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(sta_evt_grp,
            STA_WIFI_CONNECTED_BIT | STA_WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & STA_WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 STA_WIFI_SSID, STA_WIFI_PASS);
    } else if (bits & STA_WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 STA_WIFI_SSID, STA_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, ap_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, ap_got_ip));
    vEventGroupDelete(sta_evt_grp);

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

/*========================================= END FILE ============================================*/