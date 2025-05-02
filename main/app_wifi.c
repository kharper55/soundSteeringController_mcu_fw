/*===================================================================================================
    File: app_wifi.c
    Author: Kevin Harper
    Date: 04/2025
    Details: Function and global definitions for WiFi-enabling peripheral FreeRTOS tasks.

    Written using ESP-IDF v5.1.1 API. Built in 03/2025 using v5.1.2
//==================================================================================================*/

#include "app_include/app_wifi.h" // Function prototypes, constants, preprocessor defs/macros, typedefs

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
---------------------------------------------------------------*/
esp_err_t app_wifi_init(wifi_mode_t mode) {

    esp_err_t ret = ESP_OK;
    wifi_init_config_t wfcfg = WIFI_INIT_CONFIG_DEFAULT(); // for esp_wifi_init()
    //esp_event_loop_args_t event_loop_args; // for esp_event_loop_create()
    //esp_event_loop_handle_t event_loop;    // for esp_event_loop_create()

    /*
    esp_event_loop_args_t event_loop_args = {
        .queue_size = 32,
        .task_name = "event_task",
        .task_priority = uxTaskPriorityGet(NULL),
        .task_stack_size = 2048,
        .task_core_id = tskNO_AFFINITY,
        .dispatch_method = ESP_EVENT_ANY_BASE,
    };
    */

    // Check user input mode selection
    if (!(mode == WIFI_MODE_AP || mode == WIFI_MODE_STA || mode == WIFI_MODE_APSTA)) return ESP_FAIL;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // ============ 1. Wi-Fi/LwIP Init Phase ============ //

     // Create an LwIP core task and initialize LwIP-related work.
    ret = esp_netif_init();
    if (ret != ESP_OK) return ret;

    // Create a system Event task and initialize an application event's callback function
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK) return ret;

    // Create default WIFI AP/STA. In case of any init error this API aborts. Returns pointer to esp-netif instance
    esp_netif_t *netif;
    switch(mode) {
        case(WIFI_MODE_AP):
            netif = esp_netif_create_default_wifi_ap();
            break;
        case(WIFI_MODE_STA):
            netif = esp_netif_create_default_wifi_sta();
            break;
        case(WIFI_MODE_APSTA):
                // Must create *both*
                esp_netif_create_default_wifi_ap();
                netif = esp_netif_create_default_wifi_sta();
            break;
        // Guaranteed to never execute
        default:
            break;
    }

    // Initialize WiFi Allocate resource for WiFi driver, such as WiFi control structure, RX/TX buffer, WiFi NVS 
    // structure etc. This also starts WiFi task.
    ret = esp_wifi_init(&wfcfg);
    if (ret != ESP_OK) return ret;

    // ============ 2. Configuration Phase ============ //

    // Configure the Wi-Fi mode as AP/STA or both
    ret = esp_wifi_set_mode(mode); // (WIFI_MODE_STA/WIFI_MODE_AP) to 
    if (ret != ESP_OK) return ret;

    wifi_config_t sta_cfg = {
        .sta = {
            .ssid = "TMOBILE-6CB9" /* YOUR SSID */,
            .password = "jizz1234" /* YOUR PASSWORD */,
            .threshold.authmode = /*WIFI_AUTH_WPA3_PSK*/ WIFI_AUTH_WPA2_PSK /*WIFI_AUTH_WEP*/ /*WIFI_AUTH_WPA_PSK*/ /*WIFI_AUTH_WPA_WPA2_PSK*/
            // Often required with WPA3-Personal auth mode
            ,.pmf_cfg = {
                .required = false,
                .capable = true,
            }
            
        }
    };

    wifi_config_t ap_cfg = {
        .ap = {
            .ssid = "KevinESP.AP",
            .password = "11111111",
            .ssid_len = 0,
            .channel = 1,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .max_connection = 4,
            .pmf_cfg = {
                .required = false,
                .capable = true,
            }
        }
    };
    
    // Always configure both for APSTA mode
    if (mode == WIFI_MODE_STA || mode == WIFI_MODE_APSTA) {
        ret = esp_wifi_set_config(WIFI_IF_STA, &sta_cfg);
        if (ret != ESP_OK) return ret;
    } 
    if (mode == WIFI_MODE_AP || mode == WIFI_MODE_APSTA) { // If APSTA, it will already have STA configured by this point
        ret = esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
        if (ret != ESP_OK) return ret;
    }

    // ============ 3. Start Phase ============ //

    // Start the Wi-Fi driver which posts WIFI_EVENT_STA_START to the event task; then, the event 
    // task will do some common things and will call the application event callback function.
    ret = esp_wifi_start(); 
    if (ret != ESP_OK) return ret;

    // ============ 4. Wi-Fi Connect Phase ============ //

    // Wi-Fi driver will start the internal scan/connection process. If the internal scan/connection process is successful, 
    // the WIFI_EVENT_STA_CONNECTED will be generated. The Wi-Fi connection may fail because, for example, the password is 
    // wrong, or the AP is not found. In a case like this, WIFI_EVENT_STA_DISCONNECTED will arise and the reason for such
    // a failure will be provided. For handling events that disrupt Wi-Fi connection, please refer to phase 5. 

    // Connect WiFi station to the AP.
    ret = esp_wifi_connect();
    if (ret != ESP_OK) return ret;

    return ret;

    /*
    ESP32 Wi-Fi AP General Scenario

    1. Wi-Fi/LwIP Init Phase

        Step 1.1 ~ 1.5 is a recommended sequence that initializes a Wi-Fi-/LwIP-based application. 
        However, it is NOT a must-follow sequence, which means that you can create the application task 
        in step 1.1 and put all other initialization in the application task. Moreover, you may not want to 
        create the application task in the initialization phase if the application task depends on the sockets. 
        Rather, you can defer the task creation until the IP is obtained.

        1.1 - The main task calls esp_netif_init() to create an LwIP core task and initialize LwIP-related work.
        1.2 - The main task calls esp_event_loop_create() to create a system Event task and initialize an application 
              event's callback function. In the scenario above, the application event's callback function does nothing
              but relaying the event to the application task.
        1.3 - The main task calls esp_netif_create_default_wifi_ap() or esp_netif_create_default_wifi_sta() to create 
              default network interface instance binding station or AP with TCP/IP stack.
        1.4 - The main task calls esp_wifi_init() to create the Wi-Fi driver task and initialize the Wi-Fi driver.
        1.5 - The main task calls OS API to create the application task.

        OR

        1.1 - The main task calls OS API to create the APPLICATION task.
        1.2 - The APPLICATION task calls esp_netif_init() to create an LwIP core task and initialize LwIP-related work.
        1.3 - The APPLICATION task calls esp_event_loop_create() to create a system Event task and initialize THE application 
              event's callback function.
        1.4 - The APPLICATION task calls esp_netif_create_default_wifi_ap() or esp_netif_create_default_wifi_sta() to create 
              default network interface instance binding station or AP with TCP/IP stack.
        1.5 - The APPLICATION task calls esp_wifi_init() to create the Wi-Fi driver task and initialize the Wi-Fi driver.
        
    2. Wi-Fi Configuration Phase

        Once the Wi-Fi driver is initialized, you can start configuring the Wi-Fi driver. In this scenario, 
        the mode is AP, so you may need to call esp_wifi_set_mode() (WIFI_MODE_AP) to configure the Wi-Fi 
        mode as AP. You can call other esp_wifi_set_xxx APIs to configure more settings, such as the protocol mode, 
        the country code, and the bandwidth. Refer to ESP32 Wi-Fi Configuration.

        Generally, the Wi-Fi driver should be configured before the Wi-Fi connection is set up. But this is NOT 
        mandatory, which means that you can configure the Wi-Fi connection anytime, provided that the Wi-Fi driver is 
        initialized successfully. However, if the configuration does not need to change after the Wi-Fi connection is 
        set up, you should configure the Wi-Fi driver at this stage, because the configuration APIs 
        (such as esp_wifi_set_protocol()) will cause the Wi-Fi to reconnect, which may not be desirable.

        If the Wi-Fi NVS flash is enabled by menuconfig, all Wi-Fi configuration in this phase, or later phases, 
        will be stored into flash. When the board powers on/reboots, you do not need to configure the Wi-Fi driver 
        from scratch. You only need to call esp_wifi_get_xxx APIs to fetch the configuration stored in flash previously. 
        You can also configure the Wi-Fi driver if the previous configuration is not what you want.

    3. Wi-Fi Start Phase

        3.1 - Call esp_wifi_start() to start the Wi-Fi driver.
        3.2 - The Wi-Fi driver posts WIFI_EVENT_STA_START to the event task; then, the event task will do some common 
              things and will call the application event callback function.
        3.3 - The application event callback function relays the WIFI_EVENT_STA_START to the application task. We 
              recommend that you call esp_wifi_connect(). However, you can also call esp_wifi_connect() in other phrases after 
              the WIFI_EVENT_STA_START arises.

    4. Wi-Fi Connect Phase

        4.1 - Once esp_wifi_connect() is called, the Wi-Fi driver will start the internal scan/connection process.
        4.2 - If the internal scan/connection process is successful, the WIFI_EVENT_STA_CONNECTED will be generated. 
              In the event task, it starts the DHCP client, which will finally trigger the DHCP process.
        4.3 - In the above-mentioned scenario, the application event callback will relay the event to the application 
              task. Generally, the application needs to do nothing, and you can do whatever you want, e.g., print a log.

        In step 4.2, the Wi-Fi connection may fail because, for example, the password is wrong, or the AP is not found. 
        In a case like this, WIFI_EVENT_STA_DISCONNECTED will arise and the reason for such a failure will be provided. 
        For handling events that disrupt Wi-Fi connection, please refer to phase 5.

    5. Wi-Fi Disconnect Phase 

        5.1 - When the Wi-Fi connection is disrupted, e.g., the AP is powered off or the RSSI is poor, 
              WIFI_EVENT_STA_DISCONNECTED will arise. This event may also arise in phase 3. Here, the event task will notify
              the LwIP task to clear/remove all UDP/TCP connections. Then, all application sockets will be in a wrong status.
              In other words, no socket can work properly when this event happens.
        5.2 - In the scenario described above, the application event callback function relays WIFI_EVENT_STA_DISCONNECTED 
              to the application task. The recommended actions are: 1) call esp_wifi_connect() to reconnect the Wi-Fi, 
              2) close all sockets, and 3) re-create them if necessary. For details, please refer to WIFI_EVENT_STA_DISCONNECTED.

    6. Wi-Fi Deinit Phase

        6.1 - Call esp_wifi_disconnect() to disconnect the Wi-Fi connectivity.
        6.2 - Call esp_wifi_stop() to stop the Wi-Fi driver.
        6.3 - Call esp_wifi_deinit() to unload the Wi-Fi driver.

    */
}

    /*
    ESP32 Wi-Fi Station General Scenario

    1. Wi-Fi/LwIP Init Phase

        Step 1.1 ~ 1.5 is a recommended sequence that initializes a Wi-Fi-/LwIP-based application. 
        However, it is NOT a must-follow sequence, which means that you can create the application task 
        in step 1.1 and put all other initialization in the application task. Moreover, you may not want to 
        create the application task in the initialization phase if the application task depends on the sockets. 
        Rather, you can defer the task creation until the IP is obtained.

        1.1 - The main task calls esp_netif_init() to create an LwIP core task and initialize LwIP-related work.
        1.2 - The main task calls esp_event_loop_create() to create a system Event task and initialize an application 
              event's callback function. In the scenario above, the application event's callback function does nothing
              but relaying the event to the application task.
        1.3 - The main task calls esp_netif_create_default_wifi_ap() or esp_netif_create_default_wifi_sta() to create 
              default network interface instance binding station or AP with TCP/IP stack.
        1.4 - The main task calls esp_wifi_init() to create the Wi-Fi driver task and initialize the Wi-Fi driver.
        1.5 - The main task calls OS API to create the application task.

        OR

        1.1 - The main task calls OS API to create the APPLICATION task.
        1.2 - The APPLICATION task calls esp_netif_init() to create an LwIP core task and initialize LwIP-related work.
        1.3 - The APPLICATION task calls esp_event_loop_create() to create a system Event task and initialize THE application 
              event's callback function.
        1.4 - The APPLICATION task calls esp_netif_create_default_wifi_ap() or esp_netif_create_default_wifi_sta() to create 
              default network interface instance binding station or AP with TCP/IP stack.
        1.5 - The APPLICATION task calls esp_wifi_init() to create the Wi-Fi driver task and initialize the Wi-Fi driver.

    2. Wi-Fi Configuration Phase

        Once the Wi-Fi driver is initialized, you can start configuring the Wi-Fi driver. In this scenario, 
        the mode is station, so you may need to call esp_wifi_set_mode() (WIFI_MODE_STA) to configure the Wi-Fi 
        mode as station. You can call other esp_wifi_set_xxx APIs to configure more settings, such as the protocol mode, 
        the country code, and the bandwidth. Refer to ESP32 Wi-Fi Configuration.

        Generally, the Wi-Fi driver should be configured before the Wi-Fi connection is set up. But this is NOT 
        mandatory, which means that you can configure the Wi-Fi connection anytime, provided that the Wi-Fi driver is 
        initialized successfully. However, if the configuration does not need to change after the Wi-Fi connection is 
        set up, you should configure the Wi-Fi driver at this stage, because the configuration APIs 
        (such as esp_wifi_set_protocol()) will cause the Wi-Fi to reconnect, which may not be desirable.

        If the Wi-Fi NVS flash is enabled by menuconfig, all Wi-Fi configuration in this phase, or later phases, 
        will be stored into flash. When the board powers on/reboots, you do not need to configure the Wi-Fi driver 
        from scratch. You only need to call esp_wifi_get_xxx APIs to fetch the configuration stored in flash previously. 
        You can also configure the Wi-Fi driver if the previous configuration is not what you want.

    3. Wi-Fi Start Phase

        3.1 - Call esp_wifi_start() to start the Wi-Fi driver.
        3.2 - The Wi-Fi driver posts WIFI_EVENT_STA_START to the event task; then, the event task will do some common 
              things and will call the application event callback function.
        3.3 - The application event callback function relays the WIFI_EVENT_STA_START to the application task. We 
              recommend that you call esp_wifi_connect(). However, you can also call esp_wifi_connect() in other phrases after 
              the WIFI_EVENT_STA_START arises.

    4. Wi-Fi Connect Phase

        4.1 - Once esp_wifi_connect() is called, the Wi-Fi driver will start the internal scan/connection process.
        4.2 - If the internal scan/connection process is successful, the WIFI_EVENT_STA_CONNECTED will be generated. 
              In the event task, it starts the DHCP client, which will finally trigger the DHCP process.
        4.3 - In the above-mentioned scenario, the application event callback will relay the event to the application 
              task. Generally, the application needs to do nothing, and you can do whatever you want, e.g., print a log.

        In step 4.2, the Wi-Fi connection may fail because, for example, the password is wrong, or the AP is not found. 
        In a case like this, WIFI_EVENT_STA_DISCONNECTED will arise and the reason for such a failure will be provided. 
        For handling events that disrupt Wi-Fi connection, please refer to phase 6.

    5. Wi-Fi 'Got IP' Phase

        5.1 - Once the DHCP client is initialized in step 4.2, the got IP phase will begin.
        5.2 - If the IP address is successfully received from the DHCP server, then IP_EVENT_STA_GOT_IP will arise and 
              the event task will perform common handling.
        5.3 - In the application event callback, IP_EVENT_STA_GOT_IP is relayed to the application task. For LwIP-based 
              applications, this event is very special and means that everything is ready for the application to begin its 
              tasks, e.g., creating the TCP/UDP socket. A very common mistake is to initialize the socket before 
              IP_EVENT_STA_GOT_IP is received. DO NOT start the socket-related work before the IP is received.
        
    6. Wi-Fi Disconnect Phase

        6.1 - When the Wi-Fi connection is disrupted, e.g., the AP is powered off or the RSSI is poor, 
              WIFI_EVENT_STA_DISCONNECTED will arise. This event may also arise in phase 3. Here, the event task will notify
              the LwIP task to clear/remove all UDP/TCP connections. Then, all application sockets will be in a wrong status.
              In other words, no socket can work properly when this event happens.
        6.2 - In the scenario described above, the application event callback function relays WIFI_EVENT_STA_DISCONNECTED 
              to the application task. The recommended actions are: 1) call esp_wifi_connect() to reconnect the Wi-Fi, 
              2) close all sockets, and 3) re-create them if necessary. For details, please refer to WIFI_EVENT_STA_DISCONNECTED.

    7. Wi-Fi IP Change Phase

        7.1 - If the IP address is changed, the IP_EVENT_STA_GOT_IP will arise with "ip_change" set to true.
        7.2 - This event is important to the application. When it occurs, the timing is good for closing all created sockets and recreating them.

    8. Wi-Fi Deinit Phase
    
        8.1 - Call esp_wifi_disconnect() to disconnect the Wi-Fi connectivity.
        8.2 - Call esp_wifi_stop() to stop the Wi-Fi driver.
        8.3 - Call esp_wifi_deinit() to unload the Wi-Fi driver.

    */

    // or esp_netif_create_default_wifi_sta()


/*========================================= END FILE ============================================*/