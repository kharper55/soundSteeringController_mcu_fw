/*===================================================================================================
    Ultrasonic Drive Controller - Code to implement controller for phased array in order to demonstrate
                                  the phenomena of "sound from ultrasound".

    Created By Kevin Harper, 01-08-2024

    Update 03/10/2025:
        - plan to work on BT + I2S->FPGA i/f drivers
        - simple laptop + phone? interface... no just laptop - network + BT? can we just stream the audio via network as opp to BT... then no TDMA for single antenna
        - FPGA i2s integration to data path.... driver, PCM->PWM counts???...
        - general code clean up and commenting needed. Rearrange prototype and def for functions so main at top

    Written using ESP-IDF v5.1.1 API
    New builds using ESP-IDF v5.1.6 API as of 03/10/2025 - https://docs.espressif.com/projects/esp-idf/en/v5.1.6/esp32/api-guides/index.html
//==================================================================================================*/

// TOOL CHANGE INCLUSIONS 03/10/2025 (see "app_utility")

// Work on I2C ANDDDD BT drivers... open circuit the uart2 pins? boot time config?
// Work on continuous mode ADC driver
// Work on all comms. Especially from remote to controller...
// We need some encoding of the data... data to be sent is from 2 encoders (angles) and 2 pots (volume, other)
// ^ no clue what i meant by this last line 03/10
// Work on error handling

#include "app_include/app_utility.h"    // CRC32 implementation + other header dependents
#include "app_include/app_uart2.h"      // UART comms to remote
#include "app_include/app_adc.h"        // ESP32 ADC peripheral driver interface functions
#include "app_include/app_gpio.h"       // GPIO function handles
#include "app_include/app_spi.h"        // SPI function handles (link to Artix7 FPGA)
#include "app_include/app_i2c.h"        // I2C function handles and interface defintion for digipot
#include "app_include/app_i2s.h"        // Not actually utilized currently
#include "app_include/app_bt.h"         // Not actually utilized currently
#include "app_include/app_timer.h"      // Not actually utilized currently, port from remote source code
#include "app_include/app_wifi.h"       // Not actually utilized currently

//NOTE: NEED TO PORT IN CHANGES TO ADC TASKS
static int vdrive_raw = 0;
static int vdrive_cali = 0;
static int vdrive_filt = 0;

static int vntc_raw = 0;
static int vntc_cali = 0;
static int vntc_filt = 0;

uint8_t spi_tx_data[4] = {0xBA, 0xAD, 0xF0, 0x0D};
uint8_t spi_rx_data[4] = {0x00, 0x00, 0x00, 0x00};

uint32_t count = 0;
bool heartbeat = false; // Level status for heartbeat output. Flopped at set interval in app_main

// Try automatically getting length with sizeof(arr)/sizeof(firstEl)(this requires predeclaring an array and passing to the struct which I am not sure is possible)
adc_filter_t vdrive_filt_handle = {0, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 10, false};
adc_filter_t vntc_filt_handle = {0, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 10, false};

adc_temp_t ntc_temp = TEMP_UNKNOWN;
uint16_t rdacReg = 0;
uint16_t ctrlReg = 0;
uint16_t otpReg = 0;
uint16_t otpAddr = 0;

uint16_t RDAC_VAL = 0;

SemaphoreHandle_t xSemaphore;

AD5272_actions_t digipot_action = HOLD;
uint16_t digipot_value = AD5272_RDAC_MID;

bool artix7_send_flag = false;
bool ad5272_update_flag = false;

extern const char * temperature_names[5];  // Defined in app_adc.h. Might want to capitilize these
extern const char * gpio_status_names[2];  // Defined in app_gpio.h
extern const char * serial_cmd_names[7];   // Defined in app_uart2.h
extern const char * device_state_names[2]; // Defined in app_gpio.h

/*---------------------------------------------------------------
    ADC Oneshot-Mode Driver Continuous Read Task
    
    Generic oneshot adc task for various MCU adc channels
---------------------------------------------------------------*/
static void adc_task(void * pvParameters) {

    const bool VERBOSE_FLAG = true;
    esp_err_t err = ESP_OK;

    // Unique parameters per ADC channel
    adcOneshotParams_t * params = (adcOneshotParams_t *) pvParameters;
    const char * TAG = params->TAG;
    adc_oneshot_unit_handle_t * adc_handle = params->handle;
    adc_cali_handle_t * cali_handle = params->cali_handle;
    adc_unit_t unit = params->unit;
    adc_channel_t chan = params->channel;
    adc_atten_t atten = params->atten;
    int delay_ms = params->delay_ms;
    adc_filter_t * filt = params->filt;
    int * vraw = params->vraw;
    int * vcal = params->vcal;
    int * vfilt = params->vfilt;
    
    adc_oneshot_init(adc_handle, unit, chan); // VPOTC adc16
    adc_calibration_init(unit, chan, atten, cali_handle);

    while (1) {
        
        err = adc_oneshot_read(*adc_handle, chan, vraw);
        if (err != ESP_OK) {
            if(VERBOSE_FLAG) {
                // Handle timeout or other errors
                if (err == ESP_ERR_TIMEOUT) {
                    // Timeout occurred, handle it
                    ESP_LOGW(TAG, "ADC read timeout occurred");
                } else {
                    // Handle other errors
                    ESP_LOGE(TAG, "ADC read failed with error code: %d", err);
                }
            }
        } 
        else {
            err = adc_cali_raw_to_voltage(*cali_handle, *vraw, vcal);
            if (err == ESP_OK) {

                *vfilt = adc_filter(*vcal, filt);
            
                if (VERBOSE_FLAG) {
                    ESP_LOGI(TAG, "ADC%d_%d raw  : %d counts", unit + 1, chan, *vraw);
                    ESP_LOGI(TAG, "ADC%d_%d cal  : %dmV", unit + 1, chan, *vcal);
                    ESP_LOGI(TAG, "ADC%d_%d filt : %dmV", unit + 1, chan, *vfilt);
                }

            }
        }
        vTaskDelay(pdMS_TO_TICKS(delay_ms));    // This delay defines the filter effective time constant
    }
    
    adc_oneshot_del_unit(*adc_handle);
    adc_calibration_deinit(*cali_handle);
}

/*---------------------------------------------------------------
    UART2 TX FreeRTOS task
---------------------------------------------------------------*/
static void tx_task(void *arg) {
    // Work is yet to be done here to complete the communication circle
    // from remote to controller
    const static char *TAG = "U2T";
    esp_log_level_set(TAG, ESP_LOG_INFO);
    while (1) {
        //sendData(TAG, "XXXX");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/*---------------------------------------------------------------
    UART2 RX FreeRTOS task
---------------------------------------------------------------*/
// We are expecting some fixed max byte width (but varies among transaction types depending on leading hex-code).
// We also must pop a known fixed number of crc32 bytes (8) off the end upon reception and compare with a local computation upon the data string
static void rx_task(void * pvParameters) {

    static const bool VERBOSE = false;

    const int INCREMENT_AMOUNT = 16;

    const int MIN_RX_BYTES = 9;
    const int MAX_RX_BYTES = 17;
    const int CRC_SIZE = 8;
    uint8_t * data = (uint8_t*) malloc(RX_BUF_SIZE+1);

    // Parameters for U2Rx
    u2rxParams_t * params = (u2rxParams_t *) pvParameters;
    const char * TAG = params->TAG;
    uint8_t * data2 = params->data_buff;
    size_t data_buff_len = params->buff_len;
    bool * flag = params->flag;
    bool * flag2 = params->flag2;
    uint16_t * rdacVal = params->val;
    int delay_ms = params->delay_ms;

    static bool deviceState = OFF; // upon reset, everything is off
    static int localDigipotVal = AD5272_RDAC_MID;
    static bool deviceAudioChan = SDOA;
    static bool change = false;

    int decimal = 0;
    char hex[2] = {0x00, 0x00};

    /*typedef enum serial_cmds_t {
        NOP                      = 0x0,
        TOGGLE_ON_OFF            = 0x2,  // Hex code for togglining device on/off (i.e. power to the array)
        CHANGE_CHANNEL           = 0x4,  // Hex code for changing only channel with one transaction
        CHANGE_COORD             = 0x8,  // Hex code for changing only coordinate with one transaction
        CHANGE_VOLUME            = 0xA,  // Hex code for changing only volume with one transaction
        CHANGE_COORD_AND_VOLUME  = 0xC,  // Hex code for changing volume, channel, and coordinate with one transaction
        REQUEST_INFO             = 0xE   // Hex code for requesting readback from the device. Not currently implemented
    };*/
    while (1) {

        const int rxBytes = uart_read_bytes(UART_NUM_2, data, RX_BUF_SIZE, 10 / portTICK_PERIOD_MS);
        
        if (rxBytes && (rxBytes >= MIN_RX_BYTES && rxBytes < MAX_RX_BYTES)) { // Verify we received at least the minimum number of bytes required

            uint32_t rxCRC = 0;

            for (int i = 0; i < CRC_SIZE; i++) {
                uint8_t decValue = hex2dec(data[rxBytes - CRC_SIZE + i]);
      
                // Combine hexadecimal value into the CRC
                rxCRC |= (uint32_t)decValue << ((CRC_SIZE - 1 - i) * 4); // Adjusted bit shift index
            }

            // Log received data
            data[rxBytes - CRC_SIZE] = 0; // Null-terminate data string before CRC
            ESP_LOGI(TAG, "Read %d bytes: '%s'", rxBytes - CRC_SIZE, data);

            // Compute local CRC
            uint32_t localCRC = app_compute_crc32_bytes(data, rxBytes - CRC_SIZE);

            // Compare CRCs
            if (localCRC != rxCRC) {
                ESP_LOGI(TAG, "CRC Mismatch!\n\nRX    : %08lX\nLOCAL : %08lX", rxCRC, localCRC);
                // Set a flag that is registered in the txTask that will communicate back to the remote requesting new data
            }
            // CRC check passed
            else {
                if (VERBOSE) ESP_LOGI(TAG, "GOT COMMAND: %d", hex2dec(data[0]));
                // Grab code byte (MSB)
                switch(hex2dec(data[0])) { // 0x2
                    case(TOGGLE_ON_OFF):
                        // should drive gpio accordingly. Optionally tell microblaze to mute the steering peripheral.
                        
                        // tell microblaze to turn off, so that the spectrum LED goes off, option

                        // For now, simply disable PWM_BUFF_EN, DISABLE GPIO_LOAD_SW_EN IN THAT ORDER when ON!
                        switch(deviceState) {
                            case(OFF):

                                if (VERBOSE) ESP_LOGI(TAG, "TURNING DEVICE ON");

                                // Should probably sweep the digipot down as well, or at least sw reset it
                                gpio_set_level(LOAD_SWITCH_EN_PIN, 1);
                                gpio_set_level(PWM_BUFF_EN_PIN, 1);
                                deviceState = ON;
                                break;

                            case(ON):

                                 if (VERBOSE) ESP_LOGI(TAG, "TURNING DEVICE OFF");

                                gpio_set_level(PWM_BUFF_EN_PIN, 0);
                                gpio_set_level(LOAD_SWITCH_EN_PIN, 0);
                                deviceState = OFF;
                                break;

                            default:
                                break;
                        }

                        // ENABLE load switch en and then PWM_BUFF_EN to turn on.
                        // Also can mute the device and retain ADC configuration and running operation if communicate to artix7

                        break;

                    // This function is complete 0x4
                    case(CHANGE_CHANNEL): // Could have like a subcase here, since both change channel and change coord need to signal to
                        // signal to the microblaze that we'd like to change channels
                        data2[0] = CHANGE_CHANNEL << 4; // Shift two digit hex num over so it only occupies one space
                        data2[1] = 0x00; // These bytes represent the elevation angle
                        data2[2] = 0x00;
                        data2[3] = 0x00;
                        *flag = true;
                        // Pass along command code, decode in microblaze

                        if (VERBOSE) ESP_LOGI(TAG, "Changed audio channel from %d to %d!", deviceAudioChan, !deviceAudioChan);

                        deviceAudioChan = !deviceAudioChan;
                        break;

                    case(CHANGE_COORD): // 0x8
                        // signal to the microblaze that we'd like to change coordinates
                        // Stuff the new data into a shared struct type. SPI master task will register the change and communicate the data
                  
                        // may need to double up on transactions 
                        // since our data length is so wide

                        if (VERBOSE) ESP_LOGI(TAG, "CHANGING COORDINATE");
                        
                        /* Changed on 04/29 to have code occupy MSB, this bumps the data back!*/
                        data2[0] = CHANGE_COORD << 4;// Pass along the code
                        data2[1] = concat_hex_chars(data[1], data[2]); // These bytes represent the azimuth angle
                        data2[2] = concat_hex_chars(data[3], data[4]); // These bytes represent the elevation angle
                        data2[3] = 0x00;
                        *flag = true;

                        // Populate the buffer and set a flag so that the spi task recognizes
                        
                        break;

                    case(CHANGE_VOLUME): // 0xA_[BB]BB [] == valid data

                        if (VERBOSE) ESP_LOGI(TAG, "CHANGING VOLUME");

                        // increment or decrement the digipot according to a comparision to local static copy of pertinent var
        
                        //uint16_t value = data[1]; // this is the new desired value to set digipot 0-100
                        // Extract the two hexadecimal characters from the data
                        //char hex[2] = {data[1], data[2]};
                        hex[0] = data[1];
                        hex[1] = data[2];
                        decimal = 0;

                        // Convert each character to decimal
                        for (int i = 0; i < 2; i++) {
                            decimal = (decimal << 4) | hex2dec(hex[i]);
                        }
                        change = (localDigipotVal != decimal) ? true : false;
                        ESP_LOGI(TAG, "OLD: %d, NEW: %d, CONV: %d", localDigipotVal, decimal, PCT_TO_INV_CNT(decimal));
                        
                        
                        //xSemaphoreTake(xSemaphore, portMAX_DELAY);
                        if (xSemaphoreTake(xSemaphore, 10) == pdTRUE) {
                            RDAC_VAL = PCT_TO_INV_CNT(decimal);  // convert the range 0 to 100 to 1023 to 0
                            xSemaphoreGive(xSemaphore);
                        }
                        //xSemaphoreGive(xSemaphore);

                        *flag2 = true;

                        // set flag for i2c, pass the above data to i2c rdac reg
                        
                        break;
                    

                    case(CHANGE_COORD_AND_VOLUME): // 0xA_[BB]BB [] == valid data
                        // increment or decrement the digipot, and also send new coordinates to 

                            // Same stuff as in change coord alone. Gonna omit this for now as it is least used case
                            if (VERBOSE) ESP_LOGI(TAG, "CHANGING COORDINATE");
                        
                            /* Changed on 04/29 to have code occupy MSB, this bumps the data back!*/
                            data2[0] = CHANGE_COORD << 4;// Pass along the code
                            data2[1] = concat_hex_chars(data[1], data[2]); // These bytes represent the azimuth angle
                            data2[2] = concat_hex_chars(data[3], data[4]); // These bytes represent the elevation angle
                            data2[3] = 0x00;

                            if (VERBOSE) ESP_LOGI(TAG, "CHANGING VOLUME");

                            hex[0] = data[1];
                            hex[1] = data[2];

                            // Convert each character to decimal
                            for (int i = 0; i < 2; i++) {
                                decimal = (decimal << 4) | hex2dec(hex[i]);
                            }
                            change = (localDigipotVal != decimal) ? true : false;
                            ESP_LOGI(TAG, "OLD: %d, NEW: %d, CONV: %d", localDigipotVal, decimal, PCT_TO_INV_CNT(decimal));
                            if (xSemaphoreTake(xSemaphore, 10) == pdTRUE) {
                                RDAC_VAL = PCT_TO_INV_CNT(decimal);  // convert the range 0 to 100 to 1023 to 0
                                xSemaphoreGive(xSemaphore);
                            }


                            // set flag and pass value to i2c reg

                            *flag2 = true;
                                          // set i2c flag
                            *flag = true; // set spi flag 

                        break;

                    case(REQUEST_INFO): 
                        // Not yet implemented. 
                        // If a crc fail occurs, set a flag to rerequest new data
                        break;

                    default:
                        break;
                }
            }
        }
    }
    free(data);

}

/*---------------------------------------------------------------
    I2C FreeRTOS task (for AD5272 digipot)
---------------------------------------------------------------*/
static void i2c_task(void * pvParameters) {

    const bool VERBOSE = false;

    const uint8_t BUFF_SIZE = 2; // In bytes
    uint8_t rx_buff[BUFF_SIZE];
    esp_err_t ret = ESP_OK;

    // Params
    i2cMasterParams_t * params = (i2cMasterParams_t *) pvParameters;
    const char * TAG = params->TAG;
    digipot_status_t * status = params->status;

    bool * updateFlag = params->updateFlag;

    uint16_t * otpReg = status->rdacReg;
    uint16_t * rdacReg = status->ctrlReg;
    uint16_t * ctrlReg = status->otpReg;
    uint16_t * otpAddr = status->otpAddr;

    digipot_ctrl_t * ctrl = params->ctrl;
    AD5272_actions_t * action = ctrl->action;
    uint16_t * wiperVal = ctrl->wiperValue; // dereference locally to change value

    uint16_t * val = params->val;

    int delay_ms = params->delay_ms;

    // factor this to 1. take params
    //                2. act on the received value from uart2/bluetooth

    ESP_ERROR_CHECK(app_i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully.");
    ad5272_sw_reset();
    while(1) {

        ret = ad5272_rdac_reg_read(&rx_buff);
        if (VERBOSE) {
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "read RDAC bits: %d counts (%.2f ohms).", rx_buff[0] << 8 | rx_buff[1], AD5272_CNT_TO_OHM(rx_buff[0], rx_buff[1]));
            } else {
                ESP_LOGI(TAG, "NACK OR BUS BUSY");
            }
        }

        ret = ad5272_ctrl_reg_read(&rx_buff);
        if (VERBOSE) { 
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Control bits: 0x%X", ((rx_buff[0] << 8) | rx_buff[1]));
            } else {
                ESP_LOGI(TAG, "NACK OR BUS BUSY");
            }
        }

        if(*updateFlag) {
            ESP_LOGI(TAG, "Unlocking RDAC");
            ret = ad5272_ctrl_reg_write(1 << AD5272_RDAC_WEN);
            if (ret != ESP_OK) {
                ESP_LOGI(TAG, "NACK OR BUS BUSY");
            }
            //xSemaphoreTake(xSemaphore, portMAX_DELAY);
            // need to actually get the value here
            if (xSemaphoreTake(xSemaphore, 10) == pdTRUE) {
                ESP_LOGI(TAG, "Writing 0x%08X...", RDAC_VAL);
                if (RDAC_VAL > 12) {
                    ret = ad5272_rdac_reg_write(RDAC_VAL);
                }
                xSemaphoreGive(xSemaphore);
            }
            if (ret != ESP_OK) {
                ESP_LOGI(TAG, "NACK OR BUS BUSY");
            }
            //xSemaphoreGive(xSemaphore);
            ESP_LOGI(TAG, "Locking RDAC");
            ret = ad5272_ctrl_reg_write(0x0);
            if (ret != ESP_OK) {
                ESP_LOGI(TAG, "NACK OR BUS BUSY");
            }
            *updateFlag = false;
        }


        /*if (count == 3) {
            ESP_LOGI(TAG, "Unlocking RDAC");
            ret = ad5272_ctrl_reg_write(1 << AD5272_RDAC_WEN);
            if (ret != ESP_OK) {
                ESP_LOGI(TAG, "NACK OR BUS BUSY");
            }
            ESP_LOGI(TAG, "Writing 0x%X...", AD5272_RDAC_MID / 16);
            ret = ad5272_rdac_reg_write(AD5272_RDAC_MID / 16);
            if (ret != ESP_OK) {
                ESP_LOGI(TAG, "NACK OR BUS BUSY");
            }
        }*/

        //else if (count == 4) {
            //ESP_LOGI(TAG, "Writing 0x%X...", AD5272_RDAC_MID);
            //ret = ad5272_rdac_reg_write(AD5272_RDAC_MID);
            //if (ret != ESP_OK) {
            //    ESP_LOGI(TAG, "NACK OR BUS BUSY");
            //}
            //ESP_LOGI(TAG, "Writing 0x%X...", AD5272_RDAC_MIN + 10);
            //ret = ad5272_rdac_reg_write(AD5272_RDAC_MIN + 10);
            //if (ret != ESP_OK) {
            //    ESP_LOGI(TAG, "NACK OR BUS BUSY");
            //}
            //for (int i = AD5272_RDAC_MID; i > AD5272_RDAC_MIN; i--) {
            //    ret = ad5272_rdac_reg_write(i);
            //    if (ret != ESP_OK) {
            //        ESP_LOGI(TAG, "NACK OR BUS BUSY");
            //   }
            //}
        //}
        //else if (count == 128) {
        //    //ret = ad5272_rdac_reg_write(20);
        //    ESP_LOGI(TAG, "Locking RDAC");
        //    ret = ad5272_ctrl_reg_write(0x0);
        //    if (ret != ESP_OK) {
        //        ESP_LOGI(TAG, "NACK OR BUS BUSY");
        //    }
        //}
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
    
    // Execution should not get here
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}

/*---------------------------------------------------------------
    SPI Master FreeRTOS Task (Full-duplex comms to Artix7)
---------------------------------------------------------------*/
static void spi_task(void * pvParameters) {

    const bool VERBOSE = true;
    
    spiMasterParams_t * params = (spiMasterParams_t *) pvParameters;
    const char * TAG = params->TAG;
    spi_device_handle_t * handle = params->handle;
    uint8_t * tx_buff = params->tx_buff;
    uint8_t * rx_buff = params->rx_buff;
    size_t buff_size = params->buff_size;
    bool * flag = params->flag; // read here and reset
    int delay_ms = params->delay_ms;

    esp_err_t ret = ESP_OK;

    app_spi_init(handle);

    while(1) {

        // Add a flag here to send ?
        // Need to get data from uart2 rx task into here and vice versa

        if(*flag) {

            ret = spi_master_start_transaction(*handle, tx_buff, rx_buff, (int)buff_size);
            if(VERBOSE) {
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "Wrote 0x%08X.", (tx_buff[0] << 24) | (tx_buff[1] << 16) | (tx_buff[2] << 8) | (tx_buff[3]));  
                    ESP_LOGI(TAG, "Read 0x%08X.", (rx_buff[3] << 24) | (rx_buff[2] << 16) | (rx_buff[1] << 8) | (rx_buff[0]));     
                } else {
                    ESP_LOGI(TAG, "TRANSACTION FAILED OR BUS BUSY");
                }    
            }
            *flag = false;

        }
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

/*---------------------------------------------------------------
    HTTP Webserver Task
---------------------------------------------------------------*/
static void wifi_task(void * pvParameters) {
    /*
    const bool VERBOSE = true;
    
    spiMasterParams_t * params = (spiMasterParams_t *) pvParameters;
    const char * TAG = params->TAG;
    spi_device_handle_t * handle = params->handle;
    uint8_t * tx_buff = params->tx_buff;
    uint8_t * rx_buff = params->rx_buff;
    size_t buff_size = params->buff_size;
    bool * flag = params->flag; // read here and reset
    int delay_ms = params->delay_ms;

    esp_err_t ret = ESP_OK;*/

    wifiParams_t * params = (wifiParams_t *) pvParameters;
    wifi_mode_t mode = params->mode;

    esp_err_t ret = ESP_OK;

    ret = app_wifi_init(mode);

    // Check if app_wifi_init() succeeded
    if (ret != ESP_OK) { 
        // Handle error codes. Refer to error code esp_err.h
        switch(ret) {

            case(ESP_FAIL):
                break;

            // WiFi driver was not installed by esp_wifi_init()
            case(ESP_ERR_WIFI_NOT_INIT):
                break;

            // Some function called inside the API was passed invalid argument
            // User should check if the wifi related config is correc
            case(ESP_ERR_INVALID_ARG):
                break;
                
            // Out of memory when calling a WiFi API function
            case(ESP_ERR_NO_MEM):
                break;

            // WiFi internal error, station or soft-AP control block wrong
            case(ESP_ERR_WIFI_CONN):
                break;

            // WiFi driver was not started by esp_wifi_start()
            case(ESP_ERR_WIFI_NOT_STARTED):
                break;

            // WiFi mode error
            case(ESP_ERR_WIFI_MODE):
                break;

            // SSID is invalid
            case(ESP_ERR_WIFI_SSID):
                break;
            
            // Non-descript error handler
            default:
                break;
        }
    }
        
    // This needs to be one big event handler... Disconnects, actions, etc.
    while(1) {

        // Add a flag here to send ?
        // Need to get data from uart2 rx task into here and vice versa

        /*
        if(*flag) {

            ret = spi_master_start_transaction(*handle, tx_buff, rx_buff, (int)buff_size);
            if(VERBOSE) {
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "Wrote 0x%08X.", (tx_buff[0] << 24) | (tx_buff[1] << 16) | (tx_buff[2] << 8) | (tx_buff[3]));  
                    ESP_LOGI(TAG, "Read 0x%08X.", (rx_buff[3] << 24) | (rx_buff[2] << 16) | (rx_buff[1] << 8) | (rx_buff[0]));     
                } else {
                    ESP_LOGI(TAG, "TRANSACTION FAILED OR BUS BUSY");
                }    
            }
            *flag = false;

        }
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        */
    }
}

/*============================================ APP_MAIN ============================================*/

void app_main(void) {

    static const bool VERBOSE = false;
    
    const static char *TAG = "APP";
    esp_err_t ret;

    app_uart2_init(U2_BAUD);
    ESP_LOGI(TAG, "UART2 initialized successfully.");
    app_gpio_init();
    app_i2s_init();

    u2rxParams_t u2rxParams = {
        .TAG = "U2R",
        .data_buff = &spi_tx_data, // Data comes from U2Rx, and goes to SPI master task, then sent to artix7
        .buff_len = 4,
        .flag = &artix7_send_flag,
        .flag2 = &ad5272_update_flag,
        //.val = &rdacVal,
        .delay_ms = 10
    };

    adc_oneshot_unit_handle_t adc1_handle = NULL;
    adc_cali_handle_t adc1_cali_chan0_handle = NULL;
    adc_cali_handle_t adc1_cali_chan1_handle = NULL;

    spi_device_handle_t spi;

    //adc_oneshot_unit_handle_t adc2_handle = NULL; // ADC2 is not currently configured for use on this set of boards (jumper setting)
    //adc_cali_handle_t adc2_cali_handle = NULL;

    adcOneshotParams_t vdriveParams = {
        .TAG = "VDRIVE",
        .handle = &adc1_handle,
        .cali_handle = &adc1_cali_chan0_handle,
        .unit = ADC_UNIT_1,
        .channel = APP_ADC1_CHAN0,
        .atten = APP_ADC_ATTEN,
        .delay_ms = 1000,   // This results in a VERYYY averaged adc reading
        .filt = &vdrive_filt_handle,
        .vraw = &vdrive_raw,
        .vcal = &vdrive_cali,
        .vfilt = &vdrive_filt
    };

    adcOneshotParams_t vntcParams = {
        .TAG = "VNTC",
        .handle = &adc1_handle,
        .cali_handle = &adc1_cali_chan1_handle,
        .unit = ADC_UNIT_1,
        .channel = APP_ADC1_CHAN1,
        .atten = APP_ADC_ATTEN,
        .delay_ms = 1000,
        .filt = &vntc_filt_handle,
        .vraw = &vntc_raw,
        .vcal = &vntc_cali,
        .vfilt = &vntc_filt
    };

    spiMasterParams_t mspiParams = {
        .TAG = "MSPI",
        .handle = &spi,
        .tx_buff = &spi_tx_data, // We're fixing at 4 byte transactions
        .rx_buff = &spi_rx_data,
        .buff_size = 4,
        .flag = &artix7_send_flag,
        .delay_ms = 10
    };

    // This struct is passed to the I2C params into the I2C task and the members' memory locations are updated from there for readout elsewhere
    digipot_status_t digipotStatus = {
        .rdacReg = &rdacReg,
        .ctrlReg = &ctrlReg,
        .otpReg = &otpReg,
        .otpAddr = &otpAddr
    };

    digipot_ctrl_t digipotCtrl = {
        .action = &digipot_action,  // This action is to be update by other tasks
        .wiperValue = &digipot_value,   // This value is only used for updating the RDAC register
    };

    i2cMasterParams_t mi2cParams = {
        .TAG = "MI2C",
        .status = &digipotStatus,
        .ctrl = &digipotCtrl,
        .updateFlag = &ad5272_update_flag,
        //.val = &rdacVal,
        .delay_ms = 10
    };

    wifiParams_t wifiParams = {
        .TAG = "WIFI",
        .mode = WIFI_MODE_STA
    };
    
    // ===== Application Task Declarations ===== //

    // add dedicated gpio task?

    xSemaphore = xSemaphoreCreateMutex(); // Not sure if this was meant to stay.
 
    // Low speed peripheral functions (APP_CPU)
    xTaskCreatePinnedToCore(rx_task,  "uart_rx_task", 1024*8, (void *)&u2rxParams, configMAX_PRIORITIES, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(tx_task,  "uart_tx_task", 1024*4, NULL /*TBD: (void *)&u2txParams*/, configMAX_PRIORITIES-1, NULL, APP_CPU_NUM); // Only invoke this task when explicitly requested by the remote
    xTaskCreatePinnedToCore(adc_task, "vdrive_task",  1024*2, (void *)&vdriveParams, configMAX_PRIORITIES-2, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(adc_task, "vntc_task",  1024*2, (void *)&vntcParams, configMAX_PRIORITIES-2, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(spi_task, "spi_task",  1024*2, (void *)&mspiParams, configMAX_PRIORITIES-1, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(i2c_task, "i2c_task",  1024*4, (void *)&mi2cParams, configMAX_PRIORITIES-1, NULL, APP_CPU_NUM);
    
    // RF (webserver via IEE802.11), (PRO_CPU)
    xTaskCreatePinnedToCore(wifi_task, "wifi_task",  1024*8, (void *)&wifiParams, configMAX_PRIORITIES-1, NULL, PRO_CPU_NUM);

    while(1) {

        ESP_LOGI("", "");
        ESP_LOGI(TAG, "Count: %ld", count);

        if (count == 2) { 
            // Drive this pin low if MCU needs to shut down the array for whatever reason
            // ESP32 should relay back to remote if requested channel is not enabled, and not allow switching of the channel
            gpio_set_level(PWM_BUFF_EN_PIN, 0); // These are driven in the u2rx task according to input commands received
            gpio_set_level(LOAD_SWITCH_EN_PIN, 0);

            if (VERBOSE) {
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "MCU PWM enable has been set. FPGA must follow suit.");
                }
                else {
                    ESP_LOGI(TAG, "Issue driving GPIO %d. MCU PWM enable has not been set.", PWM_BUFF_EN_PIN);
                }
            }
        } 
        
        get_drive_temp(&ntc_temp, vntc_filt);

        if (VERBOSE) {
            ESP_LOGI(TAG, "TEMP: %s", temperature_names[ntc_temp]);
            ESP_LOGI(TAG, "AUX: %s", gpio_status_names[gpio_get_level(AUX_SW_PIN)]); // AD4680 SDOA channel analog signal input
            ESP_LOGI(TAG, "ECM: %s", gpio_status_names[gpio_get_level(ECM_SW_PIN)]);    
        }

        count++;
        app_heartbeat_toggle();

        //vTaskDelay(pdMS_TO_TICKS(1000));
        vTaskDelay(HEARTBEAT_BLINK_PERIOD_MS / portTICK_PERIOD_MS);
    }
}

/*========================================= END PROGRAM ============================================*/