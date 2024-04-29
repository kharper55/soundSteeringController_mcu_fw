/*===================================================================================================
    Ultrasonic Drive Controller - Code to implement controller for phased array in order to demonstrate
                                  the phenomena of "sound from ultrasound".
    Created By Kevin Harper, 01-08-2024 
    
    Written using ESP-IDF v5.1.1 API
//==================================================================================================*/

// Work on I2C driver
// Work on continuous mode ADC driver
// Work on all comms. Especially from remote to controller...
// We need some encoding of the data... data to be sent is from 2 encoders (angles) and 2 pots (volume, other)

#include "app_include/app_utility.h"    // CRC32 implementation + other headers
#include "app_include/app_uart2.h"
#include "app_include/app_adc.h"
#include "app_include/app_gpio.h"
#include "app_include/app_spi.h"
#include "app_include/app_i2c.h"
#include "app_include/app_i2s.h"   
#include "app_include/app_bluetooth.h" 

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

AD5272_actions_t digipot_action = HOLD;
uint16_t digipot_value = AD5272_RDAC_MID;

bool artix7_send_flag = false;

extern const char * temperature_names[5];
extern const char * gpio_status_names[2];
extern const char * serial_cmd_names[7];

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
    // Work is yet to be done here to complete the communication circle from remote to controller
    const static char *TAG = "U2T";
    esp_log_level_set(TAG, ESP_LOG_INFO);
    while (1) {
        sendData(TAG, "XXXX");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/*---------------------------------------------------------------
    UART2 RX FreeRTOS task
---------------------------------------------------------------*/
// We are expecting some fixed max byte width (but varies among transaction types depending on leading hex-code).
// We also must pop a known fixed number of crc32 bytes (8) off the end upon reception and compare with a local computation upon the data string
static void rx_task(void * pvParameters) {


    const int MIN_RX_BYTES = 9;
    const int MAX_RX_BYTES = 17;
    const int CRC_SIZE = 8;
    uint8_t * data = (uint8_t*) malloc(RX_BUF_SIZE+1);

    //const static char *TAG = "U2R";
    //esp_log_level_set(TAG, ESP_LOG_INFO);

    // Parameters for U2Rx
    u2rxParams_t * params = (u2rxParams_t *) pvParameters;
    const char * TAG = params->TAG;
    uint8_t * data2 = params->data_buff;
    size_t data_buff_len = params->buff_len;
    bool * flag = params->flag;
    int delay_ms = params->delay_ms;

    /*
    // Typedefs
    typedef enum serial_cmds_t {
        NOP                      = 0x0,
        TOGGLE_ON_OFF            = 0x2,  // Hex code for togglining device on/off (i.e. power to the array)
        CHANGE_CHANNEL           = 0x4,  // Hex code for changing only channel with one transaction
        CHANGE_COORD             = 0x8,  // Hex code for changing only coordinate with one transaction
        CHANGE_VOLUME            = 0xA,  // Hex code for changing only volume with one transaction
        CHANGE_COORD_AND_VOLUME  = 0xC,  // Hex code for changing volume, channel, and coordinate with one transaction
        REQUEST_INFO             = 0xE   // Hex code for requesting readback from the device
    };
    */

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
                // Grab code byte (MSB)
                switch(hex2dec(data[0])) {
                    case(TOGGLE_ON_OFF):
                        // should drive gpio accordingly. Optionally tell microblaze to mute the steering peripheral.
                        break;
                    case(CHANGE_CHANNEL): // Could have like a subcase here, since both change channel and change coord need to signal to
                        // signal to the microblaze that we'd like to change channels
                        break;
                    case(CHANGE_COORD):
                        // signal to the microblaze that we'd like to change coordinates
                        // Stuff the new data into a shared struct type. SPI master task will register the change and communicate the data
                  
                        // may need to double up on transactions 
                        // since our data length is so wide

                        data2[0] = concat_hex_chars(data[1], data[2]); // These bytes represent the azimuth angle
                        data2[1] = concat_hex_chars(data[3], data[4]); // These bytes represent the elevation angle
                        data2[3] = 0x00;
                        data2[4] = 0x00;
                        *flag = true;

                        // Populate the buffer and set a flag so that the spi task recognizes
                        
                        break;
                    case(CHANGE_VOLUME):
                        // increment or decrement the digipot according to a comparision to local static copy of pertinent var
                        break;
                    case(CHANGE_COORD_AND_VOLUME):
                        // increment or decrement the digipot, and also send new coordinates to MATLAB
                        break;
                    default:
                        break;
                }
                ESP_LOGI(TAG, "GOT COMMAND: %d", hex2dec(data[0]));
            }
        }
    }
    free(data);

}

/*---------------------------------------------------------------
    I2C FreeRTOS task (for AD5272 digipot)
---------------------------------------------------------------*/
static void i2c_task(void * pvParameters) {

    const uint8_t BUFF_SIZE = 2; // In bytes
    uint8_t rx_buff[BUFF_SIZE];
    esp_err_t ret = ESP_OK;

    // Params
    i2cMasterParams_t * params = (i2cMasterParams_t *) pvParameters;
    const char * TAG = params->TAG;
    digipot_status_t * status = params->status;

    uint16_t * otpReg = status->rdacReg;
    uint16_t * rdacReg = status->ctrlReg;
    uint16_t * ctrlReg = status->otpReg;
    uint16_t * otpAddr = status->otpAddr;

    digipot_ctrl_t * ctrl = params->ctrl;
    AD5272_actions_t * action = ctrl->action;
    uint16_t * wiperVal = ctrl->wiperValue; // dereference locally to change value

    int delay_ms = params->delay_ms;

    // factor this to 1. take params
    //                2. act on the received value from uart2/bluetooth

    ESP_ERROR_CHECK(app_i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully.");
    while(1) {

        ret = ad5272_rdac_reg_read(&rx_buff);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "read RDAC bits: %d counts (%.2f ohms).", rx_buff[0] << 8 | rx_buff[1], AD5272_CNT_TO_OHM(rx_buff[0], rx_buff[1]));
        } else {
            ESP_LOGI(TAG, "NACK OR BUS BUSY");
        }

        ret = ad5272_ctrl_reg_read(&rx_buff);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Control bits: 0x%X", ((rx_buff[0] << 8) | rx_buff[1]));
        } else {
            ESP_LOGI(TAG, "NACK OR BUS BUSY");
        }

        if (count == 3) {
            ESP_LOGI(TAG, "Unlocking RDAC");
            ret = ad5272_ctrl_reg_write(1 << AD5272_RDAC_WEN);
            if (ret != ESP_OK) {
                ESP_LOGI(TAG, "NACK OR BUS BUSY");
            }
            ESP_LOGI(TAG, "Writing 0x%X...", AD5272_RDAC_MAX);
            ret = ad5272_rdac_reg_write(AD5272_RDAC_MAX);
            if (ret != ESP_OK) {
                ESP_LOGI(TAG, "NACK OR BUS BUSY");
            }
        }
        else if (count == 4) {
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
            for (int i = AD5272_RDAC_MID; i > AD5272_RDAC_MIN + 20; i--) {
                ret = ad5272_rdac_reg_write(i);
                if (ret != ESP_OK) {
                    ESP_LOGI(TAG, "NACK OR BUS BUSY");
                }
            }
        }
        else if (count == 5) {
            ESP_LOGI(TAG, "Locking RDAC");
            ret = ad5272_ctrl_reg_write(0x0);
            if (ret != ESP_OK) {
                ESP_LOGI(TAG, "NACK OR BUS BUSY");
            }
        }
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

/*============================================ APP_MAIN ============================================*/

void app_main(void) {

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
        .delay_ms = 0
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
        .delay_ms = 1000
    };
    
    // ===== Application Task Declarations ===== //

    // add dedicated gpio task?

    xTaskCreate(rx_task,  "uart_rx_task", 1024*2, (void *)&u2rxParams, configMAX_PRIORITIES, NULL);
    xTaskCreate(tx_task,  "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL); // Only invoke this task when explicitly requested by the remote
    xTaskCreate(adc_task, "vdrive_task",  1024*2, (void *)&vdriveParams, configMAX_PRIORITIES-2, NULL);
    xTaskCreate(adc_task, "vntc_task",  1024*2, (void *)&vntcParams, configMAX_PRIORITIES-2, NULL);
    xTaskCreate(spi_task, "spi_task",  1024*2, (void *)&mspiParams, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(i2c_task, "i2c_task",  1024*2, (void *)&mi2cParams, configMAX_PRIORITIES-1, NULL);

    while(1) {

        ESP_LOGI("", "");
        ESP_LOGI(TAG, "Count: %ld", count);

        if (count == 2) { 
            // Drive this pin low if MCU needs to shut down the array for whatever reason
            // ESP32 should relay back to remote if requested channel is not enabled, and not allow switching of the channel
            ret = gpio_set_level(PWM_BUFF_EN_PIN, 1);
            gpio_set_level(LOAD_SWITCH_EN_PIN, 1);

            // ESP32 on the controller is responsible for ensuring that in no circumstances
            // does PWM output begin with LOAD_SWITCH_EN driven low. This is accomplished
            // by holding PWM_BUFF_EN low until an appropriate condition is met. The FPGA
            // has a similar control signal which is combined with this MCU signal; they must
            // BOTH be high for PWM output to begin. IF PWM OUTPUT BEGINS WITH LOAD_SWITCH_EN
            // DRIVEN LOW (i.e DISABLED/OFF!), THEN THE PWM WILL BACK BIAS THE CLAMP DIODES
            // IN THE MIC4127 GATE DRIVERS, CAUSING A FAINT INCESSANT 40KHZ TONE.

            // SEQUENCE FOR OFF:
            // PWM_BUFF_EN    <-- LOW
            // LOAD_SWITCH_EN <-- LOW

            // SEQUENCE FOR ON:
            // LOAD_SWITCH_EN <-- HIGH
            // PWM_BUFF_EN    <-- HIGH

            // In simple terms:
            // When PWM is happening, LOAD_SWITCH_EN must be HIGH
            // For PWM to actually reach the output, PWM_BUFF_EN must be HIGH, and the FPGA must agree
            // But in order for the ESP32 to drive PWM_BUFF_EN high, one of either of SDOA/SDOB jacks must be (HIGH?)
            //      and in addition, user must signal an on/off toggle request via the remote

            // PWM_BUFF_EN should only be driven high given that load switch enable is enabled,
            // and that either of the two channels is plugged in (sdoa/sdob jack switch),
            // and that user requests the device to be enabled.
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "MCU PWM enable has been set. FPGA must follow suit.");
            }
            else {
                ESP_LOGI(TAG, "Issue driving GPIO %d. MCU PWM enable has not been set.", PWM_BUFF_EN_PIN);
            }
        } 
        
        get_drive_temp(&ntc_temp, vntc_filt);
        ESP_LOGI(TAG, "TEMP: %s", temperature_names[ntc_temp]);
        ESP_LOGI(TAG, "AUX: %s", gpio_status_names[gpio_get_level(AUX_SW_PIN)]); // AD4680 SDOA channel analog signal input
        ESP_LOGI(TAG, "ECM: %s", gpio_status_names[gpio_get_level(ECM_SW_PIN)]);

        count++;
        ESP_ERROR_CHECK(app_heartbeat_toggle());

        //vTaskDelay(pdMS_TO_TICKS(1000));
        vTaskDelay(HEARTBEAT_BLINK_PERIOD_MS / portTICK_PERIOD_MS);
    }
}

/*========================================= END PROGRAM ============================================*/