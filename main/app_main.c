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
static void rx_task(void *arg) {


    const int MIN_RX_BYTES = 9;
    const int CRC_SIZE = 8;

    const static char *TAG = "U2R";
    esp_log_level_set(TAG, ESP_LOG_INFO);
    uint8_t * data = (uint8_t*) malloc(RX_BUF_SIZE+1);

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
        if (rxBytes && rxBytes >= MIN_RX_BYTES) { // Verify we received at least the minimum number of bytes required

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
static void i2c_task(void *arg) {
    const static char *TAG = "I2C";
    uint8_t dataX[2];
    uint8_t dataY[2];
    esp_err_t ret = ESP_OK;

    // factor this to 1. take params
    //                2. act on the received value from uart2/bluetooth

    ESP_ERROR_CHECK(app_i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully.");
    while(1) {

        ret = ad5272_rdac_reg_read(&dataX);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "read RDAC bits: %d counts (%.2f ohms).", dataX[0] << 8 | dataX[1], AD5272_CNT_TO_OHM(dataX[0], dataX[1]));
        } else {
            ESP_LOGI(TAG, "NACK OR BUS BUSY");
        }

        ret = ad5272_ctrl_reg_read(&dataY);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Control bits: 0x%X", ((dataY[0] << 8) | dataY[1]));
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
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    // Execution should not get here
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}

/*---------------------------------------------------------------
    SPI Master FreeRTOS Task (Full-duplex comms to Artix7)
---------------------------------------------------------------*/
static void spi_task(void *arg) {
    const static char *TAG = "SPI";
    esp_err_t ret = ESP_OK;
    spi_device_handle_t spi;
    app_spi_init(&spi);

    while(1) {
        //ESP_LOGI(TAG, "SPI Writing 0x%x...", (spi_tx_data[3] << 24) | (spi_tx_data[2] << 16) | (spi_tx_data[1] << 8) | (spi_tx_data[0]));
        ret = spi_master_start_transaction(spi, spi_tx_data, spi_rx_data, 4);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Wrote 0x%08X.", (spi_tx_data[0] << 24) | (spi_tx_data[1] << 16) | (spi_tx_data[2] << 8) | (spi_tx_data[3]));  
            ESP_LOGI(TAG, "Read 0x%08X.", (spi_rx_data[3] << 24) | (spi_rx_data[2] << 16) | (spi_rx_data[1] << 8) | (spi_rx_data[0]));     
        } else {
            ESP_LOGI(TAG, "TRANSACTION FAILED OR BUS BUSY");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
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

    adc_oneshot_unit_handle_t adc1_handle = NULL;
    adc_cali_handle_t adc1_cali_chan0_handle = NULL;
    adc_cali_handle_t adc1_cali_chan1_handle = NULL;

    //adc_oneshot_unit_handle_t adc2_handle = NULL; // ADC2 is not configured for use on this set of boards (jumper setting)
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
        .vfilt = &vdrive_filt,
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
        .vfilt = &vntc_filt,
    };

    // ===== Application Task Declarations ===== //

    xTaskCreate(rx_task,  "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES,   NULL);
    xTaskCreate(tx_task,  "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL); // Only invoke this task when explicitly requested by the remote
    xTaskCreate(adc_task, "vdrive_task",  1024*2, (void *)&vdriveParams, configMAX_PRIORITIES-2, NULL);
    xTaskCreate(adc_task, "vntc_task",  1024*2, (void *)&vntcParams, configMAX_PRIORITIES-2, NULL);
    xTaskCreate(spi_task, "spi_task",  1024*2, NULL, configMAX_PRIORITIES-1,    NULL);
    xTaskCreate(i2c_task, "i2c_task",  1024*2, NULL, configMAX_PRIORITIES-1,    NULL);

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

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/*========================================= END PROGRAM ============================================*/