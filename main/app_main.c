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

#include "app_include/app_utility.h"
#include "app_include/app_uart2.h"
#include "app_include/app_adc.h"
#include "app_include/app_gpio.h"
#include "app_include/app_spi.h"
#include "app_include/app_i2c.h"
#include "app_include/app_i2s.h"   
#include "app_include/app_bluetooth.h" 

static int vdrive_raw = 0;
static int vdrive_cali = 0;
static int vdrive_avg = 0;

static int ntc_raw = 0;
static int ntc_cali = 0;
static int ntc_avg = 0;

uint8_t spi_tx_data[4] = {0xBA, 0xAD, 0xF0, 0x0D};
uint8_t spi_rx_data[4] = {0x00, 0x00, 0x00, 0x00};

uint32_t count = 0;
bool heartbeat = false; 

adc_filter_t vdrive_filt = {0, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 10, false};
adc_filter_t ntc_filt = {0, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 10, false};
// Try automatically getting length with sizeof(arr)/sizeof(firstEl)(this requires predeclaring an array and passing to the struct which I am not sure is possible)

/* Task to read all three pertinent ADC channels using oneshot modes */
// NOTE! Only ADC1 driver is used on this board. The remote is optionally configurable to use both. 
static void adc_task(void) {

    const static char *TAG = "ADC";
    //const bool VERBOSE = false;

    //-------------ADC1 Init---------------//
    adc_oneshot_unit_handle_t adc1_handle;
    adc_cali_handle_t adc1_cali_chan0_handle = NULL;
    adc_cali_handle_t adc1_cali_chan1_handle = NULL;

    ESP_ERROR_CHECK(adc_oneshot_init(&adc1_handle, ADC_UNIT_1, APP_ADC1_CHAN0));
    ESP_ERROR_CHECK(adc_calibration_init(ADC_UNIT_1, APP_ADC1_CHAN0, APP_ADC_ATTEN, &adc1_cali_chan0_handle));
    adc_oneshot_init(&adc1_handle, ADC_UNIT_1, APP_ADC1_CHAN1); // This will throw an error on 2nd pass of trying to config the adc1 driver 
    ESP_ERROR_CHECK(adc_calibration_init(ADC_UNIT_1, APP_ADC1_CHAN1, APP_ADC_ATTEN, &adc1_cali_chan1_handle));

    while (1) {

        // VSW Monitor (confirmed working)
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, APP_ADC1_CHAN0, &vdrive_raw));
        //ESP_LOGI(TAG, "ADC%d Channel[%d] Raw: %d", ADC_UNIT_1 + 1, APP_ADC1_CHAN0, vdrive_raw);
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, vdrive_raw, &vdrive_cali));
        //ESP_LOGI(TAG, "ADC%d Channel[%d] Cali: %d mV", ADC_UNIT_1 + 1, APP_ADC1_CHAN0, vdrive_cali);
        // Call ADC filter function
        vdrive_avg = adc_filter(vdrive_cali, &vdrive_filt);
        ESP_LOGI(TAG, "ADC%d Channel[%d] Filt: %d mV", ADC_UNIT_1 + 1, APP_ADC1_CHAN0, vdrive_avg);

        // NTC Monitor (confirmed working)
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, APP_ADC1_CHAN1, &ntc_raw));
        //ESP_LOGI(TAG, "ADC%d Channel[%d] Raw: %d", ADC_UNIT_1 + 1, APP_ADC1_CHAN1, ntc_raw);
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan1_handle, ntc_raw, &ntc_cali));
        //ESP_LOGI(TAG, "ADC%d Channel[%d] Cali: %d mV", ADC_UNIT_1 + 1, APP_ADC1_CHAN1, ntc_cali);
        // Call ADC filter
        ntc_avg = adc_filter(ntc_cali, &ntc_filt);
        ESP_LOGI(TAG, "ADC%d Channel[%d] Filt: %d mV", ADC_UNIT_1 + 1, APP_ADC1_CHAN1, ntc_avg);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    //Tear Down
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    //if (do_calibration1_chan0) {
    adc_calibration_deinit(adc1_cali_chan0_handle);
    //}
}

/*---------------------------------------------------------------
        UART Stuff
---------------------------------------------------------------*/

/* UART TX Task */
static void tx_task(void *arg) {
    const static char *TAG = "U2T";
    esp_log_level_set(TAG, ESP_LOG_INFO);
    while (1) {
        sendData(TAG, "XXXX");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* UART RX Task */
static void rx_task(void *arg) {
    const static char *TAG = "U2R";
    esp_log_level_set(TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_2, data, RX_BUF_SIZE, 100 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(TAG, "Read %d bytes: '%s'", rxBytes, data);
        }
    }
    free(data);
}

static void i2c_task(void *arg) {
    const static char *TAG = "I2C";
    uint8_t dataX[2];
    uint8_t dataY[2];
    esp_err_t ret = ESP_OK;

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
            ESP_LOGI(TAG, "Writing 0x%X...", AD5272_RDAC_MID);
            ret = ad5272_rdac_reg_write(AD5272_RDAC_MID);
            if (ret != ESP_OK) {
                ESP_LOGI(TAG, "NACK OR BUS BUSY");
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
}

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

    xTaskCreate(rx_task,  "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES,   NULL);
    xTaskCreate(tx_task,  "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL); // Only invoke this task when explicitly requested by the remote
    xTaskCreate(adc_task, "adc_rd_task",  1024*2, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(spi_task, "spi_task",  1024*2, NULL, configMAX_PRIORITIES-1,    NULL);
    xTaskCreate(i2c_task, "i2c_task",  1024*2, NULL, configMAX_PRIORITIES-1,    NULL);

    while(1) {

        ESP_LOGI("", "");
        ESP_LOGI(TAG, "Count: %ld", count);

        if (count == 2) { 
            ret = gpio_set_level(PWM_BUFF_EN_PIN, 1);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "MCU PWM enable has been set. FPGA must follow suit.");
            }
            else {
                ESP_LOGI(TAG, "Issue driving GPIO %d. MCU PWM enable has not been set.", PWM_BUFF_EN_PIN);
            }
        } 

        /*else if (count == 97) {
            gpio_set_level(PWM_BUFF_EN_PIN, 0);
            gpio_set_level(HEARTBEAT_LED_PIN, 1);
        }*/

        count++;
        ESP_ERROR_CHECK(app_heartbeat_toggle());

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Execution should not get here
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}

/*========================================= END PROGRAM ============================================*/