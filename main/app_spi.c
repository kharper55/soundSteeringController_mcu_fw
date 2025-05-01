/*===================================================================================================
    File: app_spi.c
    Author: Kevin Harper
    Date: 01/2024
    Details: Function and global definitions for SPI peripheral FreeRTOS tasks.

    Written using ESP-IDF v5.1.1 API. Built in 03/2025 using v5.1.2
//==================================================================================================*/

#include "app_include/app_spi.h" // Function prototypes, constants, preprocessor defs/macros, typedefs

/*---------------------------------------------------------------
    xxxxxxx
---------------------------------------------------------------*/
esp_err_t app_spi_init(spi_device_handle_t * spi) {

    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .miso_io_num = SPI_MISO_PIN,
        .mosi_io_num = SPI_MOSI_PIN,
        .sclk_io_num = SPI_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    spi_device_interface_config_t devcfg = {
        .command_bits   = 0,
        .address_bits   = 0,
        .dummy_bits     = 0,
        .clock_speed_hz = 12.5*1000*1000,           //Clock out at 12.5 MHz (had a bit shift error at 25MHz, could not figure out the proper timing delay)
        .duty_cycle_pos = 128,
        .mode           = 0,                        /**< SPI mode, representing a pair of (CPOL, CPHA) configuration:
                                                    - 0: (0, 0)
                                                    - 1: (0, 1)
                                                    - 2: (1, 0)
                                                    - 3: (1, 1)
                                                    */
        //.flags = SPI_DEVICE_HALFDUPLEX,
        //.input_delay_ns = 5,
        //.cs_ena_posttrans = 16,
        .spics_io_num   = SPI_CS_PIN,               //CS pin
        .queue_size     = 1//,                          //We want to be able to queue 7 transactions at a time
        //.pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };
    //Initialize the SPI bus
    ret=spi_bus_initialize(APP_SPI_HOST, &buscfg, SPI_DMA_DISABLED);
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(APP_SPI_HOST, &devcfg, spi);
    ESP_ERROR_CHECK(ret);
    
    return ESP_OK;
}

/*---------------------------------------------------------------
        SPI Stuff
---------------------------------------------------------------*/
/* Send data to the LCD. Uses spi_device_polling_transmit, which waits until the
 * transfer is complete.
 *
 * Since data transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
esp_err_t spi_master_start_transaction(spi_device_handle_t spi, uint8_t data_tx[4], uint8_t data_rx[4], int len) {
    esp_err_t ret;
    spi_transaction_t t;
    if (len==0) return ESP_OK;             //no need to send anything
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=len*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=data_tx;               //Data
    t.rx_buffer=data_rx;
    //t.tx_data[3]=data[3];      
    //t.tx_data[2]=data[2]; 
    //t.tx_data[1]=data[1];          //Data
    //t.tx_data[0]=data[0];
    //t.flags=SPI_TRANS_USE_TXDATA;
    //t.user=(void*)1;                //D/C needs to be set to 1
    ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);              //Should have had no issues.
    return ret;
}

/*========================================= END FILE ============================================*/