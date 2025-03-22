/*===================================================================================================
    File: app_spi.h
    Author: Kevin Harper
    Date: 01/2024
    Details: Function prototypes, constants, preprocessor defs/macros for SPI peripheral

    Written using ESP-IDF v5.1.1 API. Built in 03/2025 using v5.1.2
//==================================================================================================*/

#ifndef APP_SPI_H
#define APP_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

// Includes
#include "driver/spi_master.h" // IDF provided driver interface
#include "esp_heap_caps.h"     // Heap capacity limit definition per architecture/device
#include "app_gpio.h"          // Function prototypes, constants, preprocessor defs/macros, typedefs
#include "app_utility.h"       // Various function prototypes, constants, preprocessor defs/macros, typedefs (memset())

// Pin Defines (SPI PORT "HSPI" TO ARTIX-7)
#define SPI_CS_PIN              GPIO_NUM_15 // HSPICS0
#define SPI_SCK_PIN             GPIO_NUM_14 // HSPICLK

// THESE PINS HAVE BEEN SWAPPED TO PREVENT FROM MISO BECOMING LOCKED HIGH IN DEVELOPMENT, PREVENTING CODE FROM BEING ABLE TO BE UPLOADED
#define SPI_MISO_PIN            GPIO_NUM_13 // HSPIQ
#define SPI_MOSI_PIN            GPIO_NUM_12 // HSPID

// Settings
//To speed up transfers, every SPI transfer sends a bunch of lines. This define specifies how many. More means more memory use,
//but less overhead for setting up / finishing transfers. Make sure 240 is dividable by this.
#define PARALLEL_LINES          16
#define APP_SPI_HOST            HSPI_HOST

// Macros

// Typedefs
// NOTE: Other driver-specific configuration info is contained within the app_init function
typedef struct {
    char * TAG;
    spi_device_handle_t * handle;
    uint8_t * tx_buff; // We're fixing at 4 byte transactions
    uint8_t * rx_buff; 
    size_t buff_size;
    bool * flag;
    int delay_ms;
} spiMasterParams_t;

// Static functions

// User functions
esp_err_t app_spi_init(spi_device_handle_t * spi);
esp_err_t spi_master_start_transaction(spi_device_handle_t spi, uint8_t data_tx[4], uint8_t data_rx[4], int len);

#ifdef __cplusplus
}
#endif

#endif  // APP_I2C_H