/*===================================================================================================
    File: app_uart2.h
    Author: Kevin Harper
    Date: 01/2024
    Details: Function prototypes, constants, preprocessor defs/macros for UART2 peripheral.

    Written using ESP-IDF v5.1.1 API. Built in 03/2025 using v5.1.2
//==================================================================================================*/

#ifndef APP_UART2_H
#define APP_UART2_H

#ifdef __cplusplus
extern "C" {
#endif

/* NOTE 

UART0 setup is taken care of at startup and is used by the log library
This can be changed via menuconfig. UART0 is configured to work with the on board CP2102 USB-UART bridge for debugging and programming.

*/

// Includes 
#include "driver/uart.h" // IDF provided driver
#include "app_gpio.h"    // gpio nums
#include "app_utility.h" // string.h... do we really need...

// Pin Defines
#define U2TXD_PIN               GPIO_NUM_17
#define U2RXD_PIN               GPIO_NUM_16

// Settings
#define U2_BAUD                 115200
#define RX_BUF_SIZE             512

// Macros

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

typedef struct {
    const char * TAG;
    uint8_t * data_buff;
    size_t buff_len;
    bool * flag;            // Set high to inform neighboring tasks to act on new data
    bool * flag2;           // for i2c, no time to explain
    uint16_t val;
    int delay_ms;
} u2rxParams_t;

extern const char * serial_cmd_names[7];

// Static functions

// User functions
void app_uart2_init(int baud);
int sendData(const char * logName, const char * data);


#ifdef __cplusplus
}
#endif

#endif  // APP_UART2_H