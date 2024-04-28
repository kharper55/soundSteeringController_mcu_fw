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
#include "driver/uart.h"
#include "app_gpio.h"    // gpio nums
#include "app_utility.h" // string.h

// Pin Defines
#define U2TXD_PIN               GPIO_NUM_17
#define U2RXD_PIN               GPIO_NUM_16

// Settings
#define U2_BAUD                 115200
#define RX_BUF_SIZE             512

// Macros

// Typedefs
typedef enum serial_cmds_t {
    TOGGLE_ON_OFF             = 0x0,  // Hex code for toggling device on/off (i.e. power to the array)
    CHANGE_CHANNEL            = 0x2,  // Hex code for changing only channel with one transaction
    CHANGE_VOLUME             = 0x4,  // Hex code for changing only volume with one transaction
    CHANGE_COORD              = 0x6,  // Hex code for changing only coordinate with one transaction
    CHANGE_CHANNEL_AND_VOLUME = 0x8,  // Hex code for changing both channel and volume with one transaction
    CHANGE_VOLUME_AND_COORD   = 0xA,  // Hex code for changing both volume and coordination with one transaction
    CHANGE_ALL                = 0xC,  // Hex code for changing volume, channel, and coordination with one transaction
    REQUEST_INFO              = 0xE   // Hex code for requesting readback from the device
};

// Static functions

// User functions
void app_uart2_init(int baud);
int sendData(const char * logName, const char * data);


#ifdef __cplusplus
}
#endif

#endif  // APP_UART2_H