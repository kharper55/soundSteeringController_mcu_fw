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

// Static functions

// User functions
void app_uart2_init(int baud);
int sendData(const char * logName, const char * data);


#ifdef __cplusplus
}
#endif

#endif  // APP_UART2_H