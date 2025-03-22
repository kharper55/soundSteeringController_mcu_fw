/*===================================================================================================
    File: app_uart2.c
    Author: Kevin Harper
    Date: 01/2024
    Details: Function and global definitions for UART2 peripheral FreeRTOS tasks.

    Written using ESP-IDF v5.1.1 API. Built in 03/2025 using v5.1.2
//==================================================================================================*/

#include "app_include/app_uart2.h" // Function prototypes, constants, preprocessor defs/macros, typedefs

const char * serial_cmd_names[7] = {
   "NOP",
   "TOGGLE ON/OFF",
   "CHANGE CHANNEL",
   "CHANGE COORDINATE",
   "CHANGE VOLUME",
   "CHANGE COORDINATE & VOLUME"
   "REQUEST INFO"
};

/* UART initialization function */
void app_uart2_init(int baud) {
    const uart_config_t uart_config = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, U2TXD_PIN, U2RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

/* UART data sending function */
int sendData(const char* logName, const char* data) {
    const bool VERBOSE = true;
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_2, data, len);
    ESP_LOGI(logName, "Wrote %d bytes: '%s'", txBytes, data);
    return txBytes;
}

/*========================================= END FILE ============================================*/