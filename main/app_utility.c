#include "app_include/app_utility.h"

/*---------------------------------------------------------------
    Basic CRC32 Implementation (Little Endian)
---------------------------------------------------------------*/
uint32_t app_compute_crc32_str(char * str, int data_len) {
    return crc32_le(0, (const uint8_t *)str, strlen(str));
}

/*---------------------------------------------------------------
    Basic CRC32 Implementation (Little Endian)
---------------------------------------------------------------*/
uint32_t app_compute_crc32_bytes(uint8_t * bytes, int data_len) {
    return crc32_le(0, (const uint8_t *)bytes, data_len);
}

uint8_t hex2dec(uint8_t data) {
    // Convert hexadecimal character to numerical value
    uint8_t decValue = 0;

    if (data >= '0' && data <= '9') {
        decValue = data - '0';
    } else if (data >= 'A' && data <= 'F') {
        decValue = data - 'A' + 10;
    } else {
        // Handle invalid hexadecimal character
        // You might want to add error handling here
    }
    return decValue;
}
