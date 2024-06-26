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

uint8_t hex2dec(char hex) {
    if (hex >= '0' && hex <= '9') {
        return hex - '0';
    } else if (hex >= 'A' && hex <= 'F') {
        return hex - 'A' + 10;
    } else if (hex >= 'a' && hex <= 'f') {
        return hex - 'a' + 10;
    }
    // Handle invalid hexadecimal characters
    return 0;
}

uint8_t concat_hex_chars(char high, char low) {
    uint8_t high_val = hex2dec(high);
    uint8_t low_val = hex2dec(low);

    // Combine the high and low nibbles into a single byte
    return (high_val << 4) | low_val;
}