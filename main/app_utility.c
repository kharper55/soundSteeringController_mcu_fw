/*===================================================================================================
    File: app_utility.c
    Author: Kevin Harper
    Date: 01/2024
    Details: Various function definitions which are used generally and not for any one processor 
             peripheral in particular.

    Written using ESP-IDF v5.1.1 API. Built in 03/2025 using v5.1.2
//==================================================================================================*/

#include "app_include/app_utility.h" // Function prototypes, constants, preprocessor defs/macros, typedefs

const char * device_state_names[2] = {
    "OFF",
    "ON"
};

const char * connection_state_names[4] = { // Applicable for GPIO and BT/WIFI
    "DISCONNECTED",
    "CONNECTING",
    "CONNECTED",
    "DISCONNECTING"
};

const char * process_state_names[3] = {
    "SUSPENDED",
    "STOPPED",
    "RUNNING"
};

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

/*---------------------------------------------------------------
    xxxxxxx
---------------------------------------------------------------*/
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

/*---------------------------------------------------------------
    xxxxxxx
---------------------------------------------------------------*/
uint8_t concat_hex_chars(char high, char low) {
    uint8_t high_val = hex2dec(high);
    uint8_t low_val = hex2dec(low);

    // Combine the high and low nibbles into a single byte
    return (high_val << 4) | low_val;
}


/*---------------------------------------------------------------
    Initialize the circular buffer
---------------------------------------------------------------*/
void init_buffer(circularBuffer *cb, uint16_t size) {
    // this is almost certainly a waste of clock cycles
    /*for (int i = 0; i < size; i++) {
        cb->buffer[i] = 0;
    }*/
    cb->head = 0;
    cb->tail = 0;
    cb->mutex = xSemaphoreCreateMutex();  // Create mutex
    cb->mutexWaitMs = 1; // 1ms minimal wait time
    cb->size = size;
}

/*---------------------------------------------------------------
    Clear circ buff. dont use this
---------------------------------------------------------------*/
void clear_buffer(circularBuffer *cb) {
    xSemaphoreTake(cb->mutex, cb->mutexWaitMs);  // Take mutex before accessing buffer
    for (int i = 0; i < cb->size; i++) {
        cb->buffer[i] = 0;
    }
    cb->head = 0;
    cb->tail = 0;
    xSemaphoreGive(cb->mutex);  // Release mutex after accessing buffer
}

/*---------------------------------------------------------------
    Reset circ buff. For when you dont explicitly need to clear the data contents of the buffer
---------------------------------------------------------------*/
void reset_buffer(circularBuffer *cb) {
    xSemaphoreTake(cb->mutex, cb->mutexWaitMs);  // Take mutex before accessing buffer
    cb->head = 0;
    cb->tail = 0;
    xSemaphoreGive(cb->mutex);  // Release mutex after accessing buffer
}

/*---------------------------------------------------------------
    Push single data member
---------------------------------------------------------------*/
// Push a key into the circular buffer
void push_data(circularBuffer *cb, uint8_t key) {
    xSemaphoreTake(cb->mutex, cb->mutexWaitMs);  // Take mutex before accessing buffer
    cb->buffer[cb->tail] = key;
    cb->tail = (cb->tail + 1) % cb->size;
    if (cb->tail == cb->head) {
        cb->head = (cb->head + 1) % cb->size; // Increment head to drop oldest element
    }
    xSemaphoreGive(cb->mutex);  // Release mutex after accessing buffer
}

/*---------------------------------------------------------------
    Pop single data member
---------------------------------------------------------------*/
int pop_data(circularBuffer *cb) {
    xSemaphoreTake(cb->mutex, cb->mutexWaitMs);  // Take mutex before accessing buffer
    if (cb->head == cb->tail) {
        xSemaphoreGive(cb->mutex);  // Release mutex if buffer is empty
        return -1;
    }
    int data = cb->buffer[cb->head];
    cb->head = (cb->head + 1) % cb->size;
    xSemaphoreGive(cb->mutex);  // Release mutex after accessing buffer
    return data;
}


/*---------------------------------------------------------------
    Push a block of data
---------------------------------------------------------------*/
void push_blk_data(circularBuffer *cb, uint8_t * data, uint8_t size) {
    xSemaphoreTake(cb->mutex, cb->mutexWaitMs);  // Take mutex before accessing buffer
    
    if (cb->tail + size < cb->size) {

        for (int i = cb->tail; i < (cb->tail + cb->size); i++) {
            cb->buffer[i] = *data;
            data+=sizeof(*data);  // Update ptr location
        }

        cb->tail = (cb->tail + size)/*% cb->size*/;     // Modulo handles the wrap

        // only need to update the head if we fill the buffer, wont happen here since if
    }
    else {
        // no

        // fill until the end of the buffer, then begin to overwrite the other data
        for (int i = cb->tail; i < (cb->size); i++) {
            cb->buffer[i] = *data;
            data+=sizeof(*data);  // Update ptr location
        }

        cb->tail = (cb->tail + size) % cb->size;     // Modulo handles the wra

        if (cb->tail == cb->head) {
            cb->head = (cb->head + 1) % cb->size;    // Increment head to drop oldest element
        }
    }
    xSemaphoreGive(cb->mutex);  // Release mutex after accessing buffer
}

/*---------------------------------------------------------------
    Pop a block of data
---------------------------------------------------------------*/
int pop_blk_data(circularBuffer *cb, uint8_t size) {
    xSemaphoreTake(cb->mutex, cb->mutexWaitMs);  // Take mutex before accessing buffer
    if (cb->head == cb->tail) {
        xSemaphoreGive(cb->mutex);  // Release mutex if buffer is empty
        return -1;
    }
    int key = cb->buffer[cb->head];
    cb->head = (cb->head + 1) % cb->size;
    xSemaphoreGive(cb->mutex);  // Release mutex after accessing buffer
    return 0; // Update to return actual size of data returned. this should fetch all possible data from head to tail less one
}

/*========================================= END FILE ============================================*/