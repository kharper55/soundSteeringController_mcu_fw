/*===================================================================================================
    File: app_utility.h
    Author: Kevin Harper
    Date: 01/2024
    Details: Various function prototypes, constants, preprocessor defs/macros which are used
             generally and not for any one processor peripheral in particular.

    Written using ESP-IDF v5.1.1 API. Built in 03/2025 using v5.1.2
//==================================================================================================*/

#ifndef APP_UTILITY_H
#define APP_UTILITY_H

#ifdef __cplusplus
extern "C" {
#endif

// Includes 
#include <stdio.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_mac.h" // TOOL CHANGE INCLUSION 03/10/2025 for v5.1.1->v5.1.6
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "sdkconfig.h"
#include <string.h>
#include <ctype.h>
#include "rom/crc.h"            // For crc32 calculations
#include "freertos/ringbuf.h"

//#include "freertos/queue.h"
#include "freertos/semphr.h"

// Pin Defines

// Settings
#define INIT_DELAY_PD_MS 500 // Wait this long in the main task after starting all other tasks
#define DEFAULT_TASK_DELAY_PD_MS 10

// Macros

// Typedefs
extern const char * device_state_names[2];
extern const char * connection_state_names[4];
extern const char * process_state_names[3];

typedef struct {
    uint8_t * buffer;         // uint8_ts
    uint16_t head;            // Index of the first element in the buffer
    uint16_t tail;            // Index of the next empty slot in the buffer
    SemaphoreHandle_t mutex;  // Mutex to protect buffer access
    uint8_t mutexWaitMs;      // Time to wait for mutex release for all methods
    uint16_t size;            // Size of the circular buffer. This type should match that of head, tail indeces
} circularBuffer;             // Max size is uint16_t 65535

// Static functions

// User functions
uint32_t app_compute_crc32_str(char * str, int data_len);
uint32_t app_compute_crc32_bytes(uint8_t * bytes, int data_len);
//uint8_t hex2dec(uint8_t data);
uint8_t hex2dec(char data);
//int hex2dec(char hex[]);
uint8_t concat_hex_chars(char high, char low);

// circ_buff port from remote source. circ buff chosen for i2s data bc stale streaming data is of no use to user
void init_buffer(circularBuffer *cb, uint16_t size);
void clear_buffer(circularBuffer *cb);
void reset_buffer(circularBuffer *cb);
void push_data(circularBuffer *cb, uint8_t data);
int pop_data(circularBuffer *cb); // returns -1 is buffer empty
void push_blk_data(circularBuffer *cb, uint8_t * data, uint8_t size); // making judgement call that 255 data members gives us more than enough margin
int pop_blk_data(circularBuffer *cb, uint8_t size); // returns -1 is buffer empty

#ifdef __cplusplus
}
#endif

#endif  // APP_UTILITY_H

/*========================================= END FILE ============================================*/