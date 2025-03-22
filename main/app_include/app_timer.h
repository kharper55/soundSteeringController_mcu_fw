/*===================================================================================================
    File: app_timer.h
    Author: Kevin Harper
    Date: 01/2024
    Details: Function prototypes, constants, preprocessor defs/macros for timer peripheral.

    Written using ESP-IDF v5.1.1 API. Built in 03/2025 using v5.1.2
//==================================================================================================*/

#ifndef APP_TIMER_H
#define APP_TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

// Includes
#include "driver/gptimer.h" // IDF provided driver interface
#include "app_utility.h"    // Various unction prototypes, constants, preprocessor defs/macros, typedefs

// Macros
#define GPTIMER_BASE_TICK_FREQ_HZ (uint32_t)(1 * 1000 * 1000) // Set 1MHz base resolution
#define GPTIMER_MS_TO_TICKS(X)    (uint32_t)((X / 1000.0) * GPTIMER_BASE_TICK_FREQ_HZ)
#define GPTIMER_TICKS_TO_MS(X)    (uint32_t)((X * 1000.0) / GPTIMER_BASE_TICK_FREQ_HZ)

//typedef void (*gptimer_callback_t) (void *); // void pointer typedef?

typedef struct {
    gptimer_handle_t * timerHandle;
    bool running;
} gptimer_state_t;

esp_err_t app_initTimer(gptimer_handle_t * timerHandle, void (*cb)(int), uint16_t time_ms, bool oneShot);
esp_err_t app_toggleTimerRun(gptimer_handle_t * timerHandle, gptimer_state_t * state);
esp_err_t app_reconfigTimerAlarm(gptimer_handle_t * timerHandle, gptimer_state_t * state, uint16_t time_ms, bool oneShot);

#ifdef __cplusplus
}
#endif

#endif  // APP_TIMER_H