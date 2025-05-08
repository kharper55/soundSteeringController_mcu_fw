/*===================================================================================================
    File: app_adc.h
    Author: Kevin Harper
    Date: 01/2024
    Details: Function prototypes, constants, preprocessor defs/macros for ADC peripheral

    Written using ESP-IDF v5.1.1 API. Built in 03/2025 using v5.1.2
//==================================================================================================*/

#ifndef APP_ADC_H
#define APP_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

// Includes 
#include "esp_adc/adc_oneshot.h"        // IDF build tools provide standard interfaces for ADC drivers
#include "esp_adc/adc_cali.h"           // Calibration routines to compensate for measurement nonidealities
#include "esp_adc/adc_cali_scheme.h"    // See above.
#include "esp_adc/adc_continuous.h"     // IDF interface for continuous mode ADC driver
#include "app_utility.h"                // Various function prototypes, constants, preprocessor defs/macros, typedefs

// Pin Defines 
#define VDRIVE_BUFF_PIN         GPIO_NUM_34 // ADC16
#define APP_ADC1_CHAN0          ADC_CHANNEL_6

// NTC Thermistor ADC voltage divider monitoring SMPS output inductor
#define NTC_BUFF_PIN            GPIO_NUM_39 // ADC13 "SENSOR_VN"
#define APP_ADC1_CHAN1          ADC_CHANNEL_3

// Settings
#define APP_ADC_ATTEN           ADC_ATTEN_DB_0
#define ADC_TAG                 "ADC"

#define PCT_MIN                   0
#define PCT_MAX                   100
#define ADC_MAX                   1100
#define ADC_MIN                   100
#define SCALE_VPOT(X)             (int)((((X - ADC_MIN) * (PCT_MAX - PCT_MIN))/(float)(ADC_MAX - ADC_MIN)) + PCT_MIN)
#define SCALE_VPOT_INVERT(X)      (int)(PCT_MAX - (((X - ADC_MIN) * (PCT_MAX - PCT_MIN))/(float)(ADC_MAX - ADC_MIN)))
#define GET_NTC_TEMP(X)           SCALE_VPOT_INVERT(X)

// Macros

// Typedefs

// General parameters for a circular buffer averaging filter accepting ADC
// counts as an input (see 'adc_filter(...)' definition)
typedef struct {
    int count;
    int sum;
    int buff[10];
    int buff_len;
    bool bufferFullFlag;
} adc_filter_t;

// General parameters for the ADC FreeRTOS task (for oneShot operation)
// should factor this to include continuous mode operation stuff
typedef struct {
    char * TAG;
    adc_oneshot_unit_handle_t * handle;
    adc_cali_handle_t * cali_handle;
    adc_unit_t unit;
    adc_channel_t channel;
    adc_atten_t atten;
    adc_filter_t * filt;
    int delay_ms;
    int * vraw;
    int * vcal;
    int * vfilt;
} adcParams_t;

// An enum-like type giving a more qualitative indication of the temperature measurement
// from the ADC peripheral
typedef enum {
    TEMP_UNKNOWN,
    TEMP_COLD,
    TEMP_ROOM,
    TEMP_WARM,
    TEMP_HOT
} adc_temp_t;

// Descriptive string names for the adc_temp_t values
extern const char * temperature_names[5];

// User functions
esp_err_t adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
void adc_calibration_deinit(adc_cali_handle_t handle);
esp_err_t adc_oneshot_init(adc_oneshot_unit_handle_t * adc_handle, adc_unit_t unit, adc_channel_t channel); /* I hate this function so much but hoping to factor lines out of app_main */
float adc_filter(int value, adc_filter_t * filterObject);
void adc_continuous_init(adc_continuous_handle_t * adc_handle, adc_unit_t * units, adc_channel_t * channels, uint8_t num_channels);
void get_drive_temp(adc_temp_t * temperature, int vmeas);

#ifdef __cplusplus
}
#endif

#endif  // APP_ADC_H

/*========================================= END FILE ============================================*/