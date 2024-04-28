#ifndef APP_ADC_H
#define APP_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

// Includes 
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_continuous.h"
#include "app_utility.h"

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
typedef struct {
    int count;
    int sum;
    int buff[10];
    int buff_len;
    bool bufferFullFlag;
} adc_filter_t;

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
} adcOneshotParams_t;

typedef enum {
    TEMP_UNKNOWN,
    TEMP_COLD,
    TEMP_ROOM,
    TEMP_WARM,
    TEMP_HOT
} adc_temp_t;

extern const char * adc_temp_names[5];

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