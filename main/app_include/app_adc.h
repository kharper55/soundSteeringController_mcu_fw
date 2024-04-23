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

// Macros

// Typedefs
typedef struct {
    int count;
    int sum;
    int buff[10];
    int buff_len;
    bool bufferFullFlag;
} adc_filter_t;

// User functions
esp_err_t adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
void adc_calibration_deinit(adc_cali_handle_t handle);
esp_err_t adc_oneshot_init(adc_oneshot_unit_handle_t * adc_handle, adc_unit_t unit, adc_channel_t channel); /* I hate this function so much but hoping to factor lines out of app_main */
float adc_filter(int value, adc_filter_t * filterObject);

#ifdef __cplusplus
}
#endif

#endif  // APP_ADC_H