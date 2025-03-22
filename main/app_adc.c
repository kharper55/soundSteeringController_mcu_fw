/*===================================================================================================
    File: app_adc.c
    Author: Kevin Harper
    Date: 01/2024
    Details: Function and global definitions for ADC peripheral FreeRTOS tasks.

    Written using ESP-IDF v5.1.1 API. Built in 03/2025 using v5.1.2
//==================================================================================================*/

#include "app_include/app_adc.h" // Function prototypes, constants, preprocessor defs/macros, typedefs

// Descriptive string names for the adc_temp_t values
const char * temperature_names[5] = {
    "UNKOWN",
    "COLD",
    "ROOM",
    "WARM",
    "HOT"
};

/*---------------------------------------------------------------
    ADC Calibration Curve Fitting Unit Init
---------------------------------------------------------------*/
esp_err_t adc_calibration_init(adc_unit_t unit, adc_channel_t channel, 
                          adc_atten_t atten, adc_cali_handle_t *out_handle) {
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;


    if (!calibrated) {
        ESP_LOGI(ADC_TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(ADC_TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(ADC_TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(ADC_TAG, "Invalid arg or no memory");
    }

    return ret;
}

/*---------------------------------------------------------------
    ADC Calibration Deinit
---------------------------------------------------------------*/
void adc_calibration_deinit(adc_cali_handle_t handle) {
    ESP_LOGI(ADC_TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
}

/*---------------------------------------------------------------
    ADC Unit Oneshot Read Init
---------------------------------------------------------------*/
/*static*/ esp_err_t adc_oneshot_init(adc_oneshot_unit_handle_t * adc_handle, adc_unit_t unit, 
                             adc_channel_t channel) {

    esp_err_t ret = ESP_OK;

    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = unit,
    };

    if (unit == ADC_UNIT_2) {
        init_config.ulp_mode = ADC_ULP_MODE_DISABLE;
    }

    ret = adc_oneshot_new_unit(&init_config, adc_handle);
    if (ret != ESP_OK) {
        return ret;
    }

    // Attenuation to 0-1.1V range handled in hardware for all channels
    adc_oneshot_chan_cfg_t adc_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = APP_ADC_ATTEN,
    };
    ret = adc_oneshot_config_channel(*adc_handle, channel, &adc_config);

    return ret;
}

/*---------------------------------------------------------------
    ADC Filter, Circular Buffer Averaging Function
---------------------------------------------------------------*/
// Refactored to use a custom struct type containing previously static defined information
float adc_filter(int value, adc_filter_t * filterObject) {

    int denominator = 1;
    int temp = 0;

    // Check if the buffer is full
    if (filterObject->count < filterObject->buff_len) {
        // ... hmm dont understand below conditional. maybe the struct member name is off (bufferFullFlag and count < buff_len?)
        if (filterObject->bufferFullFlag) temp = filterObject->buff[filterObject->count]; // Get the current value in the circular buffer about to be overwritten...
        filterObject->buff[filterObject->count] = value;   // Replace the value at the index where temp was
        filterObject->sum += (filterObject->buff[filterObject->count] - temp); // Add to the running sum of circular buffer entries the difference between the newest value added and the value that was previously in its position, so that additions dont need to be recompleted.
        filterObject->count++; // increment the count
    }

    // The buffer is full... Wrap the count and start replacing values...
    if (filterObject->count == filterObject->buff_len) {
        if (!filterObject->bufferFullFlag) filterObject->bufferFullFlag = true;
        filterObject->count = 0;
    }

    denominator = (filterObject->bufferFullFlag) ? filterObject->buff_len : filterObject->count;

    return (filterObject->sum / (float)denominator);
}

/*---------------------------------------------------------------
    ADC Unit Continuous Read Init
---------------------------------------------------------------*/
/*static*/ void adc_continuous_init(adc_continuous_handle_t * adc_handle, adc_unit_t * units, 
                                adc_channel_t * channels, uint8_t num_channels) {

    /* Needs to be ported in/figured out. Not sure continuous mode driver is necessary in this application */
}

/*---------------------------------------------------------------
    ADC Unit Continuous Read Init
---------------------------------------------------------------*/
void get_drive_temp(adc_temp_t * temperature, int vmeas) {
    adc_temp_t * tempStateHandle = temperature;
    static adc_temp_t tempState = TEMP_UNKNOWN;
    float tempReading = GET_NTC_TEMP(vmeas);

    if (tempReading > 70) {         // Temp is between 70 & 100% of the max
        tempState = TEMP_HOT;
    }
    else if (tempReading > 50) {    // Temp is between 50 & 70% of the max
        tempState = TEMP_WARM;
    }
    else if (tempReading > 30) {    // Temp is between 30 & 50% of the max
        tempState = TEMP_ROOM;
    }   
    else if (tempReading > 0) {    // Temp is between 0 & 30% of the max
        tempState = TEMP_COLD;
    }
    else {
        tempState = TEMP_UNKNOWN;    
    }

    *tempStateHandle = tempState;
}