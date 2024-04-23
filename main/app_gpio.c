#include "app_include/app_gpio.h"

esp_err_t app_gpio_init(void) {
    // GP OUTPUTS
    gpio_reset_pin(HEARTBEAT_LED_PIN);
    gpio_set_direction(HEARTBEAT_LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(HEARTBEAT_LED_PIN, 1);

    gpio_reset_pin(LOAD_SWITCH_EN_PIN);
    gpio_set_direction(LOAD_SWITCH_EN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LOAD_SWITCH_EN_PIN, 1);

    // Init and disable array by default
    gpio_reset_pin(PWM_BUFF_EN_PIN);
    gpio_set_direction(PWM_BUFF_EN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(PWM_BUFF_EN_PIN, 0);

    // AUDIO SWITCH INPUTS
    gpio_reset_pin(ECM_SW_PIN);
    gpio_set_direction(ECM_SW_PIN, GPIO_MODE_INPUT);

    gpio_reset_pin(AUX_SW_PIN);
    gpio_set_direction(AUX_SW_PIN, GPIO_MODE_INPUT);
    
    // Init extra io's as input pulldowns for now
    gpio_reset_pin(EXTRA_IO0_PIN);
    gpio_set_direction(EXTRA_IO0_PIN, GPIO_MODE_INPUT);
    gpio_pulldown_en(EXTRA_IO0_PIN);

    gpio_reset_pin(EXTRA_IO1_PIN);
    gpio_set_direction(EXTRA_IO1_PIN, GPIO_MODE_INPUT);
    gpio_pulldown_en(EXTRA_IO1_PIN);

    gpio_reset_pin(EXTRA_IO2_PIN);
    gpio_set_direction(EXTRA_IO2_PIN, GPIO_MODE_INPUT);
    gpio_pulldown_en(EXTRA_IO2_PIN);

    return ESP_OK;
}

esp_err_t app_heartbeat_toggle(void) {
    static bool heartbeat_level;
    static bool first_call = true;
    esp_err_t ret;

    if (first_call) {
        heartbeat_level = gpio_get_level(HEARTBEAT_LED_PIN);
        first_call = false;
    }
    heartbeat_level = !heartbeat_level;
    ret = gpio_set_level(HEARTBEAT_LED_PIN, heartbeat_level);

    return ret;
}