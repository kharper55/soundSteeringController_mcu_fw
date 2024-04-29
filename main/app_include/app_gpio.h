#ifndef APP_GPIO_H
#define APP_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

// Includes
#include "driver/gpio.h"

// Pin Defines

// Inputs
#define FAN_TACHO_PIN           GPIO_NUM_36 // SENSOR_VP on ESP32. Monitor to ensure tachometer is pulsing while fan driven. Use a broad gptimer that is reset on tachometer level changes
#define ECM_SW_PIN              GPIO_NUM_35 // Jack switch monitor from the condenser mic input
#define AUX_SW_PIN              GPIO_NUM_33 // Jack switch monitor from the std aux input (swapped with above on 04/28)

// Outputs
#define HEARTBEAT_LED_PIN       GPIO_NUM_2
#define PWM_BUFF_EN_PIN         GPIO_NUM_32 // ORed control input (with ARTIX7) to enable the output of the PWM buffers for the ultrasonic drive signal
#define LOAD_SWITCH_EN_PIN      GPIO_NUM_19 // Load switch enable control input for phased array drive switcher input power (Fully shutdown device when disabled; circumvent fixed DC path for boost topology)
#define FAN_EN_PIN              GPIO_NUM_23 // Acts as a gate enable signal for a running PWM clock from the FPGA to drive the fan when an NTC voltage reading reaches a particular threshold

// Extra pins for user/future application (ARTIX7 has an additional 3 extra IO pins. These are suitable for bussing out I2S data and eventually intended to add another degree of modularity to the system.)
#define EXTRA_IO0_PIN           GPIO_NUM_4
#define EXTRA_IO1_PIN           GPIO_NUM_5
#define EXTRA_IO2_PIN           GPIO_NUM_18

// Settings

// Macros

// Typedefs
typedef enum {
    LOW,
    HIGH
} gpio_status_t;

// User functions
esp_err_t app_gpio_init(void);
esp_err_t app_heartbeat_toggle(void);

extern const char * gpio_status_names[2];

#ifdef __cplusplus
}
#endif

#endif  // APP_GPIO_H