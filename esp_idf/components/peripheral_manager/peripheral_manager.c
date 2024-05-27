#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#include "am2302_rmt.h"

#include "config.h"
#include "peripheral_manager.h"

// TAG for logging
const static char *TAG = "PERIPHERAL_MANAGER";

// Setup the connections to the sensors
am2302_handle_t sensor_init(void) {
    ESP_LOGI(TAG, "Initialising the connection to sensors");

    // Set the door switch as a digital input with a pulldown resistor
    gpio_set_direction(DOOR_SW_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(DOOR_SW_PIN, GPIO_PULLDOWN_ONLY);
    
    // Set pin and clock source for DHT22 sensor
    am2302_config_t am2302_config = {
        .gpio_num = DHT_PIN,
    };
    am2302_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
    };
    am2302_handle_t sensor = NULL;
    ESP_ERROR_CHECK(am2302_new_sensor_rmt(&am2302_config, &rmt_config, &sensor));
    ESP_LOGI(TAG, "Sensors initialised.");

    return sensor;
}

// Record reads from the sensors to the SensorData struct
esp_err_t sensor_read(am2302_handle_t *sensor, SensorData *sensor_data) {

    // Switch is normally open. A high read means the door is closed
    ESP_LOGI(TAG, "Taking a read from the door sensor");
    sensor_data->door_status = gpio_get_level(DOOR_SW_PIN);
    ESP_LOGI(TAG, "Door is closed?: %d", sensor_data->door_status);

    ESP_LOGI(TAG, "Taking reads from the environmental sensor");
    esp_err_t read_success = am2302_read_temp_humi(*sensor, &sensor_data->temperature, &sensor_data->humidity);
    ESP_LOGI(TAG, "Temperature: %.1f °C, Humidity: %.1f %%", sensor_data->temperature, sensor_data->humidity);

    return read_success;
}

// Interrupt handler for door switch
static void door_state_change_callback(void *pvParameters) {
    TimerHandle_t *debounce_timer_handle = (TimerHandle_t *)pvParameters;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xTimerStopFromISR(*debounce_timer_handle, &xHigherPriorityTaskWoken);
    xTimerStartFromISR(*debounce_timer_handle, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Configure the interrupt handler for the door switch
void interrupt_init(TimerHandle_t *debounce_timer_handle) {
    gpio_set_intr_type(DOOR_SW_PIN, GPIO_INTR_ANYEDGE);
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(DOOR_SW_PIN, door_state_change_callback, (void *)debounce_timer_handle));
}

// Reads from door switch and returns the state
bool get_door_state(gpio_num_t gpio) {
    return gpio_get_level(gpio);
}
