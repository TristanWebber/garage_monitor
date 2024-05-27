#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#include "peripheral_manager.h"
#include "dht22.h"

const static char *TAG = "PERIPHERAL_MANAGER";

// Interrupt handler for door switch. Start debounce timer
static void door_state_change_callback(void *pvParameters) {
    TimerHandle_t *debounce_timer_handle = (TimerHandle_t *)pvParameters;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xTimerStopFromISR(*debounce_timer_handle, &xHigherPriorityTaskWoken);
    xTimerStartFromISR(*debounce_timer_handle, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

Sensors::Sensors(gpio_num_t door_sw_pin, gpio_num_t dht_pin) {
    _door_sw_pin = door_sw_pin;
    _dht_pin = dht_pin;
    _sensor_data = {false, 0, 0};
}

Sensors::~Sensors() {
    delete _dht22;
}

esp_err_t Sensors::init(TimerHandle_t *debounce_timer_handle) {
    ESP_LOGI(TAG, "Initialising the connection to sensors");

    // Set the door switch
    gpio_set_direction(_door_sw_pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(_door_sw_pin, GPIO_PULLDOWN_ONLY);
    gpio_set_intr_type(_door_sw_pin, GPIO_INTR_ANYEDGE);
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(_door_sw_pin, door_state_change_callback, (void *)debounce_timer_handle));

    // Set the DHT
    _dht22 = new DHT22(_dht_pin);

    return ESP_OK;
}

esp_err_t Sensors::read(SensorData *sensor_data) {

    sensor_data->door_status = gpio_get_level(_door_sw_pin);

    esp_err_t read_status = _dht22->read();
    if (read_status == ESP_OK) {
        sensor_data->temperature = _dht22->get_temperature();
        sensor_data->humidity = _dht22->get_humidity();
    } else {
        sensor_data->temperature = 0;
        sensor_data->humidity = 0;
    }

    ESP_LOGI(TAG, "Values: Door = %d, Temp = %f, Humi = %f.", sensor_data->door_status, sensor_data->temperature, sensor_data->humidity);

    return read_status;
}

// Reads from door switch and returns the state
bool Sensors::get_door_state(void) {
    return gpio_get_level(_door_sw_pin);
}
