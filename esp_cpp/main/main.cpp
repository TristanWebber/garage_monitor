#include "esp_err.h"
#include "freertos/projdefs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "main.h"
#include "config.h"
#include "peripheral_manager.h"
#include "wifi_manager.h"
#include "mqtt_manager.h"

static void read_and_send_task(void *pvParameters);
static void interrupt_send_callback(TimerHandle_t xTimer);

static Main cpp_main;
Sensors sensors(Config::DOOR_SW_PIN, Config::DHT_PIN);
Wifi wifi;
Mqtt mqtt_client;

static TimerHandle_t debounce_timer_handle;

void Main::start(void) {

    sensors.init(&debounce_timer_handle);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(wifi.init());

    while (wifi.get_state() != Wifi::state_e::CONNECTED) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    ESP_ERROR_CHECK(mqtt_client.client_init());

    create_tasks();
}

void::Main::create_tasks(void) {
    xTaskCreate(&read_and_send_task, "read_and_send_task", 5 * 1024, NULL, 1, NULL);
    debounce_timer_handle = xTimerCreate("interrupt_send_callback", pdMS_TO_TICKS(Config::DEBOUNCE_DURATION_MILLIS), pdFALSE, (void *)0, interrupt_send_callback);
}

void read_and_send_task(void *pvParameters) {

    TickType_t task_start = xTaskGetTickCount();

    while (true) {
        Sensors::SensorData sensor_data;
        sensors.read(&sensor_data);

        while (mqtt_client.get_state() != Mqtt::state_e::CONNECTED) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        char pub_buff[6];
        sprintf(pub_buff, "%d", sensor_data.door_status);
        mqtt_client.publish(Config::DOOR_TOPIC, pub_buff, false);
        sprintf(pub_buff, "%.2f", sensor_data.temperature);
        mqtt_client.publish(Config::TEMP_TOPIC, pub_buff, false);
        sprintf(pub_buff, "%.2f", sensor_data.humidity);
        mqtt_client.publish(Config::HUMI_TOPIC, pub_buff, false);

        xTaskDelayUntil(&task_start, Config::PUBLISH_INTERVAL_SECONDS * 1000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void interrupt_send_callback(TimerHandle_t xTimer) {
    bool door_status = sensors.get_door_state();

    while (mqtt_client.get_state() != Mqtt::state_e::CONNECTED) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    char pub_buff[2];
    sprintf(pub_buff, "%d", door_status);
    mqtt_client.publish(Config::DOOR_TOPIC, pub_buff, false);
}

extern "C" void app_main(void) {
    cpp_main.start();
}
