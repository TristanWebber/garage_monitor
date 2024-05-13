#include <stdio.h>

#include "esp_err.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "am2302_rmt.h"

#include "config.h"
#include "peripheral_manager.h"
#include "wifi_manager.h"
#include "mqtt_manager.h"

static TaskHandle_t door_interrupt_handle = NULL;

void read_and_send_task(void *pvParameters);
void interrupt_send_task(void *pvParameters);

void app_main(void) {

    am2302_handle_t sensor = sensor_init();

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(wifi_init(10000));

    mqtt_client_init();

    interrupt_init(&door_interrupt_handle);

    xTaskCreate(&read_and_send_task, "read_and_send_task", 5 * 1024, (void *)sensor, 1, NULL);
    xTaskCreate(&interrupt_send_task, "interrupt_send_task", 5 * 1024, NULL, 1, &door_interrupt_handle);
}

void read_and_send_task(void *pvParameters) {

    TickType_t task_start = xTaskGetTickCount();

    while (true) {
        am2302_handle_t sensor = (am2302_handle_t)pvParameters;
        SensorData sensor_data;
        sensor_read(&sensor, &sensor_data);

        char pub_buff[6];
        sprintf(pub_buff, "%d", sensor_data.door_status);
        mqtt_publish(DOOR_TOPIC, pub_buff, false);
        sprintf(pub_buff, "%.2f", sensor_data.temperature);
        mqtt_publish(TEMP_TOPIC, pub_buff, false);
        sprintf(pub_buff, "%.2f", sensor_data.humidity);
        mqtt_publish(HUMI_TOPIC, pub_buff, false);

        xTaskDelayUntil(&task_start, PUBLISH_INTERVAL * 1000 / portTICK_PERIOD_MS);

    }
    vTaskDelete(NULL);
}

void interrupt_send_task(void *pvParameters) {
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        bool door_status = debounce_door(DOOR_SW_PIN, &door_interrupt_handle);

        char pub_buff[2];
        sprintf(pub_buff, "%d", door_status);
        mqtt_publish(DOOR_TOPIC, pub_buff, false);
    }
    vTaskDelete(NULL);
}
