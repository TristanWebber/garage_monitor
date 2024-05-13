#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "wifi_manager.h"
#include "config.h"

// Private functions
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t eventId, void* event_data);

static const char *TAG = "WIFI_MANAGER";

static EventGroupHandle_t wifi_event_group_handle;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define MAX_RETRY_COUNT    5

static uint8_t retry_count = 0;

esp_err_t wifi_init(uint32_t timeout_millis) {
    ESP_LOGI(TAG, "Attempting to connect to WiFi.");
    wifi_event_group_handle = xEventGroupCreate();

    // Start the network interface
    ESP_LOGV(TAG, "Attempting to start event loop and network interface.");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    ESP_LOGV(TAG, "Event loop and network interface started.");

    // Init the wifi with default config structure
    ESP_LOGV(TAG, "Attempting to allocate resource for wifi driver.");
    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
    ESP_LOGV(TAG, "Resources for wifi driver allocated.");

    // Register event handler function to wifi and IP events
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL));

    // Set wifi to station mode
    ESP_LOGV(TAG, "Attempting to start wifi connection in STA mode.");
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = SSID_WIFI_STA,
            .password = PASSWORD_WIFI_STA,
            .threshold.authmode = WIFI_AUTH_WPA_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wifi STA connection started.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t wifi_event_bits = xEventGroupWaitBits(
        wifi_event_group_handle,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        pdMS_TO_TICKS(timeout_millis)
    );

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    
    if (wifi_event_bits == WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to ap SSID: %s", SSID_WIFI_STA);
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Failed to connect to SSID: %s", SSID_WIFI_STA);
    return ESP_FAIL;
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* eventData) {
    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
        ESP_LOGV(TAG, "WIFI_EVENT_STA_START");
        esp_wifi_connect();
        break;
    case WIFI_EVENT_STA_CONNECTED:
        ESP_LOGV(TAG, "WIFI_EVENT_STA_CONNECTED");
        break;
    case IP_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "IP_EVENT_STA_GOT_IP");
        retry_count = 0;
        xEventGroupSetBits(wifi_event_group_handle, WIFI_CONNECTED_BIT);
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
        ESP_LOGE(TAG, "WIFI_EVENT_STA_DISCONNECTED");
        if (retry_count < MAX_RETRY_COUNT) {
            vTaskDelay(pdMS_TO_TICKS(500));
            esp_wifi_connect();
            retry_count++;
            ESP_LOGI(TAG, "Attempting to reconnect to wifi. Attempt %i of %i", retry_count, MAX_RETRY_COUNT);
        } else {
            ESP_LOGE(TAG, "Maximum wifi connection attempts exceeded. Restarting");
            esp_restart();
        }
        xEventGroupSetBits(wifi_event_group_handle, WIFI_FAIL_BIT);
        break;
    default:
        break;
    }
}
