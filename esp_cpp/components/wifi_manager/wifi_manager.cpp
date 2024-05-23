#include "wifi_manager.h"

const static char *TAG = "WIFI_MANAGER";

std::mutex Wifi::init_mutex{};
std::mutex Wifi::connect_mutex{};
wifi_init_config_t Wifi::wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
wifi_config_t Wifi::wifi_config{};
const uint8_t Wifi::MAX_RETRIES;
uint8_t Wifi::retry_count{0};
Wifi::state_e Wifi::_state{state_e::NOT_INITIALISED};

Wifi::Wifi(/* args */) {
}

Wifi::~Wifi() {
}

esp_err_t Wifi::init(void) {
    ESP_LOGI(TAG, "Initialising Wifi connection");    

    esp_err_t init_state{ESP_OK};

    // Acquire the initialisation mutex
    std::lock_guard<std::mutex> lock(init_mutex);

    // Only init the wifi if the process has not already been commenced
    if (_state != state_e::NOT_INITIALISED) {
        init_state = ESP_FAIL;
    }

    /* Start the network interface. Errors fall through, rather than abort.
     * The intent is that the failure to init would be handled in the
     * application rather than the component */
    ESP_LOGV(TAG, "Attempting to start event loop and network interface.");
    init_state = esp_netif_init();

    if (init_state == ESP_OK) {
        init_state = esp_event_loop_create_default();
    }

    if (init_state == ESP_OK) {
        const esp_netif_t *netif = esp_netif_create_default_wifi_sta();

        if (!netif) {
            init_state = ESP_FAIL;
            ESP_LOGE(TAG, "Unable to create network interface");
        }
    }
    
    // Init the wifi with default config structure
    if (init_state == ESP_OK) {
        ESP_LOGV(TAG, "Event loop and network interface started successfully");
        ESP_LOGV(TAG, "Attempting to allocate resource for wifi driver.");
        init_state = esp_wifi_init(&wifi_init_config);
    }

    // Register event handler functions to wifi and IP events
    if (init_state == ESP_OK) {
        ESP_LOGV(TAG, "Resources for wifi driver allocated successfully");
        ESP_LOGV(TAG, "Registering wifi event handler");
        init_state = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    }

    if (init_state == ESP_OK) {
        ESP_LOGV(TAG, "Wifi event handler registered successfully");
        ESP_LOGV(TAG, "Registering ip event handler");
        init_state = esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, ip_event_handler, NULL);
    }

    // Set wifi to station mode
    if (init_state == ESP_OK) {
        ESP_LOGV(TAG, "IP event handler registered successfully");
        ESP_LOGV(TAG, "Attempting to start wifi connection in STA mode.");
        init_state = esp_wifi_set_mode(WIFI_MODE_STA);
    }
    
    if (init_state == ESP_OK) {
        ESP_LOGV(TAG, "Operating mode set to STA successfully");
        ESP_LOGV(TAG, "Attempting to set the configuration settings.");
        wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA_WPA2_PSK;
        memcpy(wifi_config.sta.ssid,Config::WIFI_SSID, std::min(strlen(Config::WIFI_SSID), (size_t)sizeof(wifi_config.sta.ssid)));
        memcpy(wifi_config.sta.password,Config::WIFI_PASS, std::min(strlen(Config::WIFI_PASS), (size_t)sizeof(wifi_config.sta.password)));
        init_state = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    }
    

    if (init_state == ESP_OK) {
        ESP_LOGV(TAG, "Wifi config set successfully");
        ESP_LOGV(TAG, "Attempting to start the wifi connection.");
        init_state = esp_wifi_start();
    }
    
    if (init_state == ESP_OK) {
        ESP_LOGI(TAG, "Wifi STA connection started successfully");
        _state = state_e::INITIALISED;
    } else {
        ESP_LOGE(TAG, "Wifi STA connection failed");
        _state = state_e::ERROR;
    }
    
    return init_state;
}

void Wifi::wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    
    // Only handle wifi events
    if (event_base != WIFI_EVENT) {
        return;
    }

    // Acquire the connection mutex
    std::lock_guard<std::mutex> lock(connect_mutex);

    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
        ESP_LOGV(TAG, "WIFI_EVENT_STA_START");
        ESP_LOGI(TAG, "Attempting to connect Wifi");
        _state = state_e::READY_TO_CONNECT;
        esp_wifi_connect();
        break;
    case WIFI_EVENT_STA_CONNECTED:
        ESP_LOGV(TAG, "WIFI_EVENT_STA_CONNECTED");
        ESP_LOGI(TAG, "Wifi Station connected successfully");
        _state = state_e::WAITING_FOR_IP;
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
        ESP_LOGV(TAG, "WIFI_EVENT_STA_DISCONNECTED");
        ESP_LOGW(TAG, "Wifi Station disconnected");
        _state = state_e::DISCONNECTED;
        if (retry_count < MAX_RETRIES) {
            esp_wifi_connect();
            retry_count++;
            ESP_LOGI(TAG, "Attempting to reconnect to wifi. Attempt %i of %i", retry_count, MAX_RETRIES);
        } else {
            ESP_LOGE(TAG, "Maximum wifi connection attempts reached. Entering deep sleep");
            // Short delay to ensure serial buffer is flushed before sleep
            vTaskDelay(pdMS_TO_TICKS(100));
            esp_deep_sleep(Config::PUBLISH_INTERVAL_SECONDS * 1e6);
        }
        break;
    default:
        ESP_LOGV(TAG, "Unspecified wifi event registered. Event id code: %li", event_id);
        _state = state_e::ERROR;
        break;
    }
}

void Wifi::ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {

    // Only handle IP events
    if (event_base != IP_EVENT) {
        return;
    }

    // Acquire the connection mutex
    std::lock_guard<std::mutex> lock(connect_mutex);

    switch (event_id)
    {
    case IP_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "IP assigned. Successfully connected to access point: \"%s\"", Config::WIFI_SSID);
        retry_count = 0;
        _state = state_e::CONNECTED;
        break;
    default:
        ESP_LOGV(TAG, "Unspecified IP event registered. Event id code: %li", event_id);
        _state = state_e::ERROR;
        break;
    }
}

Wifi::state_e Wifi::get_state(void) {
    return _state;
}
