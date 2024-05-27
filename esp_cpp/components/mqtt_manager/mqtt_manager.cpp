#include "mqtt_manager.h"

const static char *TAG = "MQTT_MANAGER";

std::mutex Mqtt::init_mutex{};
std::mutex Mqtt::publish_mutex{};
esp_mqtt_client_handle_t Mqtt::client{};
esp_mqtt_client_config_t Mqtt::client_config{};
Mqtt::state_e Mqtt::_state{state_e::NOT_INITIALISED};

Mqtt::Mqtt(/* args */) {
}

Mqtt::~Mqtt() {
}

esp_err_t Mqtt::client_init(void) {
    
    ESP_LOGI(TAG, "Initialising connection to MQTT broker");    

    esp_err_t init_state{ESP_OK};

    // Acquire the initialisation mutex
    std::lock_guard<std::mutex> lock(init_mutex);

    // Only init the MQTT if the process has not already been commenced
    if (_state != state_e::NOT_INITIALISED) {
        init_state = ESP_FAIL;
    }

    client_config.broker.address.uri = Config::BROKER_URI;
    client_config.broker.verification.certificate = (const char *)ca_cert_pem_start;
    client_config.credentials.username = Config::BROKER_USER;
    client_config.credentials.authentication.password = Config::BROKER_PASS;

    ESP_LOGV(TAG, "Attempting to initiate MQTT client");
    client = esp_mqtt_client_init(&client_config);

    if (!client) {
        init_state = ESP_FAIL;
        ESP_LOGE(TAG, "Unable to initiate MQTT client");
    }

    if (init_state == ESP_OK) {
        ESP_LOGV(TAG, "MQTT client initiated successfully");
        ESP_LOGV(TAG, "Attempting to register MQTT event handler");
        _state = state_e::INITIALISED;
        init_state = esp_mqtt_client_register_event(client, MQTT_EVENT_ANY, mqtt_event_handler, NULL);
    }

    if (init_state == ESP_OK) {
        ESP_LOGV(TAG, "MQTT event handler initiated successfully");
        ESP_LOGV(TAG, "Attempting to start MQTT client");
        _state = state_e::CONNECTING;
        init_state = esp_mqtt_client_start(client);
    }

    if (init_state == ESP_OK) {
        ESP_LOGI(TAG, "MQTT client started successfully");
    } else {
        ESP_LOGE(TAG, "Failed to start MQTT client");
        _state = state_e::ERROR;
    }

    return init_state;
}

bool Mqtt::publish(const char *topic, char *message, bool retain) {
    ESP_LOGI(TAG, "Publishing message: \"%s\" to topic: \"%s\"", message, topic);
    int pubState = esp_mqtt_client_publish(client, topic, message, strlen(message), 0, retain);

    if (pubState < 0) {
        ESP_LOGE(TAG, "Message not published");
        return false;
    }

    ESP_LOGI(TAG, "Message published successfully");
    return true;
}


void Mqtt::mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {

    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32, base, event_id);
    
    // Acquire the initialisation mutex
    std::lock_guard<std::mutex> lock(init_mutex);

    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        _state = state_e::CONNECTED;
        // Subscribe would go here if needed
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        _state = state_e::DISCONNECTED;
        // TODO add connection handler logic here
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        // If actions from subs were required they would go here
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        _state = state_e::ERROR;
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            ESP_LOGI(TAG, "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
            ESP_LOGI(TAG, "Last tls stack error number: 0x%x", event->error_handle->esp_tls_stack_err);
            ESP_LOGI(TAG, "Last captured errno : %d (%s)",  event->error_handle->esp_transport_sock_errno,
                     strerror(event->error_handle->esp_transport_sock_errno));
        } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
            ESP_LOGI(TAG, "Connection refused error: 0x%x", event->error_handle->connect_return_code);
        } else {
            ESP_LOGW(TAG, "Unknown error type: 0x%x", event->error_handle->error_type);
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

Mqtt::state_e Mqtt::get_state(void) {
    return _state;
}
