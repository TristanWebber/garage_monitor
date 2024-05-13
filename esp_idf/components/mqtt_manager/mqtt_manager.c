#include "esp_log.h"
#include "mqtt_client.h"

#include "mqtt_manager.h"
#include "config.h"

// Private function declarations
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);

static const char *TAG = "MQTT_MANAGER";

static esp_mqtt_client_handle_t client;

extern const uint8_t ca_cert_pem_start[]   asm("_binary_ca_cert_pem_start");
extern const uint8_t ca_cert_pem_end[]     asm("_binary_ca_cert_pem_end");

void mqtt_client_init(void) {

    ESP_LOGI(TAG, "Attempting to connect to MQTT Broker.");

    const esp_mqtt_client_config_t mqtt_client_config = {
        .broker = {
            .address.uri = MQTT_SERVER,
            .verification.certificate = (const char *)ca_cert_pem_start
        },
        .credentials.username = MQTT_USERNAME,
        .credentials.authentication.password = MQTT_PASSWORD
    };

    client = esp_mqtt_client_init(&mqtt_client_config);
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(client, MQTT_EVENT_ANY, mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(client));
    
    ESP_LOGI(TAG, "Connected to MQTT Broker.");
}

bool mqtt_publish(const char *topic, const char *payload, bool retain) {
    ESP_LOGI(TAG, "Publishing message: \"%s\" to topic: \"%s\"", payload, topic);
    int pub_state = esp_mqtt_client_publish(client, topic, payload, strlen(payload), 0, retain);

    if (pub_state < 0) {
        ESP_LOGE(TAG, "Message not published");
        return false;
    }

    return true;

    ESP_LOGI(TAG, "Message published successfully");
}

// Event handler registered to receive MQTT events. Copied from IDF example
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32, base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        // Subscribe would go here if needed
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        // Connection handler logic would go here if needed. Client manages this automatically.
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
