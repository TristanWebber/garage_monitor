#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include "config.h"
#include "mqtt_client.h"
#include "esp_log.h"

#include <algorithm>
#include <cstring>
#include <mutex>

extern const uint8_t ca_cert_pem_start[]   asm("_binary_ca_cert_pem_start");
extern const uint8_t ca_cert_pem_end[]     asm("_binary_ca_cert_pem_end");

class Mqtt
{
private:

    static std::mutex init_mutex;
    static std::mutex publish_mutex;
    static esp_mqtt_client_handle_t client;
    static esp_mqtt_client_config_t client_config;

    static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
public:
    enum class state_e {
        NOT_INITIALISED,
        INITIALISED,
        CONNECTING,
        CONNECTED,
        DISCONNECTED,
        ERROR,
    };

    Mqtt(/* args */);
    ~Mqtt();
    esp_err_t client_init(void);
    bool publish(const char *topic, char *message, bool retain);
    state_e get_state(void);

private:
    static state_e _state;
};



#endif /* MQTT_MANAGER_H */
