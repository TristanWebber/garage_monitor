#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include "main.h"
#include "config.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_sleep.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <algorithm>
#include <cstring>
#include <mutex>

class Wifi
{
private:
    static std::mutex init_mutex;
    static std::mutex connect_mutex;
    static wifi_init_config_t wifi_init_config;
    static wifi_config_t wifi_config;
    static const uint8_t MAX_RETRIES{5};
    static uint8_t retry_count;

    static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
    static void ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
public:
    enum class state_e {
        NOT_INITIALISED,
        INITIALISED,
        READY_TO_CONNECT,
        CONNECTING,
        WAITING_FOR_IP,
        CONNECTED,
        DISCONNECTED,
        ERROR,
    };

    Wifi(/* args */);
    ~Wifi();
    esp_err_t init(void);
    state_e get_state(void);

private:
    static state_e _state;
};



#endif /* WIFI_MANAGER_H */
