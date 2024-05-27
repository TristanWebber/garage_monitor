#ifndef CONFIG_H
#define CONFIG_H

#include <cstdint>

#include "soc/gpio_num.h"

class Config
{
private:
    /* data */
public:

    static const uint16_t PUBLISH_INTERVAL_SECONDS{300};
    static const uint8_t DEBOUNCE_DURATION_MILLIS{50};

    static const gpio_num_t DOOR_SW_PIN{GPIO_NUM_3};
    static const gpio_num_t DHT_PIN{GPIO_NUM_4};

    // Wifi constants
    static constexpr const char *WIFI_SSID{"your_ssid"};
    static constexpr const char *WIFI_PASS{"your_password"};

    //MQTT constants
    static constexpr const char *BROKER_URI{"mqtts://your.mqtt-server.com:8883"};
    static constexpr const char *BROKER_USER{"your_broker_username"};
    static constexpr const char *BROKER_PASS{"your_broker_password"};

    static constexpr const char *DOOR_TOPIC{"dtck-pub/your-product/your-device-id/DOOR"};
    static constexpr const char *PHASE_2_TOPIC{"dtck-pub/your-product/your-device-id/TEMPERATURE"};
    static constexpr const char *PHASE_3_TOPIC{"dtck-pub/your-product/your-device-id/HUMIDITY"};
};

#endif /* CONFIG_H */
