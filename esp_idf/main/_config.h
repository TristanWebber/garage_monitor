#ifndef CONFIG_H
#define CONFIG_H

#include "soc/gpio_num.h"

#define PUBLISH_INTERVAL      300                  // Sleep for 5 minutes

// Sensor settings
#define DOOR_SW_PIN           GPIO_NUM_3
#define DHT_PIN               GPIO_NUM_4

// WiFi settings
#define SSID_WIFI_STA         "your_ssid"
#define PASSWORD_WIFI_STA     "your_pw"

// MQTT settings
#define MQTT_SERVER           "mqtts://your.mqtt-server.com:8883"
#define BASE_TOPIC            "your_base_mqtt_topic"
#define MQTT_USERNAME         "your_broker_username"
#define MQTT_PASSWORD         "your_broker_password"
#define DOOR_TOPIC BASE_TOPIC "DOOR"
#define TEMP_TOPIC BASE_TOPIC "TEMPERATURE"
#define HUMI_TOPIC BASE_TOPIC "HUMIDITY"

#endif /* CONFIG_H */
