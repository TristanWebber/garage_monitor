#ifndef CONFIG_H
#define CONFIG_H

// Global configuration settings
//#define DEBUG_SERIAL // Uncomment if testing
#define BAUD                  115200
#define PUBLISH_INTERVAL      300                  // Sleep for 5 minutes

// Sensor settings
#define DOOR_SW_PIN          D1
#define PHASE_2_PIN          D2
#define DHT_TYPE             DHT22

// WiFi settings
#define SSID_WIFI_STA        "ssid"
#define PASSWORD_WIFI_STA    "password"

// MQTT settings
#define MQTT_KEEPALIVE        15                   // seconds
#define MQTT_PING_INTERVAL    (MQTT_KEEPALIVE - 5)
#define MQTT_SERVER           "your_broker_here"
#define MQTT_PORT             8883
#define MQTT_CLIENT_ID        "garage_iot"
#define BASE_TOPIC            "enter the base path for the MQTT topic"
#define MQTT_USERNAME         "your username"        // The API key
#define MQTT_PASSWORD         "your password"        // The API key
#define DOOR_TOPIC BASE_TOPIC "DOOR"
#define TEMP_TOPIC BASE_TOPIC "TEMPERATURE"
#define HUMI_TOPIC BASE_TOPIC "HUMIDITY"

#endif /* CONFIG_H */
