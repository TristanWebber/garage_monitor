#include <Arduino.h>

#include "config.h"
#include "connection_manager.h"
#include "peripheral_manager.h"

void setup() {
    // Optionally print debug messages to serial
    #if defined(DEBUG_SERIAL)
    Serial.begin(BAUD);
    delay(1000);
    #endif

    // Configure sensors
    sensor_init();

    // Connect to WiFi network
    wifi_init();
    wifi_connect();

    // Connect to MQTT Broker
    mqtt_init();
    mqtt_connect();
}

uint32_t last_ping, last_send = 0;
bool first_send = true;
void loop() {

    // Send ping at intervals to maintain connection
    if (millis() - last_ping >= SECONDS_TO_MILLIS(MQTT_PING_INTERVAL)) {
        connection_handler();
        last_ping = millis();
    }

    // Read sensors and publish data at intervals
    if ((millis() - last_send >= SECONDS_TO_MILLIS(PUBLISH_INTERVAL)) || first_send) {

        SensorData sensor_data;
        read_sensors(&sensor_data);

        // Publish data to the broker
        connection_handler();
        mqtt_publish(DOOR_TOPIC, String(sensor_data.door_status).c_str());
        mqtt_publish(TEMP_TOPIC, String(sensor_data.temperature).c_str());
        mqtt_publish(HUMI_TOPIC, String(sensor_data.humidity).c_str());
        first_send = false;
        last_send = millis();
        last_ping = last_send;
    }
}
