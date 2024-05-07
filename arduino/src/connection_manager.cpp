#include "ca_cert.h"
#include "config.h"
#include "connection_manager.h"
#include "debug_print.h"

#include <WiFiClientSecure.h>
#include <PubSubClient.h>

WiFiClientSecure wifi_client;
PubSubClient mqtt_client(wifi_client);

// Set WiFi mode as STA, set CA certificate and prepare to connect
void wifi_init() {
    WiFi.mode(WIFI_AP_STA);
    wifi_client.setCACert(rootCA);
}

// Attempt to connect to WiFi network. Restarts after 10 unsuccessful attempts.
void wifi_connect() {
    WiFi.begin(SSID_WIFI_STA, PASSWORD_WIFI_STA);
    DBG_PRINT("Connecting to Wifi.");

    uint8_t connection_attempts = 10;
    while (WiFi.status() != WL_CONNECTED && connection_attempts > 0) {
        connection_attempts--;
        delay(500);
        DBG_PRINT(".");
    }
    if (WiFi.status() != WL_CONNECTED) {
        DBG_PRINTLN("WiFi connection unsuccessful. Restarting...");
        delay(10);
        esp_restart();
    } else {
        DBG_PRINTLN("Wifi connected.");
    }
}

// Prepare to connect to MQTT Broker
void mqtt_init() {
    mqtt_client.setKeepAlive(MQTT_KEEPALIVE);
    mqtt_client.setServer(MQTT_SERVER, MQTT_PORT);
}

// Attempt to connect to MQTT Broker. Restarts after 5 unsuccessful attempts.
void mqtt_connect() {
    uint8_t connection_attempts = 5;
    while (!mqtt_client.connected() && connection_attempts > 0) {
        DBG_PRINT("Connecting to MQTT Broker.");
        if(mqtt_client.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD)) {
            DBG_PRINTLN("MQTT broker connected.");
            return;
        } else {
            connection_attempts--;
            DBG_PRINT("Connection to MQTT broker failed. Reason code = ");
            DBG_PRINT(mqtt_client.state());
            DBG_PRINTLN(" . Retrying in 5 seconds.");
            delay(5000);
        }
    }

    DBG_PRINTLN("MQTT connection unsuccessful. Restarting...");
    delay(10);
    esp_restart();
}

// Check the state of the connection and take action if any service is disconnected
void connection_handler() {
    // Check WiFi connected
    if (WiFi.status() != WL_CONNECTED) {
        wifi_connect();
    }

    // Check MQTT connected
    if (!mqtt_client.connected()) {
        mqtt_connect();
    }

    // Send PINGREQ to broker to keep connection alive
    mqtt_client.loop();
    DBG_PRINTLN("pinged broker.");
}

// Publish 'data' to MQTT broker on 'topic'
bool mqtt_publish(const char* topic, const char* payload) {
    if (mqtt_client.publish(topic, payload, false)) {
        DBG_PRINTLN("Published message successfully.");
        return true;
    } else {
        DBG_PRINTLN("Publish unsuccessful.");
        return false;
    }
}
