#ifndef CONNECTION_MANAGER_H
#define CONNECTION_MANAGER_H

#define SECONDS_TO_MILLIS(x) (x * 1000)

// Set WiFi mode as STA, set CA certificate and prepare to connect
void wifi_init();

// Attempt to connect to WiFi network. Restarts after 10 unsuccessful attempts.
void wifi_connect();

// Prepare to connect to MQTT Broker
void mqtt_init();

// Attempt to connect to MQTT Broker. Restarts after 5 unsuccessful attempts.
void mqtt_connect();

// Check the state of the connection and take action if any service is disconnected
void connection_handler();

// Publish 'data' to MQTT broker on 'topic'
bool mqtt_publish(const char* topic, const char* payload);

#endif /* CONNECTION_MANAGER_H */
