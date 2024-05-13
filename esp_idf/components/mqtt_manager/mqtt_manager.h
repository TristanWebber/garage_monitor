#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include <stdbool.h>

// Connect to MQTT broker and start client
void mqtt_client_init(void);

// Publish a message at qos1
bool mqtt_publish(const char *topic, const char *payload, bool retain);

#endif /* MQTT_MANAGER_H */
