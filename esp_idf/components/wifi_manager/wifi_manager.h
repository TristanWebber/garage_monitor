#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include "esp_err.h"

// Initiate wifi and connect in station mode. Attempt for a maximum duration of timeout_millis
esp_err_t wifi_init(uint32_t timeout_millis);

#endif /* WIFI_MANAGER_H */
