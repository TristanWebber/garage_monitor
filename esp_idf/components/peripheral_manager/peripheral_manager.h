#ifndef PERIPHERAL_MANAGER_H
#define PERIPHERAL_MANAGER_H

#include <stdbool.h>

#include "esp_err.h"
#include "freertos/idf_additions.h"

#include "am2302_rmt.h"

typedef struct SensorData
{
   bool door_status;
   float temperature;
   float humidity;
} SensorData;

// Setup the connections to the sensors
am2302_handle_t sensor_init(void);

// Configure the interrupt handler for the door switch
void interrupt_init(TaskHandle_t *door_interrupt_handle);

// Record reads from the sensors to the SensorData struct
esp_err_t sensor_read(am2302_handle_t *sensor, SensorData *sensor_data);

// Debounce interrupt reads from door switch and return the state after settling period
bool debounce_door(gpio_num_t gpio, TaskHandle_t *door_interrupt_handle);

#endif /* PERIPHERAL_MANAGER_H */
