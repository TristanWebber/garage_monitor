#ifndef PERIPHERAL_MANAGER_H
#define PERIPHERAL_MANAGER_H

typedef struct SensorData
{
  bool door_status;
  float temperature;
  float humidity;
} SensorData;

extern SensorData sensor_data;

// Setup the connection to the sensors
void sensor_init();

// Take reads from sensors
bool sensor_read(SensorData *sensor_data);

#endif /* PERIPHERAL_MANAGER_H */
