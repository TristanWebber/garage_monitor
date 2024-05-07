#include <Arduino.h>
#include "DHT.h"
#include "config.h"
#include "debug_print.h"
#include "peripheral_manager.h"

DHT dht(DHT_PIN, DHT_TYPE);

// Setup the connections to the sensors
void sensor_init() {
    DBG_PRINTLN("Initialising the connection to sensors");
    pinMode(DOOR_SW_PIN, INPUT_PULLDOWN);
    dht.begin();
}

// Record reads from the sensors to the SensorData struct
bool read_sensors(SensorData *sensor_data) {

    // Switch is normally open. A high read means the door is closed
    DBG_PRINTLN("Taking a read from the door sensor");
    sensor_data->door_status = digitalRead(DOOR_SW_PIN);
    DBG_PRINT("Door is closed?: "); DBG_PRINTLN(sensor_data->door_status);

    // Take temperature and humidity reads (units: celsius, %)
    DBG_PRINTLN("Taking reads from the environmental sensor");
    uint8_t read_attempts = 5;
    bool read_success = false;
    do {
        // Sensor has a 0.5Hz refresh rate, so wait 2 seconds for repeat reads
        if (read_attempts < 5) {
            delay(2000);
        }
        sensor_data->temperature = dht.readTemperature();
        sensor_data->humidity = dht.readHumidity();
        read_success = !isnan(sensor_data->temperature) && !isnan(sensor_data->humidity);
        read_attempts--;
    } while (!read_success && read_attempts > 0);

    DBG_PRINT("Measured temperature: "); DBG_PRINT(sensor_data->temperature); DBG_PRINTLN("degC");
    DBG_PRINT("Measured humidity: "); DBG_PRINT(sensor_data->humidity); DBG_PRINTLN("percent");

    return read_success;
}
