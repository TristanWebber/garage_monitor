#ifndef PERIPHERAL_MANAGER_H
#define PERIPHERAL_MANAGER_H

#include <atomic>

#include "esp_err.h"
#include "soc/gpio_num.h"

#include "dht22.h"

class Sensors {

public:

    typedef struct SensorData {
        bool door_status;
        float temperature;
        float humidity;
    } SensorData;

    typedef void (Sensors::*interrupt_handle_t)(std::atomic_flag&);

    Sensors(gpio_num_t door_sw_pin, gpio_num_t dht_pin);
    ~Sensors(void);

    esp_err_t init(void);
    interrupt_handle_t get_interrupt_handle(void);
    esp_err_t read(SensorData *sensor_data);
    bool get_door_state(void);

private:
    gpio_num_t _door_sw_pin;
    gpio_num_t _dht_pin;
    SensorData _sensor_data;
    DHT22 *_dht22;

    [[noreturn]] void interrupt(std::atomic_flag& atomic_flag);
};

#endif /* PERIPHERAL_MANAGER_H */
