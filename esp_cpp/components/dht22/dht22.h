#ifndef DHT22_H
#define DHT22_H

#include <array>
#include <cstdint>

#include "esp_err.h"
#include "soc/gpio_num.h"
#include "hal/gpio_types.h"

class DHT22 {

private:
    static const int64_t _DHT22_MIN_INTERVAL = 2e6;
    static const uint8_t _DATA_BITS = 40;
    static const uint8_t _DATA_BYTES = _DATA_BITS / 8;
    static const uint16_t _TIMEOUT_US = 1000;
    static const uint8_t _HIGH_PULSE_DURATION = 50;

    std::array<uint8_t, _DATA_BITS> _dht_bits{0};
    std::array<uint8_t, _DATA_BYTES> _dht_bytes{0};

    gpio_num_t _pin;
    int64_t _last_read_time = 0;
    float _temperature = 0;
    float _humidity = 0;
    bool _real_data = false;
    int64_t _us_delay_compensation = 0;

    void gpio_pin_mode(gpio_mode_t mode);
    esp_err_t read_bits(void);
    uint16_t expect_pulse(bool level);
    esp_err_t convert_data(void);
    void us_delay(uint64_t us_delay);
    uint64_t us_delay_calibrate(void);

public:
    DHT22(gpio_num_t dht_pin);
    ~DHT22(void);
    esp_err_t read(void);
    float get_temperature(void);
    float get_humidity(void);
};

#endif /* DHT22_H */
