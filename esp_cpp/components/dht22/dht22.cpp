#include <algorithm>
#include <math.h>

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#include "dht22.h"

static const char *TAG = "DHT22";
static portMUX_TYPE port_mutex =  portMUX_INITIALIZER_UNLOCKED;

static inline void blocking_delay_us(uint32_t us) {
    int64_t start = esp_timer_get_time();
    int64_t now = start;
    while (now - start < us) {
        now = esp_timer_get_time();
    }
}

void IRAM_ATTR DHT22::gpio_pin_mode(gpio_mode_t mode) {
    gpio_config_t conf = {
        .pin_bit_mask = (1ULL << _pin),
        .mode = mode,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&conf);
}

DHT22::DHT22(gpio_num_t dht_pin) {
    esp_log_level_set("gpio", ESP_LOG_NONE);
    _pin = dht_pin;
    _last_read_time = esp_timer_get_time() - _DHT22_MIN_INTERVAL;
    gpio_pin_mode(GPIO_MODE_INPUT);
}

DHT22::~DHT22(void) {
    gpio_set_direction(_pin, GPIO_MODE_DISABLE);
}

esp_err_t DHT22::read(void) {
    // Only update data if more than 2sec has elapsed since last read
    int64_t current_time = esp_timer_get_time();
    if (current_time - _last_read_time < _DHT22_MIN_INTERVAL) {
        return _real_data ? ESP_OK : ESP_ERR_TIMEOUT;
    }

    _real_data = false;
    std::fill(_dht_bits.begin(), _dht_bits.end(), 0);

    // MCU pulls pin low for > 1ms
    gpio_pin_mode(GPIO_MODE_OUTPUT);
    gpio_set_level(_pin, 0);
    blocking_delay_us(1100);

    // Waiting for the sensor to pull the data line low
    gpio_pin_mode(GPIO_MODE_INPUT);
    blocking_delay_us(55);

    taskENTER_CRITICAL(&port_mutex);
    esp_err_t status = read_bits();
    taskEXIT_CRITICAL(&port_mutex);

    if (status != ESP_OK) {
        ESP_LOGE(TAG, "Timed out waiting for response from sensor");
        return ESP_ERR_TIMEOUT;
    }
    // check results
    status =  convert_data();

    return status;
}

esp_err_t DHT22::read_bits(void) {

    if (expect_pulse(0) == _TIMEOUT_US) {
        return ESP_ERR_TIMEOUT;
    }

    if (expect_pulse(1) == _TIMEOUT_US) {
        return ESP_ERR_TIMEOUT;
    }

    // Read in each of the 40 bits of data...
    expect_pulse(0);
    for (int i = 0; i < _DATA_BITS; i++) {
        _dht_bits[i] = expect_pulse(1);
        expect_pulse(0);
    }

    return ESP_OK;
}

uint16_t DHT22::expect_pulse(bool level) {
    uint16_t count = 0;
    int64_t start = esp_timer_get_time();
    while (gpio_get_level(_pin) == level) {
        if (count++ >= _TIMEOUT_US) {
            return _TIMEOUT_US;
        }
        blocking_delay_us(15);
    }
    int64_t end = esp_timer_get_time();
    return (end - start) > _HIGH_PULSE_DURATION;
}

esp_err_t DHT22::convert_data(void) {

    // Clear the array
    std::fill(_dht_bytes.begin(), _dht_bytes.end(), 0);

    for (uint8_t i = 0; i < _DATA_BITS; i++) {
        if (_dht_bits[i] > 1) {
            ESP_LOGE(TAG, "Invalid data. Timeout occurred during collection.");
            return ESP_ERR_TIMEOUT;
        }
        uint8_t byte_index = i / 8;
        uint8_t bit_index = 7 - (i % 8);
        _dht_bytes[byte_index] |= _dht_bits[i] << bit_index;
    }

    if (_dht_bytes[4] != (uint8_t)(_dht_bytes[0] + _dht_bytes[1] + _dht_bytes[2] + _dht_bytes[3])) {
        ESP_LOGE(TAG, "Invalid data collected. Failed parity check.");
        return ESP_ERR_INVALID_CRC;
    }

    int16_t int_temperature = (_dht_bytes[2] << 8) + _dht_bytes[3];
    int16_t int_humidity = (_dht_bytes[0] << 8) + _dht_bytes[1];

    _temperature = int_temperature / 10.0;
    _humidity = int_humidity / 10.0;

    _real_data = true;
    return ESP_OK;
}

float DHT22::get_temperature(void) {
    return _real_data ? _temperature : MAXFLOAT;
}

float DHT22::get_humidity(void) {
    return _real_data ? _humidity : MAXFLOAT;
}
