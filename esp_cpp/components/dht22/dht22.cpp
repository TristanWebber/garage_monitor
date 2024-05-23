#include <algorithm>

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#include "dht22.h"

static const char *TAG = "DHT22";

#if !defined(CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD)
    ESP_LOGE(TAG, "CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD must be defined in menuconfig");
    vTaskDelay(pdMS_TO_TICKS(1000));
    assert(0);
#endif

static inline void blocking_delay_us(uint32_t us) {
    int64_t start = esp_timer_get_time();
    int64_t now = start;
    while (now - start < us) {
        now = esp_timer_get_time();
    }
}

DHT22::DHT22(gpio_num_t dht_pin) {
    esp_log_level_set("gpio", ESP_LOG_NONE);
    _pin = dht_pin;
    _last_read_time = esp_timer_get_time() - _DHT22_MIN_INTERVAL;
    gpio_pin_mode(GPIO_MODE_INPUT);
    us_delay_calibrate();
}

DHT22::~DHT22() {
    gpio_set_direction(_pin, GPIO_MODE_DISABLE);

    // Stop and delete the timer
    esp_timer_handle_t timer = (esp_timer_handle_t) pvTaskGetThreadLocalStoragePointer(NULL, 1);
    if (timer) {
        esp_timer_stop(timer);
        esp_timer_delete(timer);
    }
}

esp_err_t DHT22::read() {
    // Only update data if more than 2sec has elapsed since last read
    int64_t current_time = esp_timer_get_time();
    if (current_time - _last_read_time < _DHT22_MIN_INTERVAL) {
        return ESP_OK;
    }

    us_delay_calibrate();

    std::fill(_dht_bits.begin(), _dht_bits.end(), 0);

    // MCU pulls pin low for > 1ms
    gpio_pin_mode(GPIO_MODE_OUTPUT);
    gpio_set_level(_pin, 0);
    us_delay(1000);

    // Waiting for the sensor to pull the data line low
    gpio_pin_mode(GPIO_MODE_INPUT);
    us_delay(55);

    esp_err_t status = read_bits();

    if (status != ESP_OK) {
        ESP_LOGE(TAG, "Timed out waiting for response from sensor");
        return ESP_ERR_TIMEOUT;
    }
    // check results
    status =  convert_data();

    return status;
}

float DHT22::get_temperature() {
    return _temperature;
}

float DHT22::get_humidity() {
    return _humidity;
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
        us_delay(20);
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

    return ESP_OK;
}

static void IRAM_ATTR us_delay_isr_handler(void* arg) {
    TaskHandle_t task = (TaskHandle_t)(arg);
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(task, &higherPriorityTaskWoken);
    esp_timer_isr_dispatch_need_yield();
}

void DHT22::us_delay(uint64_t us_delay) {

    esp_timer_handle_t timer = (esp_timer_handle_t) pvTaskGetThreadLocalStoragePointer(NULL, 1);
    if (!timer) {
        const esp_timer_create_args_t oneshot_timer_args = {
            .callback = us_delay_isr_handler,
            .arg = (void*) xTaskGetCurrentTaskHandle(),
            .dispatch_method = ESP_TIMER_ISR,
        };
        ESP_ERROR_CHECK(esp_timer_create(&oneshot_timer_args, &timer));
        vTaskSetThreadLocalStoragePointer(NULL, 1, (void*) timer);
    }

    if (us_delay == 0) { return; }

    if (us_delay <= _us_delay_compensation) {
        blocking_delay_us(us_delay);
        return;
    }

    ESP_ERROR_CHECK(esp_timer_start_once(timer, us_delay - _us_delay_compensation));
    xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
}

uint64_t DHT22::us_delay_calibrate() {

    const int calibration_loops = 10;
    const uint64_t calibration_usec = 100;
    uint64_t compensation = 0;

    us_delay(0); // to preheat the timer for this task
    for (int i = 0; i < calibration_loops; i++) {
        uint64_t start = esp_timer_get_time();
        us_delay(calibration_usec);
        uint64_t diff = esp_timer_get_time() - start - calibration_usec;
        compensation += diff;
    }
    _us_delay_compensation = compensation / calibration_loops;
    return _us_delay_compensation;
}
