# ESP-CPP Implementation

- [Introduction](#introduction)
- [Quickstart](#quickstart)
- [The Strategy](#the-strategy)
- [Getting Started](#getting-started)
- [The `main` File](#the-main-file)
- [The Other Files](#the-other-files)
- [The `dht22` driver Files](#the-dht22-driver-files)
- [Building and Flashing](#building-and-flashing)
- [Observations and Next Steps](#observations-and-next-steps)

## Introduction

This third example demonstrates another ESP-IDF based approach to creating a connected sensing device. The project will use C++ to achieve the same functional outcomes as the previous C-based project. The ESP-IDF toolchain introduced in the previous project iteration has the ability to compile C++, so the environment we use will be identical to the previous example. Another difference to the previous implementation will be an investigation into the creation of a driver for the DHT22 sensor. Previously, we leveraged a driver from the ESP Component Registry, but in this project we will go about reinventing the wheel and in the process, gain some valuable insights into the low level interface to the sensor, and the way the ESP32 behaves in timing-critical applications.

The general approach taken will be to:

- Configure the `main.cpp` file to operate in much the same way as the previous example
- Explore some C++ language features while building out the functions
- Build a driver for the DHT22 Temperature and Humidity sensor

Let's get in to it.

## Quickstart

This series of projects is presented as an educational piece, however if you simply want to take some or all of the repository and use it for your own projects without suffering through the narrative you can:

0. [Acquire and assemble](https://github.com/TristanWebber/garage_monitor/blob/main/README.md#hardware) the appropriate hardware
1. Setup a [dashboard](https://github.com/TristanWebber/garage_monitor/blob/main/README.md#dashboard)
1. Ensure you have the [ESP-IDF and toolchain](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html) installed
2. Clone the repository. If you are only interested in this subproject, use `sparse-checkout`
```bash
git clone --no-checkout https://github.com/TristanWebber/garage_monitor.git && \
cd garage_monitor && \
git sparse-checkout esp_cpp && \
git checkout
```
3. [Update](#other-files) and rename the `_config.h` file
4. [Source](#mqtt) the `ca_cert.pem` file and update the relevant CMakeLists.txt files if necessary
5. [Build and flash](#building-and-flashing) the project to your microcontroller

## The strategy

A reminder on the objective. The project needs to:
- Take readings from a reed switch on a door
- Take readings from a temperature and humidity sensor
- Publish data to a cloud MQTT broker over WiFi
- Repeat periodically

This iteration will first look at what classes and methods will be needed to achieve the required functionality. Next, the DHT22 driver will be built. The first pass will be a very linear, naive approach using a single thread and blocking functions to manage timing. Then it will look at approaches to achieving the same time-critical performance without blocking. Finally, some commentary about why the inbuilt RMT component of the IDF is the superior option. Throughout, it will explore some areas where higher level C++ abstractions allow for safer, more terse or simply different code.

## Getting started

If you are cloning this repository, these steps will be unecessary. However, for completeness, the steps taken to create the project from scratch are shared [here](https://github.com/TristanWebber/garage_monitor/tree/main/esp_idf/README.md#getting-started). Steps are identical, however, change the project name to `esp_cpp`.

## The main file

The very first challenge of the project is the entry point. The ESP-IDF has C APIs so to call them from a C++ program, the compiler needs to know not to mangle names. We also know that the ESP-IDF uses the `app_main()` function as the entry point and for a C++ project this is no exception. So a way to get into C++ land as quickly as possible, we could use the following:

#### **`main.cpp`**
```cpp
static Main cpp_main;

extern "C" void app_main(void) {
    cpp_main.start();
}
```

Using `extern "C"` tells the C++ compiler not to mangle names. All our C `app_main` function does is call the `start()` method of the `Main` C++ class. So taking the lead from the previous project, the `start()` method should look like:

#### **`main.cpp`**
```cpp
Sensors sensors(Config::DOOR_SW_PIN, Config::DHT_PIN);
Wifi wifi;
Mqtt mqtt_client;

static TimerHandle_t debounce_timer_handle;

void Main::start(void) {
    sensors.init(&door_interrupt_handle);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(wifi.init());
    ESP_ERROR_CHECK(mqtt_client.client_init());

    create_tasks();
}
```

#### **`main.h`**
```cpp
#ifndef MAIN_H
#define MAIN_H

class Main final {
private:
    void create_tasks(void);
public:
    void start(void);
};

#endif /* MAIN_H */
```

This gives the very familiar structure from the previous project, with the exception being the use of classes and methods. Observe the `create_tasks()` private method - arguably there would be no issues if this implementation were to simply rely on the C API for FreeRTOS, but there is a multithreading library `esp_pthread.h` that is a C++ wrapper of FreeRTOS. Using this wrapper allows the use of C++ style to access FreeRTOS functionality.

Let's explore this by refactoring the `read_and_send_task` to a C++ thread. As a reminder, the way to create a minimal FreeRTOS task was:

```cpp
#include "freertos/Freertos.h"
#include "freertos/task.h"

static void read_and_send_task(void *pvParameters) {
    while (true) {
        // read and send
        vTaskDelay(pdMS_TO_TICKS(SEND_INTERVAL));
    }
}

void app_main(void) {
    xTaskCreate(&read_and_send_task, "read_and_send_task", 5 * 1024, NULL, 1, NULL);
}
```

This does exactly as we intend, however FreeRTOS is not familiar to all developers. For developers familiar with C++, but unfamiliar with FreeRTOS, it is possible to use the `std::thread` class and entirely ignore the semantics of FreeRTOS. For example, the `read_and_send_task` could be created as follows:

```cpp
#include <chrono>
#include <thread>

#include "esp_pthread.h"

[[noreturn]] void Main::read_and_send(void) {
    while (true) {
        // read and send
        std::this_thread::sleep_for(std::chrono::seconds {Config::SEND_INTERVAL});
    }
}

void app_main (void) {
    // Set configuration for thread
    auto cfg = esp_pthread_get_default_config();
    esp_pthread_set_cfg(&cfg);

    // Create a thread to execute the static `read_and_send` method
    std::thread read_and_send_thread(&Main::read_and_send);

    // Wait for the thread to terminate
    read_and_send_thread.join();
}
```

Hiding the implementation details of FreeRTOS for a simple task allows for standard C++ to be used, with the exception of setting the config. The default config is a struct, so it can be updated if custom values are required by the application. Also note that the FreeRTOS `vTaskDelay` and `xTaskDelayUntil` can be replaced with the methods `std::this_thread::sleep_for` and `std::this_thread::sleep_until`. We use `[[noreturn]]` to flag to the compiler that we expect the task to be an infinite loop and the `join` method ensures that the program does not leave `app_main` unless the thread finishes executing.

Unfortunately this is not so straightforward for the interrupt task, because the IDF's implementation of the C++ mutex cannot be called from a ISR function and results in the application instantly aborting. However, keeping in context that we don't actually need the interrupt to occur with millisecond precision, we can just create a crude thread to frequently check if the door state has changed. We place this in the peripheral manager to keep all the direct interactions with peripherals in one place, and hide some of the implementation details:

#### **`peripheral_manager.cpp`**
```cpp
Sensors::interrupt_handle_t Sensors::get_interrupt_handle(void) {
    return &Sensors::interrupt;
}

[[noreturn]] void Sensors::interrupt(std::atomic_flag& atomic_flag) {
    bool last_state = get_door_state();
    while (true) {
        bool current_state = get_door_state();
        atomic_flag.wait(true);
        if (current_state != last_state) {
            last_state = current_state;
            atomic_flag.test_and_set();
            atomic_flag.notify_one();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
```
And start the threads in main:

#### **`main.cpp`**
```cpp
#include <chrono>
#include <thread>

std::atomic_flag atomic_flag {};
Sensors sensors(Config::DOOR_SW_PIN, Config::DHT_PIN);

void Main::create_tasks(void) {

    Sensors::interrupt_handle_t interrupt_handle = sensors.get_interrupt_handle();

    std::thread interrupt_listen_thread(interrupt_handle, &sensors, std::ref(atomic_flag));
    std::thread interrupt_send_thread(&Main::interrupt_send);

    interrupt_listen_thread.join();
    interrupt_send_thread.join();
}

[[noreturn]] void Main::interrupt_send(void) {
    while (true) {
        atomic_flag.wait(false);

        std::this_thread::sleep_for(std::chrono::milliseconds(Config::DEBOUNCE_DURATION_MILLIS));
        bool door_status = sensors.get_door_state();

        // read and send

        atomic_flag.clear();
        atomic_flag.notify_one();
    }
}
```

Starting these two tasks allows one to listen for events, and the other to respond to the events. Using the atomic_flag provides similar behaviour to the FreeRTOS task notifications. This has allowed for direct interaction with FreeRTOS to be entirely eliminated from the main and peripheral manager files - instead replaced by C++ standard concurrency features.

## The other files

The refactoring process for other files in the project follows much the same process. There is little change to the overall function and implementation approach.

### Peripheral manager

In the peripheral manager files, the approach is to create a `Sensors` class and refactor the functions used in the C version of the project into methods. The class declaration hints at some of the changes available to us going to the C++ implementation:

#### **`peripheral_manager.h`**
```cpp
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

    esp_err_t init(TimerHandle_t *debounce_timer_handle);
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
```

The general approach here is that if we were to add multiple sensors to the microcontroller, we could manage these additional sensors with new instances of the Sensors class. The public methods are broadly the same as the functions created in the C project, with the key difference that the `Sensors` constructor allows us to pass the pins as arguments and then they can be stored in private variables, thus reducing the need to pass the pins as arguments in every function as we had to in the C version of the project.

The only callout to make about the implementation of `peripheral_manager.cpp` is the treatment of the DHT22 instances. We want to have an instance of DHT22 as a private member for each instance of Sensors. However, it's not possible to instantiate DHT22 until the pin number for the sensor is passed to the Sensors constructor method. So we declare a DHT22 pointer as a private member of the Sensors class and then allocate memory for the DHT22 instance when the relevant context becomes available:

#### **`peripheral_manager.cpp`**
```cpp
---snip---

Sensors::Sensors(gpio_num_t door_sw_pin, gpio_num_t dht_pin) {
    _door_sw_pin = door_sw_pin;
    _dht_pin = dht_pin;
    _sensor_data = {false, 0, 0};
    _dht22 = new DHT22(_dht_pin);
}

Sensors::~Sensors(void) {
    delete _dht22;
}
```

This allows a unique DHT22 instance to be dynamically allocated to each Sensors instance, and the destructor method allows the memory to be freed if the Sensors instance goes out of scope.

### Wifi and MQTT managers, Config file

Compared to the C ESP-IDF implementation, the approach in the C++ version is similar. The notable exceptions are:

- Using classes
- The use of `enum class` to store the state of the connection. This allows the application to better handle when the wifi or mqtt connections are not present
- The classes handle and return errors, rather than the approach in the previous implementation where any error resulted in a panic
- Mutexes are used to prevent race conditions

## The DHT22 driver files

Taking on the task of implementing the DHT22 driver is a good opportunity to dip in to some proper low level programming. To date, the implementations made use of drivers by others. However, doing a driver from scratch is not that challenging for a straightforward digital device, and it gives insights into how devices communicate and interoperate with each other that could otherwise go entirely unseen.

Some introductory notes on how the DHT22 communicates with the host. Timing values described here are the nominal values. Refer to the datasheet to find the typical ranges of these:
- Uses a single wire serial data (SDA) pin to transmit and receive data
- A transmission can be roughly broken into three stages
- **The host requests data** by:
    - Pulling the SDA pin low for ~1,000us, then
    - Allowing the SDA pin to float, therefore the pullup resistor results in the DHT seeing a high.
- **The DHT22 acknowledges the request** by:
    - After ~30us the DHT takes control of the SDA
    - The SDA pin is pulled low for ~80us
    - The SDA pin is pulled high for ~80us
- **The DHT22 starts the transmission of data**
    - A data transmission from the DHT22 consists of 40 bits of data
    - The bits are encoded in a series of pulses. The duration of the high pulse determines whether the bit is a `0` or a `1`
    - When the duration is ~26us, the bit is a `0` and when the duration is ~70us the bit is a `1`
    - All low pulses are ~50us. After the final pulse, the bus is released to the host.
- The sensor has a 0.5Hz refresh time, therefore the host should attempt readings no more than every 2 seconds.

To decode the data, consider it as a group of 5 bytes:

- The first two bytes encode 10x relative humidity (%) as a 16 bit, MSB first, two's complement integer
- The second two bytes encode 10x temperature (degC) as a 16 bit, MSB first, two's complement integer
- The final byte is a parity byte. It should be the sum of the previous 4 bytes. If this relationship is not preserved, some of the transmission was not accurately collected and should be discarded

**TODO - Add a shot of the packet format**

So let's jump in to the solutions.

### Blocking driver

In the first iteration of the driver, we will implement it as a blocking function. This allows us to create driver code that appears identical to the datasheet. This is an interesting exercise because it requires some consideration of how the FreeRTOS scheduler works and we will see that this approach compromises performance of other FreeRTOS tasks.

#### Pseudocode for the driver

The first step is to create a component. Details are contained in the [previous](https://github.com/TristanWebber/garage_monitor/tree/main/esp_idf#making-a-component) project so will not be repeated here. As a reminder, the `peripheral_manager` is expecting an interface that looks something like this:

#### **`dht22.h`**
```cpp
---snip---
public:
    DHT22(gpio_num_t dht_pin);
    ~DHT22(void);
    esp_err_t read(void);
    float get_temperature(void);
    float get_humidity(void);
```

So based on the knowledge from the datasheet, the pseudocode for the implementation could look something like this:

#### **`dht22.cpp`**
```cpp
DHT22::DHT22(gpio_num_t dht_pin) {
    _pin = dht_pin;

    // Set to allow the first read to occur immediately after instantiation
    _last_read_time = esp_timer_get_time() - MIN_READ_INTERVAL;

    // Input pullup will ensure the DHT sees a high signal when idle
    gpio_set_direction(_pin, GPIO_MODE_INPUT);
}

esp_err_t DHT22::read(void) {

    // If a read has happened less than 2 seconds previous, return OK to let the application know the existing reads are still valid
    if (current_time - _last_read_time < MIN_READ_INTERVAL) {
        return ESP_OK;
    }

    // Set the array storing data bits to 0s. C++ gives us a nice one-liner
    std::fill(_dht_bits.begin(), _dht_bits.end(), 0);

    // Actions by host MCU
    request_transmission();

    // Check for preamble from DHT, read bits
    read_bits();

    // Check parity byte, convert bits to floats, set private variables for temp and hum
    convert_data();
}

// Simple getter functions. Perhaps consider returning unrealistic values if there are errors
float DHT22::get_temperature(void) {
    return _temperature;
}

float DHT22::get_humidity(void) {
    return _humidity;
}
```

So far so good. This shows us most of the private variables and methods that need to be added to the class. We also get a peek at some C++ features that reduce the amount of code required and improve safety. Using a C++ array rather than a C array still allocates the memory to the stack (i.e. size is known at compile time and access is faster than heap-allocated memory) but we get some bonus methods.

```cpp
#include <array>

// Create an array of 40 uint8_t elements, all set as 0 initially
std::array<uint8_t, 40> _dht_bits{0};

// Do stuff with array
---snip---

// Return the array to 0 values
std::fill(_dht_bits.begin(), _dht_bits.end(), 0);
```

Using the `std::array::end` method gives some built in security that we will not accidentally access memory past the end of the array. It gives us a more reliable option to get the number of elements in the array than using a constant, and it is more terse than C's idiomatic approach of `sizeof(array) / sizeof(array[0])`.

#### Achieving precise timing with a blocking delay

Let's now turn our attention to `request_transmission()`. Recall from the datasheet that the host requests a transmission by pulling the SDA line to low for ~1ms and then returns to a floating value, awaiting acknowledgement from the DHT22. This sounds trivial, however if we use a `vTaskDelay`, consider what is actually happening:

- The current task will yield to the FreeRTOS scheduler
- The task's priority dictates when it will be able to resume - higher priority tasks and interrupts will take precedence
- This means a vTaskDelay is really a minimum delay. It's not appropriate for tasks requiring exact timing.

It's straightforward to solve this with a blocking delay. For example:

```cpp
#include "esp_timer.h"

static inline void blocking_delay_us(uint32_t us) {
    int64_t start = esp_timer_get_time();
    int64_t now = start;
    while (now - start < us) {
        now = esp_timer_get_time();
    }
}
```

This is a brute force approach to achieving a delay, in a threaded application but it gives us an opportunity to rapidly and intuitively explore the driver. Later we will look at working _with_ the scheduler rather than fighting it but for now, let's just make it work. Creating code to replace the `request_transmission()` placeholder:

```cpp
// Host pulls pin low for > 800us
gpio_set_direction(_pin, GPIO_MODE_OUTPUT);
gpio_set_level(_pin, 0);
blocking_delay_us(1000);

// Set pin to floating and wait for sensor to pull low
gpio_set_direction(_pin, GPIO_MODE_INPUT);
blocking_delay_us(55);

// The pulse should be low for ~80us
if (expect_pulse(0) == TIMEOUT_US) {
    return ESP_ERR_TIMEOUT;
}

// The pulse should be high for ~80us
if (expect_pulse(1) == TIMEOUT_US) {
    return ESP_ERR_TIMEOUT;
}
```

Let's ignore the details of `expect_pulse` for a moment. Feel free to try this, however I suggest taking it on face value that this will return a timeout error. And the reason for this is the performance of the `gpio_set_direction` API. If this were to be timed, it takes in the order of several milliseconds to complete. So while the MCU is waiting for the gpio to change to an input, the entire transmission has been and gone.

#### Using IRAM for faster execution

In order for this to happen rapidly, let's instead use a faster way to set the pin mode, and use IRAM (instruction ram) to store the method. The default ram location is in flash, and this is slower to access than IRAM, located on the chip itself. I can't tell you why the `gpio_config(gpio_config_t *gpio_config)` is faster than the `gpio_set_direction()`, but timing them shows that the former is a clear winner. This gives us the following method. We tell the compiler the method needs to go in IRAM simply by adding `IRAM_ATTR`:

```cpp
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
```

#### Reading the pulse duration

This improves performance to the point that it is able to be used in our implementation. Now, to move on to reading the bits. Recall that a short high pulse correlates to a `0` bit and a long high pulse correlates to a `1` bit. This implies a need to measure the duration of the high pulses. First define the `expect_pulse(bool level)` method:

```cpp
uint16_t DHT22::expect_pulse(bool level) {
    uint16_t count = 0;
    int64_t start = esp_timer_get_time();
    while (gpio_get_level(_pin) == level) {
        if (count++ >= _TIMEOUT_US) {
            return _TIMEOUT_US;
        }
        blocking_delay_us(1);
    }
    int64_t end = esp_timer_get_time();
    return (end - start) > _HIGH_PULSE_DURATION;
}
```

This is a naive implementation that checks, once every microsecond, if the pin level has changed. The duration of the pulse is stored in the count variable. If count exceeds some nominal timeout that exceeds normal behaviour from the sensor, we return the timeout. Otherwise, we return a `1` or `0` based on whether the duration of the bit was longer than the threshold to be considered a logical 1. Then all we need to do is implement a `for` loop, writing each of the 40 bits to the `_dht_bits` array. If we wanted to go all-in with a C++ approach we could use a `for_each` and lambda function instead of the C for loop but... It's ugly, not allowing us to use fewer lines of code and there's a small performance overhead so let's not. For the sake of completeness, it would look like this:

```cpp
std::for_each(_dht_bits.begin(), _dht_bits.end(), [this](uint8_t &bit) {
    bit = expect_pulse(1);
    expect_pulse(0);
});
```

#### Taking control of timing with critical sections

So running this works first time, every time? Unfortunately not... This is where the shortcomings of this intuitive approach really become clear. If this was a single threaded application, it would be robust, but because this is a FreeRTOS application, the scheduler can and will preempt the task and switch to one with a higher priority - for example, the wifi event group or the interrupt we configured for the door switch. This can be solved by temporarily disabling interrupts.

```cpp
// Create a mutex for the critical section
static portMUX_TYPE port_mutex = portMUX_INITIALIZER_UNLOCKED;

esp_err_t DHT22::read(void) {
    ---snip---

    taskENTER_CRITICAL(&port_mutex);
    esp_err_t status = read_bits();
    taskEXIT_CRITICAL(&port_mutex);

    ---snip---
}
```

Ok, finally the driver will reliably read the bits from the sensor without interference from the scheduler. Let's turn the data into something human-readable.

#### Converting the data from bits to floats

We start by converting the bit array to a byte array. Arguably, this step could have been performed in the critical section, and would likely have had no negative impact on timing but best practice dictates that as little as possible should be done in the critical sections.

This is the first time in this project we're exposed to genuine low level operations. We can `bit bash` our way to the solution. Say we want to take our array of 40 bits and place it in to an array of 5 MSB-first bytes. Iterating through the bit array, dedcribing positions with 0-based indices:

- the 0th bit becomes the 7th bit of the 0th byte
- the 1th bit becomes the 6th bit of the 0th byte
...
- the 38th bit becomes the 1th bit of the 4th byte
- the 39th bit becomes the 0th bit of the 4th byte

It would be prudent to check our data is in fact a valid bit, or if a timeout value is detected, return an error. When we confirm a bit is good data, it's time to process it. The byte index is simply floor division of the bit array index `i / 8`. The bit index needs to start at 7, reduce to 0, return to 7 then repeat. We can do this using the modulo (remainder) operator `7 - (i % 8)`. Then we can set the relevant bit on the relevant byte using the bitwise left shift operator `<<` and the bitwise OR operator `|`. This approach results in the bytes being populated bit by bit:

```cpp
// Bit bash to bytes
for (uint8_t i = 0; i < _DATA_BITS; i++) {
    if (_dht_bits[i] > 1) {
        ESP_LOGE(TAG, "Invalid data. Timeout occurred during collection.");
        return ESP_ERR_TIMEOUT;
    }
    uint8_t byte_index = i / 8;
    uint8_t bit_index = 7 - (i % 8);
    _dht_bytes[byte_index] |= _dht_bits[i] << bit_index;
}
```

Then confirm the parity bit is correct. This is reasonably straightforward, although we need to force the sum of four `uint8_t` bytes to overflow by casting the sum to `uint8_t`. Omitting this step would result in our equality check failing for some values.

```cpp
// Check parity bit
if (_dht_bytes[4] != (uint8_t)(_dht_bytes[0] + _dht_bytes[1] + _dht_bytes[2] + _dht_bytes[3])) {
    ESP_LOGE(TAG, "Invalid data collected. Failed parity check.");
    return ESP_ERR_INVALID_CRC;
}
```

Finally, we convert the bytes to floats. The quick answer to this problem is understanding that the data is already in the format of a MSB first, two's complement signed integer and is 10 times the actual value. Therefore all that needs to be done is to cast the relevant bytes to a `int16_t` and divide by 10. The properties of the types will implicitly manage the signedness:

```cpp
int16_t i_temp = (_dht_bytes[2] << 8) | _dht_bytes[3];
float f_temp = temp / 10.0;
```

But since we're already in a bit bashing mood let's look at managing the signedness explicitly. The datasheet tells us the 7th bit of the 2th byte for temperature is the sign bit. So using a bitmask `0x7F` (i.e. `0111 1111` in binary) and bitwise `&` lets us initially ignore the sign bit when left shifting this first byte to the most significant byte of the 16 bit result. Similarly, a bitmask of `0x80` (i.e. `1000 0000` in binary), or the inbuild macro `BIT(7)` allows us to check _only_ the sign bit. It would look like this:

```cpp
int16_t int_temperature = (0x7F & _dht_bytes[2] << 8) + _dht_bytes[3];
bool temperature_neg = _dht_bytes[2] & BIT(7);
if (temperature_neg) {
    int_temperature = -int_temperature;
}
```

It's great to know that this is how the types work under the hood, however the implicit cast is neater and arguably less error prone in implementation. Finally, our approach of counting every us in `expect_pulse` is perhaps unneccessary. Our logic only needs to detect if a pulse is ~26us or ~70us. So we try using 15us. And now our blocking driver works.

Nice! We made it work, but in doing so, other tasks were delayed for several milliseconds even though most of the time spent in this driver is sitting in the blocking delays.

### Timer callback driver

The brute force approach used in the blocking driver was implemented to fight the scheduler and actively prevent other tasks from accessing resources. However, we can instead set the delays up in a way they _our_ task is the one that creates interrupts and preempts other tasks. One approach could be to use the FreeRTOS `xTimer` like what was used for the debounce timer for the door switch, however the FreeRTOS timer resolution may not suit our particular needs in this application. Let's instead use the ESP High Resolution Timer available in the `esp_timer.h` header file.

The approach described here is based on the [esp-microsleep](https://github.com/mickeyl/esp-microsleep) library.

The [documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/esp_timer.html#callback-dispatch-methods) tells us that the Interrupt Dispatch method is likely the best for our application because it is low latency and is not affected by other active tasks. This requires it to be enabled in the `menuconfig` by checking the setting under menus `Component config -> High resolution timer (esp_timer) -> Support ISR dispatch method`.

With this configuration set, the general approach to setting a one shot high resolution timer is to:

- Create a `IRAM_ATTR` callback function performing the tasks you wish to occur once the timer period elapses
- Configure the timer arguments, passing it a pointer to the callback function
- Start the timer

In this case, the desired behaviour is that after calling the timer, the application waits until the timer expires. So using FreeRTOS task notifications is the the ideal approach. This simplest possible implementation would look like:

```cpp
static void IRAM_ATTR us_delay_isr_handler(void* arg) {
    TaskHandle_t task = (TaskHandle_t)(arg);
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(task, &higherPriorityTaskWoken);
    esp_timer_isr_dispatch_need_yield();
}

static void us_delay(uint64_t us) {

    esp_timer_handle_t delay;

    const esp_timer_create_args_t oneshot_timer_args = {
        .callback = us_delay_isr_handler,
        .arg = (void*) xTaskGetCurrentTaskHandle(),
        .dispatch_method = ESP_TIMER_ISR,
    };

    ESP_ERROR_CHECK(esp_timer_create(&oneshot_timer_args, &delay));
    ESP_ERROR_CHECK(esp_timer_start_once(delay, us));
    xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
}
```

This basic structure gets some window-dressing to give the timer a little more context about its environment. As has been the case throughout this exercise, there are some challenges related to timing to be aware of. Starting the timer and executing the callback takes some time, so this information should also be passed to the timer to correct for the known time required for processing. For my ESPC3, this was approximately 10us when the task was run in isolation but up to 200us when the WiFi and MQTT event loops were running. This signals that this approach is not practical for a single-core ESP. However, for a dual core, it's far easier for us to escape the goings on of the radio transceiver.

### Remote Control Transceiver driver

A final note on the driver design. This process has been an educational adventure into building a driver from first principles using the documentation, and demonstrating the hard way that there are other factors to be aware of. The idiomatic approach to a driver like this using ESP-IDF is to use the Remote Control Transceiver (RMT) API.

The DHT communications protocol is basically identical to the NEC IR protocol, so whilst we are not dealing with reading and transmitting to a infra-red LED, the ay the bits are encoded is the same. The external component we used in the C ESP-IDF version of this project is based on the RMT API. So the lesson here is that reinventing the wheel is educational, but chances are you will end up with a lesser wheel...

## Building and flashing

Commands to build, flash and monitor the project are as per the previous project [here](https://github.com/TristanWebber/garage_monitor/tree/main/esp_idf/README.md#building_and_flashing).

## Observations and Next Steps

This implementation has shown that some facets of C++ are easily used with ESP-IDF. This can deliver benefits in the form of higher level features and data structures and allows for better control of interfaces, particularly where a project may use multiple instances of a sensor. The exercise of creating a driver for the DHT22 sensor allowed for a practical investigation into how to communicate with a digital device. It also gave a deep dive in to the realities of how FreeRTOS interacts with tasks.

The next subproject will explore using Rust with the ESP32, via the Standard Library.

Check it out [here](https://github.com/TristanWebber/garage_monitor/tree/main/rs_std)
