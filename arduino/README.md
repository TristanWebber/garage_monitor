# Arduino Implementation

- [Introduction](#introduction)
- [Quickstart](#quickstart)
- [The Strategy](#the-strategy)
- [Getting Started](#getting-started)
- [The `main` File](#the-main-file)
- [The `peripheral_manager` Files](#the-peripheral_manager-files)
- [The `connection_manager` Files](#the-connection_manager-files)
- [Other Files](#other-files)
- [Building and Flashing](#building-and-flashing)
- [Observations and Next Steps](#observations-and-next-steps)

## Introduction

This first example of the garage monitor leverages relatively high level abstractions and libraries. This way it's possible to use very little code to achieve the desired functionality. It intentionally avoids lower level details where possible so we make a number of comprimises which will be discussed in [Observations](#observations-and-next-steps).

A quick refresher on Arduino abstraction - Every Arduino project will call `setup()` and `loop()` functions. For the purposes of this first project, how they are called and what they actually are will be ignored. We'll just pretend it is some magic we can rely on and that allows us to rapidly build a basic project.

Our strategy for this project is to rely on those two Arduino entry points, use libraries written in Arduino style for our peripherals, and run everything sequentially, thus not worrying too much about how to handle some event that may occur while tasks are 'blocking'. It is a path of least resistance for implementation and results in relinquishing control over 'how' things happen, but that isn't a bad thing, particularly if the focus is simply on testing out an idea and making something work without needing to spend days reading documentation.

The project has been developed using PlatformIO for build tools. This makes downloading libraries and managing dependencies for independent projects straightforward. For anyone following along, initial setup of PlatformIO takes more effort than Arduino IDE but for anything other than tiny projects I have by far preferred PlatformIO. There's no reason the project wouldn't compile in Arduino IDE but this walkalong is written from the context of PlatformIO.

## Quickstart

This series of projects is presented as an educational piece, however if you simply want to take some or all of the repository and use it for your own projects without suffering through the narrative you can:

0. [Acquire and assemble](https://github.com/TristanWebber/garage_monitor/blob/main/README.md#hardware) the appropriate hardware
1. Setup a [dashboard](https://github.com/TristanWebber/garage_monitor/blob/main/README.md#dashboard)
1. Ensure you have an appropriate build tool like [PlatformIO](https://platformio.org/install) or [Arduino](https://support.arduino.cc/hc/en-us/articles/360019833020-Download-and-install-Arduino-IDE)
2. Clone the repository. If you are only interested in this subproject, use `sparse-checkout`
```bash
git clone --no-checkout https://github.com/TristanWebber/garage_monitor.git && \
cd garage_monitor && \
git sparse-checkout arduino && \
git checkout
```
3. [Update](#other-files) and rename the `_config.h` file
4. [Update](#wifi) and rename the `_ca_cert.h` file
5. [Build and flash](#building-and-flashing) the project to your microcontroller

## The strategy

A reminder on the objective. The project needs to:
- Take readings from a reed switch on a door
- Take readings from a temperature and humidity sensor
- Publish data to a cloud MQTT broker over WiFi
- Repeat periodically

This very first subproject aims to create a fully functioning and moderately robust tool in a way that is easy to follow and, where available, will use abstractions created by the open source community to allow us to ignore what the hardware is doing. We can just focus on describing the problem in pseudocode and then building out the actual code. In the process, we'll discover that the Arduino tools and libraries allow us to create a functioning project with code that looks remarkably similar to the pseudocode.

Whilst it would be possible to assemble everything into a single `.ino` file, I have split functions out into multiple files to keep things more readable (in my opinion...) and to make it easy to hide sensitive information in a `config.h` file.

## Getting started

Of course, this repository exists and none of these steps are necessary if you simply want to use this repository but if you want to walk along with the entire process, here's how. Assuming you already have PlatformIO installed, create a new project for the `Seeed Studio XIAO ESP32C3` board. You can use whatever ESP32 you like, but pins used in this project are based on this particular board. If using a UI, follow the prompts, or if using the CLI:

```bash
mkdir garage_monitor_arduino
cd garage_monitor_arduino
pio project init --board seeed_xiao_esp32c3
```

You'll now have an empty PlatformIO project and the necessary toolchain. If this is your first project in PlatformIO with and ESP32C3 using the Arduino framework, it may take some time for PlatformIO to download the toolchain.

## The main file

Reiterating that the primary purpose of this subproject is to use an Arduino framework to hide a lot of the implementation detail, the `main.cpp` file will be kept minimal and should clearly describe what we want the code to do. Recall that the `setup()` and `loop()` functions are the entry point for a project using the Arduino framework. In the setup, it is logical to place the functions we only need to call once, and in the loop it is logical to group the things we want to occur repeatedly.

In pseudocode, the `main` file will do something like this:

```C
// Do this once
void setup() {
    sensor_init();
    wifi_connect();
    mqtt_connect();
}

// Do this repeatedly
void loop() {

    // Stay connected to WiFi network and MQTT broker
    connection_handler();

    // Take readings and publish them to the MQTT broker
    sensor_read();
    mqtt_publish();
}
```

Sure, this looks naive, but this is almost all of the tasks the program needs to do! Each of the proposed functions will be fleshed out in the subsequent paragraphs. The only other thing that needs to be done here is to implement some way to make the loop function only read and send periodically. Say, every 5 minutes. It's possible to achieve this with an `if` block. The logic can be something like **if** _5 minutes has elapsed since the last transmission_ **OR** _the microcontroller has just started up_ **then** _read and send_. The Arduino `millis()` function returns the count of milliseconds since the processor started so all of the above can be achieved by simply keeping track of the time the last send occurred at:

```C
const uint32_t PUBLISH_INTERVAL_MILLIS = 5 * 60 * 1000;
uint32_t last_send = 0;
bool first_send = true;

void loop() {

    ---snip---

    bool send_now = (millis() - last_send >= PUBLISH_INTERVAL_MILLIS) || first_send

    if (send_now) {

        sensor_read();
        mqtt_publish();

        first_send = false;
        last_send = millis();
    }
}
```

It's wise to wrap the `connection_handler()` in a similar block but otherwise we're done here.

The final thought for the loop function is to address how it will behave if it is running for a very long time. The `millis()` function is a 32-bit unsigned integer and that means that it will reach a maximum value in about 49 days. After that, the predictable behaviour will be to wraparound to zero and restart counting from there. Similarly, the predictable behaviour of the microcontroller when it is asked to evaluate a subtraction of two unsigned integers that would result in a negative result, will be to wraparound. Using 8-bit unsigned integers and some arbitrary values makes the behaviour far easier to show. The maximum value for an 8-bit unsigned integer is 255, and 255 + 1 = 0. So working through two examples:

```
PUBLISH_INTERVAL = 100
last_send = 150

1. Normal behaviour

millis() = 200
millis() - last_send = 50

Therefore, the condition evaluates to false as expected. Less than 100ms has elapsed since the last send.

2. Overflow behaviour

Say 300 ms has elapsed since first start. The millis() overflowed beyond 255 and wrapped to 44:
millis() = 255 + 1 + 44 = 0 + 44
millis() - last_send = 44 - 150 = 0 - 1 - 105 = 255 - 105 = 150

Therefore the condition evaluates to true as expected. More than 100ms has elapsed since the last send.
```

This is behaviour is well defined in C and in some contexts, it could bite us. But in this case, wraparound is serving us perfectly well.

In the subsequent sections, the actual functions definitions can be explored.

## The peripheral_manager files

The `sensor_init()` and `sensor_read()` functions concern the configuration and usage of peripherals outside of the microcontroller itself so there is some sense in grouping them together in a file. In terms of what needs to be achieved, recall that the solution is using two different sensors - a reed switch, and a combined temperature and humidity sensor.

### Initiating the interface to sensors

The door sensor is a simple reed switch. It is normally open, and closes when a magnet is nearby. So it has two states - open circuit or closed circuit. In microcontroller language it behaves as a digital input. Say we configure the installation so that the magnet is in proximity of the switch when the door is closed, and say we want a logical `high` in this state. It follows that we will want a logical `low` when the switch is open circuit. This requires the use of a pulldown resistor - otherwise our open circuit will result in a floating value. This is all the information we need to configure the digital inpu. To init the pin, simply use the Arduino `pinMode` function thus: `pinMode(DOOR_SW_PIN, INPUT_PULLDOWN);`.

The DHT22 combined temperature and humidity sensor is more challenging at face value. The spec sheets identify that the sensor incorporates an 8-bit ADC, so to get the values we would need to read from the relevant registers and then figure out how to convert that 8 bit digital value into a float.

Fortunately, libraries are available so it's possible ignore all of that! The `DHT sensor library` looks like it'll work nicely. It looks like by using this library it is possible to instantiate, initiate and read both sensors with five lines of code, drawing on the very familiar Arduino style. It's looking like an easy job, so the most challenging part of interfacing to this sensor will be getting the libraries.

The documentation tells us there is a dependency on the `Adafruit Unified Sensor` library so install that as well. And while on the topic of libraries, jump one step ahead and get the `PubSubClient` library for MQTT, which is needed for the `connection_manager` later. One way to install the appropriate libraries is by selecting them from the PlatformIO library manager UI (if you are using a IDE plugin), or add lines to botton of the `platformio.ini` file. The latter is far quicker because the dependencies will automatically download when you next build the project:

```
---snip---
lib_deps = 
    knolleary/PubSubClient@^2.8
    adafruit/Adafruit Unified Sensor@^1.1.14
    adafruit/DHT sensor library@^1.4.6
```

And with that, our `sensor_init()` function can be written:

```C
#include "DHT.h"

DHT dht(DHT_PIN, DHY_TYPE);

void sensor_init() {
    pinMode(DOOR_SW_PIN, INPUT_PULLDOWN);
    dht.begin();
}
```

### Reading the sensors

Let's say we double down on the function thought up in the pseudocode, `sensor_read()`. There are three pieces of data to read, so grouping them into a `struct` and passing by reference becomes a tidy way to keep the single function idea, get our three pieces of data and use a single argument when calling the function.  The struct looks like this:

```C
typedef struct SensorData
{
    bool door_status;
    float temperature;
    float humidity;
} SensorData;
```

The Arduino framework allows for very straightforward reading of the digital input connected to the reed switch and the DHT library makes the reading of the temperature and humidity equally easy. But on closer inspection, it is possible for the reading of the DHT sensor to fail. There's an opportunity to think about how to manage this scenario if it occurs. We want our implementation to be robust and to manage anything that happens without user intervention. By consulting the library and the documentation for the device:

- If the read fails, the function returns `nan`
- The device has a refresh rate of 0.5Hz

So assuming the device is working correctly, if the return value is `nan`, try again, but wait 2 seconds between each attempt. We also need to consider how to manage repeat failures and how many times we attempt to read from the sensor. Perhaps it is best to give up if the sensor actually fails completely. The project's primary purpose is to allow for the garage door status to be read, so it can be concluded that it would be unacceptable for the temperature / humidity function to compromise that primary intent. One approach is to use a `do-while` loop. The code in a do-while loop will execute at least once, and some conditionals can be added so the sensor is tested a maximum of 5 times, with a delay of 2 seconds between attempts to respect the 0.5Hz refresh rate. So the whole function now looks like this:

```C
bool sensor_read(SensorData *sensor_data) {

    // Switch is normally open. A high read means the door is closed
    sensor_data->door_status = digitalRead(DOOR_SW_PIN);

    // Take temperature and humidity reads (units: celsius, %)
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

    return read_success;
}
```

This demonstrates one of the most valuable aspects of using the Arduino framework and libraries. All thought to the nuances of the code can be directed into the logic of the program and very little thought needs to be put into the inner workings of the hardware.

## The connection_manager files

The functions for the connection management are easily derived from examples provided by the libraries themselves. There are minor exceptions to this, mainly in the application-specific approach to connection failures.

### WiFi

The majority of the WiFi code is boiler plate from ESP32 Arduino library examples. The exception to this is:

- To communicate with the MQTT broker securely, TLS is required. This is achieved by:
    - Using the `WiFiClientSecure` library
    - Setting a CA server certificate with the `setCACert` method (some brokers may require a client certificate and private key as well)
- If the connection fails after many attempts, restart the device
    - This implementation allows up to 10 attempts

Your broker may have a well established way to get certificates. But for a cloud-based broker, you can also use openSSL on the command line:

```bash
openssl s_client -connect your-broker.com:8883 -showcerts < /dev/null 2> /dev/null | openssl x509
```

Then the certificate can be copy/pasted into the `_ca_cert.h` file, after renaming by dropping the leading `_`. The template header file is named this way because the `.gitignore` is set to ignore the certificate file to prevent accidental sharing of potentially sensitive information.

### MQTT

This project is using the `PubSubClient` library. Like the WiFi, the majority of the MQTT code has significant similarities to the library examples. The only real difference is that there is a `connection_handler()` function implemented. The purpose of this is to keep the connection to the MQTT broker by:

- Making sure WiFi is still connected
- Making sure the connection to the MQTT broker is still active
- Pinging the MQTT broker

This needs to be done at least as frequently as the `MQTT keep alive` interval. For this project, the keep alive interval is 15 seconds, so we elect to call the connection handler every 10 seconds.

### Bringing it together

The requirement to reconnect in the event one of the services disconnects introduces a slight complexity for our proposed original functions `wifi_connect()` and `mqtt_connect()`. It turns out that parts of connection process only needs to be done once, so adding `wifi_init()` and `mqtt_init()` allows for the `..._connect()` functions to be used and reused as required for the original connection and any subsequent reconnection attempts.

## Other files

Anywhere there is a constant, or some configuration required for the implementation, I have opted to define in the `config.h` file. This provides a convenient central location where all device settings (pin allocations, wifi and mqtt credentials, intervals between reads etc.) can be stored. This approach allows for those to be changed in a single location, and makes it straightforward to avoid sharing sensitive information because it's the only one location that needs to be excluded from a repository.

In the repository, a sanitised `_config.h` is included as a template. If you intend to use this for your own project, it will need to be renamed by dropping the leading `_` and the contents updated to ensure that the WiFi and MQTT credentials are set for the specific application. The template header file is named this way because the `.gitignore` is set to ignore the certificate file to prevent accidental sharing of potentially sensitive information.

The `debug_print.h` file is a collection of macros allowing a straightforward approach to managing whether or not the microcontroller sends diagnostics to the serial interface. The reason for this choice is that the debug messages are useful in development, but in production, the device will not be connected to a computer, therefore debug messages to the serial are useless.

## Building and flashing

Commands to build, flash and monitor the project are as follows:

```bash
# build
pio run -e seeed_xiao_esp32c3

# build and flash
pio run -e seeed_xiao_esp32c3 -t upload --upload-port /dev/ttyACM0

# monitor
pio device monitor -e seeed_xiao_esp32c3 -b 115200 -p /dev/ttyACM0
```

Your port may be different. Arguments can be combined, however for the first compile I prefer to separate them so if a step fails, it's easier to isolate where the problem occurred.

You can eliminate the need to specify most of these arguments by placing additional information in your `platformio.ini` file. For example, adding `monitor_speed = 115200` and `monitor_port = /dev/ttyACM0` to the environment configuration will eliminate the requirement for these arguments when using monitor command.

## Observations and Next Steps

The fact that this written description of the decisions made in creating the firmware took longer than actually creating the firmware should be some indication of how straightforward developing in the Arduino framework is for a simple use case such as this one. That gets listed as a very strong 'pro' for the Arduino framework. Another pro is that this works and is robust. This will happily sit for months and years, transmitting data and gracefully managing failures to the connection or sensors.

But how about some 'cons'? One of the biggest cons is that we take the third party implementations at face value. We don't really know what is happening under the hood without going very deep. This becomes an issue if we run into conflicts or interactions between a library and our particular application. These conflicts can result in peripherals not working properly and when this happens, you can waste more time trying to make the libraries work than you would implementing your own solution from bottom up. Another perhaps more serious con is that our tasks are executed in a single thread. The code currently being executed blocks everything else wrote by us and this creates a challenging situation for timing. For example, if the `connection_manager()` function discovers the WiFi or MQTT broker is disconnected, no sensor readings will take place until that function has finished executing.

Another flaw in this implementation is that the read interval dictates the program flow. Going back to the original problem, we wanted transparency on whether a door was open or closed. So a better implementation would send data when that state changed.

This leads nicely into our next subproject - Implementing the device using the ESP-IDF framework and FreeRTOS to manage tasks. In the next project, getting involved in lower level control will allow for some of the above shortfalls in the Arduino implementation to be resolved. This will come at the expense of the implementation being slightly more challenging and a little more abstract in nature.

Check it out [here](https://github.com/TristanWebber/garage_monitor/tree/main/esp_idf)
