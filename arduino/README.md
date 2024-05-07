# Arduino Implementation

This first example of the garage monitor is the most straightforward. This implementation will focus on using Arduino abstractions, intentionally avoiding lower level details where possible. This means it will use the two Arduino functions `setup()` and `loop()`, libraries written in Arduino style, and the ESP-IDF and FreeRTOS APIs will be ignored.

The project has been authored using PlatformIO because this makes downloading libraries and managing dependencies for independent projects straightforward. For anyone following along, initial setup takes more effort than Arduino IDE but for anything other than tiny projects I have by far preferred PlatformIO. There's no reason the project wouldn't compile in Arduino IDE but this walkalong is written from the context of PlatformIO.

## Quickstart

This series of projects is presented as an educational piece, however if you simply want to take the repository and use it for your own projects:
0. Acquire and assemble the appropriate hardware TODO: link
1. Ensure you have an appropriate build tool like PlatformIO or Arduino TODO: link
2. Clone the repository
3. Update the `_config.h` file TODO: link
4. Update the `_ca_cert.h` file TODO: link
5. Build and flash TODO: link

## The strategy

A reminder on the objective:
- Take readings from a reed switch on a door
- Take readings from a temperature and humidity sensor
- Publish data to a cloud MQTT broker over WiFi
- Repeat periodically

To keep the project clean and easy to follow, it will be structured with the main file containing functions that read like pseudocode. The declarations/definitions will be placed in separate files. Secrets and configurations will be placed in a separate file.

## Getting started

Of course, this repository exists and none of these steps are necessary if you simply want to use this repository but if you want to walk along with the entire process, here's how. Assuming you already have PlatformIO installed, create a new project for the `Seeed Studio XIAO ESP32C3` board. You can use whatever ESP32 you like, but pins used in this project are based on this particular board. If using a ui, follow the prompts, or if using the CLI:

```bash
mkdir garage_monitor_arduino
cd garage_monitor_arduino
pio project init --board seeed_xiao_esp32c3
```

You'll now have an empty PlatformIO project and the necessary toolchain.

## The main file

One of the primary purposes of this subproject is to show how Arduino projects hide a lot of the implementation detail, and allows a user to very quickly see a project come into existence with something that looks a lot like pseudocode. So that's a great place to start. In pseudocode, the `main` file will do something like this:

```C
# Do this once
void setup() {
    sensor_init();
    wifi_connect();
    mqtt_connect();
}

# Do this repeatedly
void loop() {
    sensor_read();
    publish_read();
}
```

The actual implementation will contain slightly more complexity in the main file, but not by much! The sensor function declarations can be grouped together, as can the connectivity functions.

## The peripheral_manager files

### Initiating and reading the door sensor

The door sensor is a simple reed switch. It is normally open, and closes when a magnet is nearby. To take a digital read and get a logical `high` when the door is closed, wire the switch to a digital input and a voltage source. To init the pin, simply use the Arduino `pinMode` function.

```C
pinMode(DOOR_SW_PIN, INPUT_PULLDOWN);
```

The function expects us to pass a pin, and a mode. The `INPUT_PULLDOWN` mode tells the microcontroller that the pin is an input and an internal pulldown resistor should be used so that a floating pin is interpreted as logical `low`. Then to read, the Arduino `digitalRead(DOOR_SW_PIN);` function does the job.

### Initiating and reading the temperature/humidity sensor

The DHT22 combined temperature and humidity sensor is more challenging at face value. The spec sheets identify that the sensor incorporates an 8-bit ADC, so to get the values we would need to read from the relevant registers and then figure out how to convert that 8 bit digital value into a float.

Fortunately, libraries are available so it's possible ignore all of that! The `DHT sensor library` looks like it'll work nicely. It looks like by using this library it is possible to instantiate, initiate and read both sensors with five lines of code, drawing on the very familiar Arduino style:

```C
#include "DHT.h"

DHT dht(DHT_PIN, DHY_TYPE);

dht.begin();

dht.readTemperature();
dht.readHumidity();
```

The documentation tells us there is a dependency on the `Adafruit Unified Sensor` library so install that as well. And while on the topic of libraries, jump one step ahead and get the `PubSubClient` library for MQTT, which is needed for the `connection_manager` later. One way to install the appropriate libraries is by selecting them from the PlatformIO library manager UI (if you are using a IDE plugin), or add lines to botton of the `platformio.ini` file if you prefer to rawdog it:

```
---snip---
lib_deps = 
    knolleary/PubSubClient@^2.8
    adafruit/Adafruit Unified Sensor@^1.1.14
    adafruit/DHT sensor library@^1.4.6
```

So there was a bit of work to get the libraries in place but now everything should just work.

### Pulling it all together

Linking all of this back to the two functions in the original pseudocode, there was a `sensor_init()` and a `sensor_read()` function.

The `sensor_init()` function can simply group the `pinMode()` and `dht.begin()` calls identified in the previous section and it's done.

The `sensor_read()` has a little more nuance because it seems the `dht.read...()` functions could fail. There's an opportunity to think about how to manage this scenario if it occurs. We want our implementation to be robust and to manage anything that happens without user intervention. So let's think about how to retry if it happens to fail. By consulting the library and the documentation for the device:

- If the read fails, the function returns `nan`
- The device has a refresh rate of 0.5Hz

This means that if the return value is `nan`, try again, but wait 2 seconds between each attempt. We also need to consider how to manage repeat failures and how many times we attempt to read from the sensor. The project's primary purpose is to allow for the garage door status to be read, so it can be concluded that it would be unacceptable for the temperature / humidity function to compromist that primary intent. One approach is:


```C
uint8_t read_attempts = 5;
bool read_success = false;
do {
    // Sensor has a 0.5Hz refresh rate, so wait 2 seconds for repeat reads
    if (read_attempts < 5) {
        delay(2000);
    }

    // Take reads
    temperature = dht.readTemperature();
    humidity = dht.readHumidity();

    // Check reads
    read_success = !isnan(temperature) && !isnan(humidity);
    read_attempts--;
} while (!read_success && read_attempts > 0);
```

Using a `do-while` loop, the code in the loop will execute at least once, and there are some conditionals in place so the sensor is tested a maximum of 5 times, and a delay of 2 seconds is used for repeat readings to respect the 0.5Hz refresh rate.

The final part of this file is to store the values in a struct to tidy things up a little.

## The connection_manager files

### WiFi

The majority of the WiFi code is boiler plate from ESP32 Arduino library examples. The exception to this is:

- To communicate with the MQTT broker securely, TLS is required. This is achieved by:
    - Using the `WiFiClientSecure` library
    - Setting a CA certificate to the client with the `setCACert` method (some brokers may require a client certificate and private key as well)
- If the connection fails after many attempts, restart the device
    - This implementation allows up to 10 attempts

Your broker may have a well established way to get certificates. But for a cloud-based broker, you can also use openSSL on the command line:

```bash
openssl s_client -connect your-broker.com:8883 -showcerts < /dev/null 2> /dev/null | openssl x509
```

Then the certificate can be saved in the `ca_cert.h` file.

### MQTT

This project is using the `PubSubClient` library. Like the WiFi, the majority of the MQTT code has significant similarities to the library examples. The only real difference is that there is a `connection_handler()` function implemented. The purpose of this is to keep the connection to the MQTT broker by:

- Making sure WiFi is still connected
- Making sure the connection to the MQTT broker is still active
- Pinging the MQTT broker

This needs to be done at least as frequently as the `MQTT keep alive` interval. For this project, the keep alive interval is 15 seconds, so we elect to call the connection handler every 10 seconds.

## Other files

The `config.h` file provides a convenient central location where all device settings (pin allocations, wifi and mqtt credentials, intervals between reads etc.) can be stored. This approach allows for those to be changed in a single location, and makes it straightforward to avoid sharing sensitive information because it's the only one location that needs to be excluded from a repository.

The `debug_print.h` file is a collection of macros allowing a straightforward approach to managing whether or not the microcontroller sends diagnostics to the serial interface.

## Broker and Dashboard

If you don't already have a go to solution for a low-code mqtt broker and dashboard, consider trying [Datacake](https://datacake.co/). There is a free tier for a limited number of devices and setting up a dashboard for a project like this is incredibly straightforward.

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

Arguments can be combined, however for the first compile I prefer to separate them so if a step fails, it's easier to isolate where the problem occurred.
