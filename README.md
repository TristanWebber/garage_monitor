TODO ToC
# Garage Monitor

Ever feel like you've left without shutting the door, leaving your prized possessions visible to all passers by? Have you ever doubled back home only to find you shut the door after all? Or worse, discovered that your hunch was correct and you carelessly forgot to close it?

This project seeks to solve this by detecting the garage door status and transmitting it to a cloud service, allowing for the status to be checked on your phone.

For a modern microcontroller, that is an easy life. Similarly, for the author, who routinely writes firmware for a day job, it's not a major challenge. To ensure neither the microcontroller or the engineer get too bored the project will add some other features, and demonstrate a number of different solutions to the problem.

## Solutions

The project will detect the garage door status with a reed switch. It will read temperature and humidity in the garage with a digital sensor with a single wire bus. This data will be published via MQTT over WiFi. It is not necessary to read from the sensors constantly so the microcontroller can be put to sleep when it's not actively reading a sensor or transmitting data.

To look at our data, the project will use existing low code cloud products.

This solution touches on many common features of embedded firmware design including:
- Inputs / Outputs
- Managing multiple tasks
- Telemetry
- Power management and Interrupts

TODO links
To demonstrate numerous ways to solve the problem, the project will use the following approaches, presented in roughly increasing order of complexity:
1. Using Arduino framework
2. Using ESP-IDF
3. Using ESP-IDF and C++
4. Using Rust and the `std` crate
5. Using Rust and the `no-std` crate
6. Using baremetal C to implement just the I/O features

## Hardware

For all of the proposed solutions, the following will be used:

TODO links
- Seeed XIAO ESP32C3 development board
- Reed switch (normally open) and magnet to detect door status
- DHT22 Temperature and Humidity sensor
- Raspberry Pi power supply
- 3D printed case

## How to solve

- Wake up due to powerup, interrupt or timer
- Take a digital read from the reed switch pins
- Setup an interface to the DHT sensor
- Take reads from the DHT sensor
- Interpret the reads from the DHT sensor
- Connect to WiFi
- Connect to NTP server
- Connect to MQTT broker
- Publish reads
- Calculate next wake time
- Set sleep so that the device wakes for an interrupt from the door switch or a period of time passes
- Go in to low power mode
