# Garage Monitor
![image](https://github.com/user-attachments/assets/e63ac389-65a6-4c24-9515-e5f5d7e31f60)

## Introduction

Have you ever felt like you left home without shutting the door, leaving your prized possessions visible to all passers-by? Have you ever doubled back home only to find the door was shut after all, so checking was just a waste of time?

This project seeks to solve this by using a microcontroller to detect the garage door status and transmit it to a cloud service, allowing for the status to be checked remotely on your phone.

For a modern microcontroller, this is a simple use case. Similarly, for the author, who routinely writes firmware for a day job, it's not a major challenge. To ensure neither the microcontroller or the engineer get too bored the project will add some other features, and demonstrate a number of different solutions to the problem.

## The Journey

This project is as simple or as complex as you care to make it. It is very easy to articulate what needs to be done by the microcontroller at a high level, so the problem presents a good opportunity to walk through a range of solutions and focus on the nuance of the implementation, rather than thrashing around with a particularly hard problem. My goal is to create the project multiple times, and in each subproject I will explore a different concept. In rough terms, I will start the journey at a high level and perform all tasks with abstractions provided by Arduino libraries, then I will gradually descend closer to the hardware and investigate pros and cons of having low-level control.

This project touches on many common features of embedded firmware design including:

- Inputs / Outputs
- Tasks
- Telemetry
- Interrupts

In the early implementations, some of these aspects will be dealt with in single-liners or not at all. The later projects will aim to pick one or more of these features as a theme of exploration.

## Solutions

The basic details of the solution will be common to all subprojects. The project will: 

- Detect the garage door status with a reed switch
- Read temperature and humidity with a digital sensor 
- Publish data via MQTT over WiFi
- Schedule readings and transmissions

## Hardware

For all of the proposed solutions, the following will be used:

- [Seeed XIAO ESP32C3](https://www.seeedstudio.com/Seeed-XIAO-ESP32C3-p-5431.html) development board
- Reed switch (normally open) and magnet to detect door status
- [DHT22](https://core-electronics.com.au/dht22-module-temperature-and-humidity.html) Temperature and Humidity sensor
- Power supply (5V USB-C)
- 3D printed case

Wiring and assembly of these components is relatively straightforward

## Dashboard

To subscribe to and visualise our data, the project will use [Datacake](https://datacake.co/) - a low-code IoT platform. For all of the proposed solutions, I send the data to the Datacake MQTT Broker, and I have set up a very basic dashboard:

![image](https://github.com/user-attachments/assets/e63ac389-65a6-4c24-9515-e5f5d7e31f60)

## Jump In

The subprojects are as follows, presented in roughly increasing order of implementation complexity:
1. [Using Arduino framework](https://github.com/TristanWebber/garage_monitor/tree/main/arduino) to create a synchronous and straightforward application
2. [Using ESP-IDF](https://github.com/TristanWebber/garage_monitor/tree/main/esp_idf) to leverage FreeRTOS tasks for concurrency, and use the Espressif APIs
3. [Using ESP-IDF and C++](https://github.com/TristanWebber/garage_monitor/tree/main/esp_cpp) to explore a higher level language, and create a driver for the environmental sensor
4. [Using Rust](https://github.com/TristanWebber/garage_monitor/tree/main/esp_rs_std) and the `std` crate to investigate the benefits and tradeoffs of Rust, while maintaining the familiarity of ESP-IDF interfaces
5. [Using Rust](https://github.com/TristanWebber/garage_monitor/tree/main/esp_rs_no_std) and the `no-std` crate to investigate baremetal programming on the ESP32
6. Using baremetal C to implement just the I/O features - COMING SOON
