# Garage Monitor

- [Introduction](#introduction)
- [The Journey](#the-journey)
- [Solutions](#solutions)
- [Hardware](#hardware)
- [Dashboard](#dashboard)

## Introduction

Have you ever felt like you left home without shutting the door, leaving your prized possessions visible to all passers by? Have you ever doubled back home only to find the dooe was shut after all and just wasted a bunch of time checking?

This project seeks to solve this by using a microcontroller to detect the garage door status and transmitting it to a cloud service, allowing for the status to be checked on your phone.

For a modern microcontroller, that is an easy life. Similarly, for the author, who routinely writes firmware for a day job, it's not a major challenge. To ensure neither the microcontroller or the engineer get too bored the project will add some other features, and demonstrate a number of different solutions to the problem.

## The Journey

This project is as simple or as complex as you care to make it. It is very easy to articulate what needs to be done by the microcontroller at a high level, so the problem presents a good opportunity to walk through a range of solutions and focus on the nuance of the implementation, rather than thrashing around with a particularly hard problem. My goal is to create the project multiple times, and in each subproject I will explore a different concept. In rough terms, I will start the journey at a relatively high level and perform all tasks with abstraction, then I will gradually descend closer to the hardware and investigate pros and cons of having a low level of control.

This project touches on many common features of embedded firmware design including:

- Inputs / Outputs
- Managing multiple tasks
- Telemetry
- Power management and Interrupts

At the early implementations, some of these aspects will be dealt with in single-liners or not at all. The later projects will aim to pick one or more of these features as a theme of exploration.

## Solutions

The basic details of the solution will be common to all subprojects. The project will: 

- Detect the garage door status with a reed switch
- Read temperature and humidity with a digital sensor 
- Publish data via MQTT over WiFi
- Schedule readings and transmissions

## Hardware

For all of the proposed solutions, the following will be used:

TODO links
- Seeed XIAO ESP32C3 development board
- Reed switch (normally open) and magnet to detect door status
- DHT22 Temperature and Humidity sensor
- Raspberry Pi power supply
- 3D printed case

TODO assembly guide

## Dashboard

To subscribe to and visualise our data, the project will use existing low code cloud products. For all of the proposed solutions, I have set up a very basic dashboard as follows:

TODO: Add links and screenshots

## Jump In

TODO links
The subprojects are as follows, presented in roughly increasing order of implementation complexity:
1. [Using Arduino framework](https://github.com/TristanWebber/garage_monitor/tree/main/arduino)
2. Using ESP-IDF
3. Using ESP-IDF and C++
4. Using Rust and the `std` crate
5. Using Rust and the `no-std` crate
6. Using baremetal C to implement just the I/O features


## Misc - topics to explore in subprojects. To remove

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
