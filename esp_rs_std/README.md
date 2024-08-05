# Rust `std` Implementation

- [Introduction](#introduction)
- [Quickstart](#quickstart)
- [The Strategy](#the-strategy)
- [Getting Started](#getting-started)
- [Reading a GPIO and sharing data between threads](#reading-a-gpio-and-sharing-data-between-threads)
- [Connectivity](#connectivity)
- [The config file](#the-config-file)
- [Building and Flashing](#building-and-flashing)
- [Observations and Next Steps](#observations-and-next-steps)

## Introduction

The first Rust iteration of the garage monitor project will be an exercise in exploring the Rust language for embedded programming on the ESP32 microcontroller. The objective is to implement an application that achieves the same functionality as the C and C++ versions created previously.

After three working versions of this project, why use a different language to create something that fundamentally does the same thing? One of the biggest reasons is that a project that is a known-quantity, so it's an ideal way to focus on the nuance of the language rather than the application logic. Another is that the Rust language exists to target some of the weak points of C and C++. In C and C++, managing memory correctly and accessing data using pointers is something that is wholly the responsibility of the programmer, and it is possible to make significant mistakes without the compiler emitting any warnings. Whereas in Rust, the language and the compiler have strict rules to prevent common memory management issues. These features should result in a robust application that is free of memory related bugs.

There are two approaches to programming the ESP32 microcontrollers in Rust:
- Using the standard (std) library - a Rust abstraction over the ESP-IDF
- Using the library (no-std) - bare metal Rust

The `std` approach has far greater functionality out of the box. This means that it is a gentler introduction to the topic, and almost anything that can be done in C with the ESP-IDF can also be done in Rust. This is the approach this first example will showcase.

## Quickstart

This series of projects is presented as an demonstration and discussion piece, however if you simply want to take some or all of the repository and use it for your own projects without suffering through the narrative you can:

0. [Acquire and assemble](https://github.com/TristanWebber/garage_monitor/blob/main/README.md#hardware) the appropriate hardware
1. Setup a [dashboard](https://github.com/TristanWebber/garage_monitor/blob/main/README.md#dashboard)
2. Ensure you have the [development environment](https://docs.esp-rs.org/book/installation/index.html) configured for `std` applications
3. Clone the repository. If you are only interested in this subproject, use `sparse-checkout`
```bash
git clone --no-checkout https://github.com/TristanWebber/garage_monitor.git && \
cd garage_monitor && \
git sparse-checkout esp_rs_std && \
git checkout
```
4. [Update](#the-config-file) and rename the `_config.h` file
5. [Build and flash](#building-and-flashing) the project to your microcontroller

## The strategy

This iteration of the project will focus on key Rust language features and challenges. The general structure of the project will loosely mirror the previous versions, and the focus of the README commentary will be on incrementally building out the functions. Some of the detail of the previous projects will be omitted from this version. So beware - This version of the monitor is not a one-to-one equivalent of the ESP-IDF versions. The code will compile and performs the core requirements, but is not resilient to real-world factors like intermittent wifi. However, these shortfalls are concious decisions to instead focus on documenting some of the most important concepts for creating multitasking applications on the esp32 with `std` Rust.

The commentary will cover:
- Reading from a GPIO
- Errors as values
- Threads and concurrency
- Introduction to modules
- Publishing data using MQTT

## Getting started

Of course, this repository exists and none of these steps are necessary if you simply want to use this repository but if you want to walk along with the entire process, here's how. Assuming you already have your [development environment](https://docs.esp-rs.org/book/installation/index.html) setup for `std` applications, create a new project using the following method. You can use whatever ESP32 you like, but pins used in this project are based on the `Seeed Studio XIAO ESP32C3`.

Using the `cargo-generate` tool, we create a brand new project from a esp-idf template:

```bash
cargo generate esp-rs/esp-idf-template cargo
```

This uses a series of prompts to create a partially configured project. When prompted, select a name for the project, select the target (ESP32C3 in this case) and do not enter the advanced configuration menu. If everything executes correctly, you'll have a template project ready to build. If you want to make sure your toolchain is working properly and you want to see rust running on your board ASAP, you can simply enter the project directory and use the `cargo run` command:

```bash
cd esp_rs_std
cargo run
```

This will build, flash and monitor. The tool will autodetect the device. Like the esp-idf and platform io tools used in previous projects, we can explicitly pass arguments if required, and this project will show some of those options as it progresses.

## Reading a GPIO and sharing data between threads
Our first challenge will be to read a GPIO to detect the status of the reed switch on the door. To do this, we will use a handful of modules from the `esp_idf_svc` crate to facilitate:
- Accessing the `Peripherals` to allow our program to use interfaces associated with our chip
- Accessing the `gpio` to allow our program to adjust settings and perform operations with the gpio pins
- Accessing `FreeRtos` to allow our program to use delays

This provides a foundation to periodically read and log the state of a GPIO:

```rs
use esp_idf_svc::hal::{
    gpio::{PinDriver, Pull},
    peripherals::Peripherals,
    delay::FreeRtos,
};

fn main() -> ! {
    // Boilerplate for ESP std Rust projects
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    // Takes ownership of the Peripherals struct
    let peripherals = Peripherals::take().unwrap();

    // Sets GPIO3 as an input with internal pulldown resistor
    let mut door_switch = PinDriver::input(peripherals.pins.gpio3).unwrap();
    door_switch.set_pull(Pull::Down).unwrap();

    // Read and log the level of the reed switch, then yield for 1 second
    loop {
        let door_status = door_switch.get_level();
        log::info!("Door status is {:?}", door_status);
        FreeRtos::delay_ms(1000);
    }
}
```

This minimal program will display a message once a second and in all likelihood, will never throw an error. This provides a first opportunity to look at some key differences between Rust and C. Examining the return type of some of the functions used in this simple example, observe that `Result<T, E>` is used. That is, the functions could return the values we want, or they could return an error as a value. As the programmer we are forced to deal with this explicitly, unlike in the previous iterations where we often simply ignore the possibility of certain functions failing. In the example above, we have used `unwrap()` as a path of least resistance. This means that if an error occurs, the program will panic. In this program, if the basic gpio configuration fails, a panic is appropriate but there are better ways to aproach this that become increasingly important as applications become more complex. The `anyhow` crate allows for unhandled errors to be handled at an application-level and logged with additional context, making our debugging cleaner and easier than it could otherwise be. We can start using anyhow by adding the crate as a dependency in our `Cargo.toml` file and updating our `main.rs` file as follows:

#### **`Cargo.toml`**
```toml
# ---snip---

[dependencies]
anyhow = "1.0.86"

# ---snip---
```

#### **`main.rs`**
```rs
use anyhow::Result;

// ---snip---

fn main() -> Result<()> {
    // ---snip---

    // Takes ownership of the Peripherals struct
    let peripherals = Peripherals::take()?;

    // Sets GPIO3 as an input with internal pulldown resistor
    let mut door_switch = PinDriver::input(peripherals.pins.gpio3)?;
    door_switch.set_pull(Pull::Down)?;

    // ---snip---
}
```

We're still saying that we think an error is unlikely and the program should crash if that happens but `anyhow` will handle it. Rather than using `unwrap()` everywhere, we use `?` giving a slightly more terse syntax.

That's the basic structure of a digital read sorted, but the design brief requires interrupts, so that when a change of state occurs, the state can be broadcast as soon as possible. Like our previous FreeRTOS implementations, we can solve this by:
- Enable an interrupt on the relevant GPIO
- Create an interrupt callback function
- Create a task to listen for notifications from the interrupt callback

This iteration results in a not-so-gentle introduction to concurrency in Rust. At a very basic level, we can take a similar approach to C++ to create FreeRTOS tasks without touching FreeRTOS. One way to do this is using the `std::thread::spawn<F, T>` API, and this allows for very easy creation of threads when they are independent of the rest of the system. However, when sharing data between threads, Rust suddenly becomes quite different to C and C++. Rust seeks to avoid any ambiguity about ownership of resources. But the realities of real-world threaded applications is that sharing of data is unavoidable. In this example, multiple threads will need to access the GPIO and information about the state of the interrupt.

We will use `Arc<Mutex<T>>` to allow for threads to share resources. The Mutex protects against race conditions, and the Arc (Atomic Reference Counter) allows the borrow checker to keep track of how many references exist. When the Arc determines that there are no references remaining in any scope, the item is dropped.

#### **`main.rs`**
```rs
use std::sync::{Arc, Mutex};
use std::thread;

use esp_idf_svc::hal::delay;
use esp_idf_svc::hal::gpio::{InterruptType, PinDriver, Pull};

// ---snip---

fn main() -> Result<()> {
    // ---snip---

    // Sets GPIO3 as an interrupt, triggered on any edge
    door_switch.set_interrupt_type(InterruptType::AnyEdge)?;

    // Flag to track if an interrupt has been triggered. Use <Arc<Mutex<bool>> for thread safety
    let interrupt_triggered = Arc::new(Mutex::new(false));

    // Attaches a callback function (a closure in this case) to execute when a notification occurs
    unsafe {
        let interrupt_triggered = interrupt_triggered.clone();
        door_switch.subscribe(move || {
            *interrupt_triggered.lock().unwrap() = true;
        })?;
    }

    // Wrap the PinDriver in Arc<Mutex<PinDriver>> to facilitate sharing
    let door_switch = Arc::new(Mutex::new(door_switch));

    // Read and log the level of the reed switch, then yield for 1 second
    let door_switch_t1 = door_switch.clone();
    let thread1 = thread::spawn(move || loop {
        let level = door_switch_t1.lock().unwrap().is_high();
        log::info!("Thread 1: GPIO3 level is {}", level);
        delay::FreeRtos::delay_ms(1000);
    });

    // Thread to listen for interrupts
    let door_switch_t2 = door_switch.clone();
    let interrupt_triggered_t2 = interrupt_triggered.clone();
    let thread2 = thread::spawn(move || loop {
        door_switch_t2.lock().unwrap().enable_interrupt().unwrap();
        // Wait for an interrupt to occur
        while !*interrupt_triggered_t2.lock().unwrap() {
            delay::FreeRtos::delay_ms(1);
        }
        // Basic debounce: A short settling period
        delay::FreeRtos::delay_ms(50);
        let level = door_switch_t2.lock().unwrap().is_high();
        // Set the interrupt flag back to false
        *interrupt_triggered_t2.lock().unwrap() = false;
        log::info!("Thread 2: GPIO3 level is {}", level);
    });

    thread1.join().unwrap();
    thread2.join().unwrap();

    unreachable!();
}
```

Like the previous examples, the pin is configured so that it triggers an interrupt on any edge. We will create a callback function that will be executed whenever the interrupt is detected. In this case, all the callback does is sets the value of a boolean. A sender thread will listen for changes to this boolean and when a change is detected, the value of the pin will be read and a message will be logged.

This introduces the two pieces of data that will be shared between threads - the gpio itself, and the flag to be set by the interrupt callback. Before these are shared to their respective threads, they are wrapped in Arc<Mutex<T>> and cloned each time they are moved. Then, in syntax familiar to Mutex in any other language, we `lock()` the mutex when we want to access the underlying structure.

Some syntax that is very common in Rust, but does not exist in C, is the use of closures, or unnamed functions. In this case, closures have been used for the interrupt callback and the threads. The closures use the syntax `|args| {code_to_run}` and this saves us writing a function. This is handy if the function is short and/or we do not intend to reuse the function elsewhere. However, it would have been equally valid to call a function in each of the three locations a closure was used here. As a final note, the `move` keyword captures the values used in the closure. That is, ownership is moved to the closure.

The DHT sensor is managed by using a crate. Implementing the interface is a straightforward case of executing according to the docs but there is a good opportunity to to investigate a rusty approach to dealting with a result type.

```rs
match dht22::Reading::read(&mut dht_delay, &mut sensor_pin) {
    Ok(read) => {
        log::info!("Thread 1: Temperature: {}, Humidity: {}", read.temperature, read.relative_humidity);
        // Publish the read
        // Publish the door status
    },
    Err(e) =>{
        log::error!("Thread 1: Failed to read DHT sensor. Error: {:?}", e);
        // Publish the door status
    },
}
```

When reading the sensor, a result is returned and we `match` if the result contains a read or an error. Rather than the `?` operator (deal with the error elsewhere) or `unwrap()` (panic if something goes wrong), the `Err(e)` arm of the match allows us to execute whatever code is neede. In this case, we consider the door reading to be more important than the environmental reads so if the environmental sensor result contains an error, we simply log that error, publish the door status and continue with the program.

With the basic interface to the reed switch in place, the next item to look at is connectivity.

## Connectivity

Like the other projects, data will be sent to a cloud MQTT broker over WiFi. Because the `esp_idf_svc` crate is a wrapper over the ESP-IDF, a lot of this process will be similar to the previous projects.

This process is:
- Create an event loop to handle WiFi and IP events
- Configure the NVS partition
- Take ownership of the modem peripheral
- Create the WiFi client
- Create the MQTT client
- Allow the MQTT client to be accessed by concurrent threads

Adding the connectivity is an opportunity to move some code out of `main.rs` to keep things tidy, and to explore modules.

Drawing on the examples from the `esp_idf_svc` crate documentation, the above steps would be achieved by:

#### **`main.rs`**
```rs
// ---snip---

fn main() -> Result<()> {
    // ---snip---
    mod config;
    mod connections;

    use connections::{wifi_create, mqtt_create};

    // Configure event loop and nvs for wifi
    // NOTE: if these fail, panic is appropriate
    let event_loop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;

    // Connect wifi
    let modem = peripherals.modem;
    let _wifi_client = wifi_create(&event_loop, &nvs, modem)?;
    thread::sleep(Duration::from_secs(3));

    // Connect mqtt
    let (mqtt_client, mut mqtt_conn) = mqtt_create(config::BROKER_URI, config::BROKER_USER, config::BROKER_PASS)?;

    // Allow MQTT client to be used in multiple threads
    let mqtt_client = Arc::new(Mutex::new(mqtt_client));

    // For each thread the client is used:
    let mqtt_client_t1 = mqtt_client.clone();

    // Lock the client mutex and use it (eg to publish)
}
```

#### **`connections.rs`**
```rs
pub fn wifi_create(
    event_loop: &EspSystemEventLoop,
    nvs: &EspDefaultNvsPartition,
    modem: Modem,
) -> Result<EspWifi<'static>, EspError> {
    // Create wifi station
    let mut esp_wifi = EspWifi::new(modem, sys_loop.clone(), Some(nvs.clone()))?;
    let mut wifi = BlockingWifi::wrap(&mut esp_wifi, sys_loop.clone())?;

    // Configure wifi station
    wifi.set_configuration(&Configuration::Client(ClientConfiguration {
        ssid: config::WIFI_SSID.try_into().unwrap(),
        password: config::WIFI_PASS.try_into().unwrap(),
        ..Default::default()
    }))?;

    // Start, connect and wait for connection
    wifi.start()?;
    log::info!("Wifi started");

    wifi.connect()?;
    log::info!("Wifi connected");

    wifi.wait_netif_up()?;
    log::info!("Wifi netif up");

    // Return result
    Ok(esp_wifi)
}

pub fn mqtt_create(url: &str, username: &str, password: &str) -> Result<(EspMqttClient<'static>, EspMqttConnection), EspError> {
    // Create and configure mqtt client and connection
    // Return result
}
```

The public connectivity functions have been tidied away into the `connections.rs` file, and are accessed in the `main.rs` file with `mod connections`.

This throws us headlong into some of the more aesthetically jarring syntax of Rust, and some concepts that are significantly different to C. The key differentiating concepts here are `Result<T, E>` and the `<'static>` lifetime annotation.

In contrast to the previous section where we were accessing the value inside results, this time we're returning a result. Here, the real value of the `?` operator becomes clear. In the `wifi_create` function, note that there is a single point where a successful result is returned - The final line `Ok(esp_wifi)`. In contrast there are many potential failure points. Every other step could return an error. This is managed by the use of the `?` operator, and this works as shorthand for checking for an error, and returning early if one has occurred. This can save a lot of lines of code in some situations.

This leaves the `<'static>` lifetime annotation. Consider the `EspMqttClient` and the way this object will be used in practice. It's something that we could plausibly want to use on an ad-hoc basis to publish or subscribe. This poses a challenge for the borrow checker because, in overly simplified terms the borrow checker says "use it or lose it". The lifetime annotation is a way to explicitly tell the compiler how something should be used and the brute force <'static> annotation will guarantee the variable lasts the life of the program.

## The config file

The config file contains secrets and configurable values. Similar to the C++ project, the config file is a namespace, so we can use the namespace, or explicitly resolve the namespace with the `config::CONSTANT` syntax. It is included in the `main.rs` file with keyword `mod config`, and because it is globally accessible in `main.rs`, it can be made visible elsewhere in the project, such as `connections.rs` with the syntax `use crate::config`. Note the constants are made public with the `pub` keyword.

If you are building from this repo for your own project, make sure to rename the `_config.rs` file to `config.rs` and update all consts to values for your own project. The `config.rs` file is ignored in the .gitignore to lower the risk of sharing of secrets.

## Building and flashing

Commands to build, flash and monitor the project are as follows:

```bash
# build
cargo build --release

# flash
espflash flash target/riscv32imc-esp-espidf/release/esp_rs_std

# monitor
espflash monitor

# build, flash and monitor
cargo run --release
```

There are many additional CLI options, however these tools have been incredibly reliable at correctly detecting the port, flash size and baud with just the defaults. For the most part, `cargo run` is the easiest during the development phase, unless you don't want to flash or monitor, in which case `cargo build` does the job.

## Observations and Next Steps

This project has shown how a modern low level language can implement all the functionality of the C family of languages, but with additional safety and plenty of modern conveniences. That safety comes at a cost - extra syntax and a pedantic compiler - but it's a great tool to have, and the current momentum against C means that sooner or later, a language like Rust or Zig will inevitably start to get more market share in the embedded domain.

The next subproject will explore using bare metal, or `no-std` Rust. This is another available approach that favours a very low memory footprint and control over the convenience of the `std` approach.

Check it out [here](https://github.com/TristanWebber/garage_monitor/tree/main/rs_std)
