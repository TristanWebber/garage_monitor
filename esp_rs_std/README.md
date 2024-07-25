# Rust `std` Implementation

!! Work in progress !!

- [Introduction](#introduction)
- [Quickstart](#quickstart)
- [The Strategy](#the-strategy)
- [Getting Started](#getting-started)
- [Reading a GPIO and sharing data between threads](#reading-a-gpio-and-sharing-data-between-threads)
- [Building and Flashing](#building-and-flashing)

## Introduction

The first Rust iteration of the garage monitor project will be an exercise in exploring the Rust language for embedded programming on the ESP32 microcontroller. The objective is to implement an application that achieves the same functionality as the C and C++ versions created previously.

So after three working versions of this project, why use a different language to create something that fundamentally does the same thing? One of the biggest reasons is that a project that is a known-quantity, so it's an ideal way to focus on the nuance of the language rather than the application logic. Another is that the Rust language exists to target some of the weak points of C and C++. In C and C++, managing memory correctly and accessing data using pointers is something that is wholly the responsibility of the programmer, and it is possible to make significant mistakes wihtout the compiler emitting any warnings. Whereas in Rust, the language and the compiler have strict rules to prevent common memory management issues. These features should result in a robust application that is free of memory related bugs.

There are two approaches to programming the ESP32 microcontrollers in Rust:
- Using the standard (std) library - a Rust abstraction over the ESP-IDF
- Using the library (no-std) - bare metal Rust

The `std` approach has far greater functionality out of the box. This means that it is a gentler introduction to the topic, and almost anything that can be done in C with the ESP-IDF can also be done in Rust.

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
4. [Build and flash](#building-and-flashing) the project to your microcontroller

## The strategy

This iteration of the project will focus on key Rust language features and challenges. The general structure of the project will mirror the previous versions, and the focus of the README commentary will be on incrementally building out the functions.

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

We're still saying that we think an error is unlikely and the program should crash if that happens but `anyhow` will handle it.

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

Insert commentary here. Cover the use of Arc<Mutex<T>>, closures, unsafe block, maybe even the thread::sleep bug.

With the basic interface to the reed switch in place, the next item to look at is connectivity.

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

There are many additional CLI options, however these tools have been incredibly reliable at correctly detecting the port, flash size and baud. For the most part, `cargo run` is the easiest during the development phase, unless you don't want to flash or monitor, in which case `cargo build` does the job.