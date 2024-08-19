# Rust `std` Implementation

- [Introduction](#introduction)
- [Quickstart](#quickstart)
- [The Strategy](#the-strategy)
- [Getting Started](#getting-started)
- [Reading a GPIO](#reading-a-gpio)
- [Connectivity](#connectivity)
- [The config file](#the-config-file)
- [Building and Flashing](#building-and-flashing)
- [Observations and Next Steps](#observations-and-next-steps)

## Introduction

This second Rust iteration of the garage monitor uses a different approach again - It is based on the `no-std` library. Recall that the previous iteration implemented the Rust `std` library as an abstraction over the ESP-IDF. That meant that we had all features of the ESP-IDF available to us for rapid developlment of features, but any application code we produced was a FreeRTOS task and therefore we did not have full control over scheduling and preempting. If we happened to want to guarantee that the only code being executed on the device was controlled by us to run exactly how and when we want, a `no-std` approach lets us achieve that. Another downside of all of the previous applications is their reliance on Espressif APIs to precompiled binaries. This created an unavoidable dependency on closed source code by a third party.

In this example, the tradeoffs and benefits of the no-std approach will be explored.

## Quickstart

This series of projects is presented as an demonstration and discussion piece, however if you simply want to take some or all of the repository and use it for your own projects without suffering through the narrative you can:

0. [Acquire and assemble](https://github.com/TristanWebber/garage_monitor/blob/main/README.md#hardware) the appropriate hardware
1. Setup a [dashboard](https://github.com/TristanWebber/garage_monitor/blob/main/README.md#dashboard)
2. Ensure you have the [development environment](https://docs.esp-rs.org/book/installation/index.html) configured for `no-std` applications
3. Clone the repository. If you are only interested in this subproject, use `sparse-checkout`
```bash
git clone --no-checkout https://github.com/TristanWebber/garage_monitor.git && \
cd garage_monitor && \
git sparse-checkout esp_rs_no_std && \
git checkout
```
4. [Update](#the-config-file) and rename the `_config.rs` file
5. [Build and flash](#building-and-flashing) the project to your microcontroller

## The strategy

This project will examine the differences and similarities between the std and no-std approaches to programming an esp32 in Rust. It will follow a similar approach to similar projects by building out the features individually.

The commentary will cover:
- Reading from a GPIO and setting an interrupt
- Concurrency
- Wifi and MQTT

## Getting started

Of course, this repository exists and none ofthese steps are necessary if you simply want to use this repository but if you want to walk along with the entire process, here's how. Assuming you already have your [development environment](https://docs.esp-rs.org/book/installation/index.html) setup for `std` applications, create a new project using the following method. You can use whatever ESP32 you like, but pins used in this project are based on the `Seeed Studio XIAO ESP32C3`.

Using the `cargo-generate` tool, we create a brand new project from a esp-template:

```bash
cargo generate esp-rs/esp-template
```

This uses a series of prompts to create a partially configured project. When prompted, select a name for the project, select the target (ESP32C3 in this case) and do not enter the advanced configuration menu. If prompted to run `cargo fmt`, select no because this may result in failure if this is not already installed on your system. If everything executes correctly, you'll have a template project ready to build. If you want to make sure your toolchain is working properly and you want to see rust running on your board ASAP, you can simply enter the project directory and use the `cargo run` command:

```bash
cd esp_rs_no_std
cargo run
```

This will build, flash and monitor. The tool will autodetect the device. Like the esp-idf and platform io tools used in previous projects, we can explicitly pass arguments if required, and this project will show some of those options as it progresses.

## Reading a GPIO
Reading from a GPIO in a `no-std` project does not differ significantly from the `std` approach of the previous example. There is still a HAL (the `esp_hal` crate) to facilitate basic operations using an API, so we needn't worry about reading and writing from registers. The process still involves some basic configuration and then periodic reading of the GPIO:

```rs
#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{Input, Io, Pull},
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
};
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    // Set GPIO3 as an input with internal pulldown resistor
    let door_switch = Input::new(io.pins.gpio3, Pull::Down);

    let delay = Delay::new(&clocks);
    loop {
        println!("Door is closed?: {}", door_switch.is_high());
        delay.delay_millis(1000);
    }
}
```

In our previous four projects, by the time the processor hits our application code, a lot of invisible code has already been run and some threads are already running on the processor. This time that is not the case, and this is flagged with tags `#![no_std]`, `#![no_main]` and `#[entry]`. Not having FreeRTOS up and running means that our `main` is the only process running at this point, but it also means that there is no FreeRTOS delay to lean on, so we use abstractions over the system clock that implement a blocking delay. This is a double edged sword in that the implementation is doing only what we ask it to (aka very little) but there is no sophisitication and if we wanted to multitask, we would need to take a different approach. This is reflected in the binaries as well - compiling this is very fast, and the binary is tiny.

Next, let's adapt this to enable an interrupt on the same gpio. First, another dependency will need to be added to Cargo.toml:

#### **`Cargo.toml`**
```toml
# ---snip---
[dependencies]
critical-section = "1.1.2"
# ---snip---
```

Then let's log change of state with an interrupt handler:

```rs
// ---snip---
use core::cell::RefCell;
use critical_section::Mutex;
use esp_hal::gpio::{Event, Gpio3};

// Allow for mutable sharing of the door switch GPIO
static DOOR_SWITCH: Mutex<RefCell<Option<Input<Gpio3>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    // ---snip---
    // Set the interrupt handler for GPIO interrupts
    io.set_interrupt_handler(handler);

    // Move switch gpio to the mutex refcells
    critical_section::with(|cs| {
        door_switch.listen(Event::AnyEdge);
        DOOR_SWITCH.borrow_ref_mut(cs).replace(door_switch);
    });

    let delay = Delay::new(&clocks);
    loop {
        let mut door_state = false;
        critical_section::with(|cs| {
            door_state = DOOR_SWITCH
                .borrow_ref(cs)
                .as_ref()
                .unwrap()
                .is_high();
        });
        println!("Door is closed?: {}", door_state);
        delay.delay_millis(300_000);
    }
}

// Interrupt handler logs the door state and clears the interrupt
#[handler]
fn handler() {
    critical_section::with(|cs| {
        DOOR_SWITCH
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt();
        let door_state = DOOR_SWITCH
            .borrow_ref(cs)
            .as_ref()
            .unwrap()
            .is_high();
        println!("Interrupt triggered. Door is closed?: {}", door_state);
    });
}
```

In similar fashion to the previous project, things escalated quickly because a mutable reference needs to be shared. This time, a global Mutex<RefCell<Option<_>>> is the data structure of choice because the interrupt handler cannot take any arguments. Creating this globally means it is declared before the GPIO has been configured. To borrow the reference within the Mutex<RefCell<>> a critical section is required. Setting this up has doubled the amount of code, but allows us to use the GPIO in both our main function and the interrupt.

Notice that there is still no sign of an operating system. The GPIO interrupt is a hardware interrupt, so when it is triggered, it sets a flag for the CPU, causing the CPU to change context to a set of instructions in a different area of memory to the normal instructions. This allows this example to have an asynchronous 'task' without needing the overhead of an operating system, and this results in a very small and efficient binary that performs the task we have programmed, and nothing more.

TODO - add debounce timer, read dht, create threads.

## Connectivity

TODO - talk about no-std approach to wifi, implement wifi, implement mqtt

## The config file

The config file contains secrets and configurable values. Similar to the C++ project, the config file is a namespace, so we can use the namespace, or explicitly resolve the namespace with the `config::CONSTANT` syntax. It is included in the `main.rs` file with keyword `mod config`, and because it is globally accessible in `main.rs`, it can be made visible elsewhere in the project, such as `connections.rs` with the syntax `use crate::config`. Note the constants are made public with the `pub` keyword.

If you are building from this repo for your own project, make sure to rename the `_config.rs` file to `config.rs` and update all consts to values for your own project. The `config.rs` file is ignored in the .gitignore to lower the risk of sharing of secrets.

## Building and flashing

Commands to build, flash and monitor the project are as follows:

```bash
# build
cargo build --release

# flash
espflash flash target/riscv32imc-unknown-none-elf/release/esp_rs_no_std

# monitor
espflash monitor

# build, flash and monitor
cargo run --release
```

## Observations and Next Steps

TODO
