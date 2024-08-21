# Rust `no-std` Implementation

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

This second Rust iteration of the garage monitor uses a different approach again - It is based on the `no-std` crate. Recall that the previous iteration was based on the Rust `std` crate - a Rust abstraction over the C bindings of the ESP-IDF. That meant that we had all features of the ESP-IDF available to us for rapid development of features and familiar APIs, but any application code we produced was a FreeRTOS task and therefore we did not have full control over scheduling and preempting. If we happened to want to guarantee that the only code being executed on the device was controlled by us to run exactly how and when we want, a `no-std` approach lets us achieve that. Another reason to favour `no-std` is it can remove dependencies on many of the Espressif APIs to precompiled binaries, as was commonplace in all of the previous examples. This created an unavoidable dependency on closed source code by a third party, and for some developers this could be considered unacceptable.

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

This project will examine the differences and similarities between the `std` and `no-std` approaches to programming an esp32 in Rust. It will follow a similar approach to previous projects by building out the features individually.

The commentary will cover:
- Reading from a GPIO and setting an interrupt
- Concurrency without FreeRTOS
- Wifi and MQTT without ESP-IDF

## Getting started

Of course, this repository exists and none of these steps are necessary if you simply want to use this repository but if you want to walk along with the entire process, here's how. Assuming you already have your [development environment](https://docs.esp-rs.org/book/installation/index.html) setup for `no-std` applications, create a new project using the following method. You can use whatever ESP32 you like, but pins used in this project are based on the `Seeed Studio XIAO ESP32C3`.

Using the `cargo-generate` tool, we create a brand new project from a esp-template:

```bash
cargo generate esp-rs/esp-template
```

This uses a series of prompts to create a partially configured project. When prompted, select a name for the project, select the target (ESP32C3 in this case) and do not enter the advanced configuration menu. If prompted to run `cargo fmt`, choose yes if you like, however if this is your first time, it will probably be a little slower because it will need to download dependencies. If everything executes correctly, you'll have a template project ready to build. If you want to make sure your toolchain is working properly and you want to see rust running on your board ASAP, you can simply enter the project directory and use the `cargo run` command:

```bash
cd esp_rs_no_std
cargo run
```

This will build, flash and monitor the template 'hello world' application. The tool will autodetect the device. Like the esp-idf and platform io tools used in previous projects, we can explicitly pass arguments if required, and this project will show some of those options as it progresses.

## Reading a GPIO
### Simple periodic GPIO read
Reading from a GPIO in a `no-std` project has only minor differences to the `std` approach of the previous example. There is still a HAL (the `esp_hal` crate) to facilitate basic operations using an API, so we needn't worry about reading and writing from registers. The process still involves some basic configuration and then periodic reading of the GPIO:

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

### Enable a GPIO Interrupt
Next, let's adapt this to enable an interrupt on the same gpio. The whole point of the device is to be able to very rapidly see what the actual state of the door is, so polling the sensor periodically is not enough. To achieve this, another dependency will need to be added to Cargo.toml:

#### **`Cargo.toml`**
```toml
# ---snip---
[dependencies]
critical-section = "1.1.2"
# ---snip---
```

Then let's log change of state with an interrupt handler:

#### **`main.rs`**
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

Notice that there is still no sign of an operating system. The GPIO interrupt is a hardware interrupt, so when it is triggered, it sets a flag for the CPU, causing the CPU to change context to a set of instructions in a different area of memory to the normal instructions. This allows this example to have an asynchronous 'task' without needing the overhead of an operating system, and this results in a very small and efficient binary that performs the task we have programmed, and nothing more. If you are keen to validate this, you can look at the assembly:

```bash
# Install the relevant tools
cargo install cargo-binutils
rustup component add llvm-tools
# Disassemble the binary and dump to a file
cargo objdump --bin esp_rs_no_std --release -- --disassemble --no-show-raw-insn --print-imm-hex > esp_rs_no_std.S
```

#### **`esp_rs_no_std.S`**
```asm
// ---snip---
//Disassembly of section .rwtext:

40380460 <interrupt1>:
40380460:       mv a1, a0
40380462:       li a0, 0x1
40380464:       j 0x40380602 <handle_interrupts>
// ---snip---
```

First off, all of the interrupts are in the `.rwtext`, segmented away from the rest of the code. Secondly, none of the assembly has a jump to any of the interrupts, so we have verified that if these are called, it is by the hardware, not the software. So this shows us that whenever a hardware interrupt occurs, the application jumps to the `handle_interrupts` function with two arguments (a1, a0) and iterates through any configured interrupts then applies any handlers.

### Add a debounce timer
The extremely low latency of the gpio interrupt means that a debounce will be needed to prevent noisy data that occurs from multiple interrupt triggers from the same event. Like the previous examples, a timer can be used. Without an OS, we can again rely on hardware features and user a timer callback. Let's reimagine the program flow:

- A GPIO interrupt triggers the gpio interrupt handler. Here, we:
    - Clear the GPIO interrupt
    - Temporarily stop listening for further GPIO interrupts
    - Start a debounce timer
- The timer expires and triggers the timer interrupt handler. Here, we:
    - Clear the timer interrupt
    - Read the GPIO state and log it
    - Re-listen for future GPIO interrupts

The code with additions and modifications looks like this:

#### **`main.rs`**
```rs
// ---snip---
// Import modules and crates for timer
use esp_hal::interrupt::{self, Priority},
use esp_hal::peripherals::{Interrupt, TIMG0},
use esp_hal::timer::timg::{Timer, Timer0, TimerGroup},
// ---snip---

// Create a Mutex<RefCell<_>> for the Timer
static DEBOUNCE_TIMER: Mutex<RefCell<Option<Timer<Timer0<TIMG0>, esp_hal::Blocking>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    // ---snip---

    // Create interrupt timer
    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks, None);
    let debounce_timer = timg0.timer0;
    debounce_timer.set_interrupt_handler(timer_handler);
    interrupt::enable(Interrupt::TG0_T0_LEVEL, Priority::Priority1).unwrap();

    // Move timer to the mutex refcell
    critical_section::with(|cs| {
        debounce_timer.listen();
        DEBOUNCE_TIMER.borrow_ref_mut(cs).replace(debounce_timer);
    });

    // ---snip---
}

// GPIO Interrupt handler unlistens, clears interrupt and starts debounce timer
#[handler]
fn handler() {
    critical_section::with(|cs| {
        // Clear GPIO interrupts
        let mut door_switch = DOOR_SWITCH.borrow_ref_mut(cs);
        let door_switch = door_switch.as_mut().unwrap();
        door_switch.clear_interrupt();
        door_switch.unlisten();

        // Start debounce timer
        let mut debounce_timer = DEBOUNCE_TIMER.borrow_ref_mut(cs);
        let debounce_timer = debounce_timer.as_mut().unwrap();
        debounce_timer.load_value(50u64.millis()).unwrap();
        debounce_timer.start();
    });
}

// Timer handler clears timer interrupt, reads door state, relistens for door interrupt
#[handler]
fn timer_handler() {
    critical_section::with(|cs| {
        DEBOUNCE_TIMER
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt();

        // Read the door GPIO state
        let mut door_switch = DOOR_SWITCH.borrow_ref_mut(cs);
        let door_switch = door_switch.as_mut().unwrap();
        println!("Interrupt triggered. Door is closed?: {}", door_switch.is_high());

        // Relisten for GPIO interrupts
        door_switch.listen(Event::AnyEdge);
    });
}
```

### Read from a DHT22 sensor

The project will use the same `dht-sensor` crate as the previous project. This approach will show how Generics and Traits in Rust can be useful for interoperability. First, the dependency can be added to the project:

#### **`Cargo.toml`**
```toml
[dependencies]
# ---snip---
dht-sensor = "0.2.1"
embedded-hal = { version = "0.2.7", features = [ "unproven" ] }
# ---snip---
```

The `features = [ "unproven" ]` flag is necessary to achieve compatibility between the sensor crate and the esp-hal crate. With the dependencies managed, the general process of reading the sensor is identical to the previous project. The obvious first attempt using a pin and delay from esp-hal:

#### **`main.rs`**
```rs
// ---snip---
use dht_sensor::{dht22, DhtReading};
// ---snip---
fn main() -> ! {
    // ---snip---
    // Sets GPIO4 as an input/output
    let mut sensor_pin = OutputOpenDrain::new(io.pins.gpio4, Level::High, Pull::None);
    let mut delay = Delay::new(&clocks);

    loop {
        // Read DHT22 sensor
        match dht22::Reading::read(&mut delay, &mut sensor_pin) {
            Ok(read) => {
                println!("Temperature: {}, Humidity: {}", read.temperature, read.relative_humidity);
            },
            Err(e) => {
                println!("Failed to read DHT sensor. Reason: {:?}", e);
            },
        }

        delay.delay_millis(300_000);
    }
}
```

Results in a slew of errors emanating from the `dht22::Reading::read` method, all pointing towards the delay and pin arguments failing to satisfy trait bounds. This means that the `Delay` and `OutputOpenDrain` types from the esp_hal crate are not implicitly compatible with the `Delay` and `InputOutputPin` types the dht_sensor crate is expecting to use. This is fine as long as we implement the missing traits, and by extension, ensure that the necessary methods are available.

To do this, let's create some wrapper types in a separate module. From the first unsuccessful attempt at making the driver work, there were some very useful error messages that become the TODO list. For the GPIO, the `InputPin` and `OutputPin` traits need to be satisfied. And as we start to implement those traits, Rust Analyzer starts to reveal the types and methods that need to be implemented in those traits to ensure compatibility.

To achieve this, first import crates and modules, using aliases because there are some naming conflicts. Then create our adapter type as a struct that simply contains the type we need to adapt - The `esp_hal::gpio::OutputOpenDrain` type. We need a `SensorAdapter::new()` method in order to actually use the new type:

#### **`dht_adapter.rs`**
```rs
use esp_hal::{
    gpio::{InputPin, Level, OutputOpenDrain, OutputPin, Pull},
    peripheral::Peripheral,
};

use embedded_hal::digital::v2::{InputPin as DhtInputPin, OutputPin as DhtOutputPin};

// sensor pin needs to implement traits embedded_hal::digital::v2::InputPin and embedded_hal::digital::v2::OutputPin
pub struct SensorAdapter<'a, P> {
    inner: OutputOpenDrain<'a, P>,
}

impl<'a, P> SensorAdapter<'a, P>
where
    P: InputPin + OutputPin,
{
    pub fn new(pin: impl Peripheral<P = P> + 'a) -> Self {
        SensorAdapter {
            inner: OutputOpenDrain::new(pin, Level::High, Pull::None),
        }
    }
}
```

So far, this is all pretty straightforward. Our wrapper type just contains the base type and none of the problems have been solved just yet. The magic happens when the traits are implemented for the adapter. If we were to start implementing one of the traits, the LSP would kindly tell us that for the InputPin trait, our base type is missing an `Error`, and methods `fn is_high(&self) -> Result<bool, Error>`, `fn is_low...`. Curiously, our base type already has methods with these names, but they do not implement an Error type, and they simply return a naked bool, rather than a Result.

So our wrapper needs to reconcile these differences:

```rs
impl<'a, P> DhtInputPin for SensorAdapter<'a, P>
where
    P: InputPin + OutputPin,
{
    type Error = core::convert::Infallible;

    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self.inner.is_high())
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(self.inner.is_low())
    }
}

impl<'a, P> DhtOutputPin for SensorAdapter<'a, P>
where
    P: InputPin + OutputPin,
{
    type Error = core::convert::Infallible;

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(self.inner.set_high())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(self.inner.set_low())
    }
}
```

Our base type doesn't return an error so in our trait implementation, the `Error` type is set as `Infallible`. For our methods, we call the methods on the base type and wrap the return value in `Ok()` so the return types are compatible. Now the traits are implemented and we've created compatibility between the driver crate and the no-std crate. A similar process is required for the `Delay` base type and the `DelayMs` & `DelayUs` traits. With that out of the way, minor changes to `main.rs` results in a fully functioning application - Still very small, fast and without operating system.

Next up, the connectivity needs to be tackled so that the results can be transmitted to our broker and inspected on our dashboard.

## Connectivity

TODO - talk about no-std approach to wifi, implement wifi, implement mqtt, embassy

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
