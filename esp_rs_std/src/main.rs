use anyhow::Result;

use std::sync::{Arc, Mutex};
use std::thread;

use esp_idf_svc::hal::delay;
use esp_idf_svc::hal::gpio::{InterruptType, PinDriver, Pull};
use esp_idf_svc::hal::peripherals::Peripherals;

use dht_sensor::{DhtReading, dht22};

fn main() -> Result<()> {
    // Boilerplate for ESP std Rust projects
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    // Takes ownership of the Peripherals struct
    let peripherals = Peripherals::take()?;

    // Sets GPIO3 as an input with internal pulldown resistor
    let mut door_switch = PinDriver::input(peripherals.pins.gpio3)?;
    door_switch.set_pull(Pull::Down)?;
    door_switch.set_interrupt_type(InterruptType::AnyEdge)?;

    // Sets GPIO4 as an input/output
    let mut sensor_pin = PinDriver::input_output(peripherals.pins.gpio4)?;

    // Flag to track if an interrupt has been triggered. Use <Arc<Mutex<bool>> for thread safety
    let interrupt_triggered = Arc::new(Mutex::new(false));

    // TODO add safety comment
    unsafe {
        let interrupt_triggered = interrupt_triggered.clone();
        door_switch.subscribe(move || {
            *interrupt_triggered.lock().unwrap() = true;
        })?;
    }

    let door_switch = Arc::new(Mutex::new(door_switch));

    // Read and log the level of the reed switch, then yield for DELAY seconds
    const DELAY_S: u32 = 30;
    let door_switch_t1 = door_switch.clone();
    let thread1 = thread::spawn(move || loop {
        // Read from the door switch
        let level = door_switch_t1.lock().unwrap().is_high();
        log::info!("Thread 1: GPIO3 level is {}", level);

        // Read from the dht sensor
        let mut dht_delay = delay::Ets;
        sensor_pin.set_high().unwrap();
        match dht22::Reading::read(&mut dht_delay, &mut sensor_pin) {
            Ok(read) => log::info!("Thread 1: Temperature: {}, Humidity: {}", read.temperature, read.relative_humidity),
            Err(e) =>log::error!("Thread 1: Failed to read DHT sensor. Error: {:?}", e),
        }
        delay::FreeRtos::delay_ms(DELAY_S * 1000);
    });

    // Thread to listen for interrupts
    let door_switch_t2 = door_switch.clone();
    let interrupt_triggered_t2 = interrupt_triggered.clone();
    let thread2 = thread::spawn(move || loop {
        door_switch_t2.lock().unwrap().enable_interrupt().unwrap();
        while !*interrupt_triggered_t2.lock().unwrap() {
            delay::FreeRtos::delay_ms(1);
        }
        delay::FreeRtos::delay_ms(50);
        let level = door_switch_t2.lock().unwrap().is_high();
        *interrupt_triggered_t2.lock().unwrap() = false;
        log::info!("Thread 2: GPIO3 level is {}", level);
    });

    // Wait for threads to finish (they won't in this example)
    thread1.join().unwrap();
    thread2.join().unwrap();

    unreachable!();
}

// Current state:
// - Notifications do not work in threads due to sharing of *const ()
// - Tried wrapping notification and notifier in arc mutex (no compile due to reason above)
// - Tried mpsc instead of notifications (crash)
// - NEXT: Perhaps the notify and yield is the issue? Wrap this in arc mtx
// - NEXT: Perhaps the delay::BLOCK is an issue? Wrap this in arc mtx
// - NEXT: Try a global variable to check in the thread2. https://dev.to/theembeddedrustacean/esp32-standard-library-embedded-rust-gpio-interrupts-2j2i. This method abandons the task notifications.

// General approach:
// 1. Read from the door sensor
// 2. Set up the interrupt for the door sensor
// 3. Read from the temp/humi sensor using the crate
// 4. Setup the telemetry - WiFi
// 5. Setup the telemetry - MQTT

//In pseudocode, the `main` file will do something like this:
//
//// Do this once
//void setup() {
//    sensor_init();
//    wifi_connect();
//    mqtt_connect();
//}
//
//// Do this repeatedly
//void loop() {
//
//    // Stay connected to WiFi network and MQTT broker
//    connection_handler();
//
//    // Take readings and publish them to the MQTT broker
//    sensor_read();
//    mqtt_publish();
//}
