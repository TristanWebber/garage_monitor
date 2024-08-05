use anyhow::Result;

use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

// Peripherals
use esp_idf_svc::hal::delay;
use esp_idf_svc::hal::gpio::{InterruptType, PinDriver, Pull};
use esp_idf_svc::hal::peripherals::Peripherals;

use dht_sensor::{DhtReading, dht22};

// Networking
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::mqtt::client::QoS;

mod config;
mod connections;

use connections::{wifi_create, mqtt_create};

fn main() -> Result<()> {
    // Boilerplate for ESP std Rust projects
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    // Takes ownership of the Peripherals struct
    let peripherals = Peripherals::take()?;

    // Configure event loop and nvs for wifi
    let event_loop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;

    // Connect wifi
    let modem = peripherals.modem;
    let _wifi_client = wifi_create(&event_loop, &nvs, modem)?;
    thread::sleep(Duration::from_secs(3));

    // Connect mqtt
    let (mqtt_client, mut mqtt_conn) = mqtt_create(config::BROKER_URI, config::BROKER_USER, config::BROKER_PASS)?;

    thread::spawn(move || {
        log::info!("MQTT Listening for messages");
        while let Ok(event) = mqtt_conn.next() {
            log::info!("[Queue] Event: {}",event.payload());
        }
        log::info!("MQTT connection loop exit");
    });

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
    let mqtt_client = Arc::new(Mutex::new(mqtt_client));

    // Read and log the level of the reed switch, then yield for PUBLISH_INTERVAL seconds
    let door_switch_t1 = door_switch.clone();
    let mqtt_client_t1 = mqtt_client.clone();
    let thread1 = thread::spawn(move || loop {
        // Read from the door switch
        let level = door_switch_t1.lock().unwrap().is_high();
        log::info!("Thread 1: GPIO3 level is {}", level);

        // Read from the dht sensor
        let mut dht_delay = delay::Ets;
        sensor_pin.set_high().unwrap();
        thread::sleep(Duration::from_millis(1100));
        match dht22::Reading::read(&mut dht_delay, &mut sensor_pin) {
            Ok(read) => {
                log::info!("Thread 1: Temperature: {}, Humidity: {}", read.temperature, read.relative_humidity);
                let mut mqtt_publisher = mqtt_client_t1.lock().unwrap();
                mqtt_publisher.publish(&config::TEMP_TOPIC, QoS::AtMostOnce, false, read.temperature.to_string().as_bytes()).unwrap();
                mqtt_publisher.publish(&config::HUMI_TOPIC, QoS::AtMostOnce, false, read.relative_humidity.to_string().as_bytes()).unwrap();
                mqtt_publisher.publish(&config::DOOR_TOPIC, QoS::AtMostOnce, false, level.to_string().as_bytes()).unwrap();
            },
            Err(e) => {
                log::error!("Thread 1: Failed to read DHT sensor. Error: {:?}", e);
                mqtt_client_t1.lock().unwrap().publish(&config::DOOR_TOPIC, QoS::AtMostOnce, false, level.to_string().as_bytes()).unwrap();
            },
        }
        delay::FreeRtos::delay_ms(config::PUBLISH_INTERVAL * 1000);
    });

    // Thread to listen for interrupts
    let door_switch_t2 = door_switch.clone();
    let mqtt_client_t2 = mqtt_client.clone();
    let interrupt_triggered_t2 = interrupt_triggered.clone();
    let thread2 = thread::spawn(move || loop {
        door_switch_t2.lock().unwrap().enable_interrupt().unwrap();
        while !*interrupt_triggered_t2.lock().unwrap() {
            delay::FreeRtos::delay_ms(1);
        }
        delay::FreeRtos::delay_ms(config::DEBOUNCE_MS);
        let level = door_switch_t2.lock().unwrap().is_high();
        *interrupt_triggered_t2.lock().unwrap() = false;
        log::info!("Thread 2: GPIO3 level is {}", level);
        mqtt_client_t2.lock().unwrap().publish(&config::DOOR_TOPIC, QoS::AtMostOnce, false, level.to_string().as_bytes()).unwrap();
    });

    // Wait for threads to finish (they won't in this example)
    thread1.join().unwrap();
    thread2.join().unwrap();

    unreachable!();
}
