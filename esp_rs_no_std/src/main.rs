#![no_std]
#![no_main]

use core::{
    cell::RefCell,
    fmt::Write,
    sync::atomic::{AtomicBool, Ordering},
};
use critical_section::Mutex;

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{Event, Gpio3, Input, Io, Pull},
    interrupt::{self, Priority},
    peripherals::{Interrupt, Peripherals, TIMG0},
    prelude::*,
    rng::Rng,
    system::SystemControl,
    timer::{systimer::SystemTimer, timg::{Timer, Timer0, TimerGroup}, PeriodicTimer},
};
use esp_println::println;
use esp_wifi::wifi::{
    AuthMethod,
    ClientConfiguration,
    Configuration,
    utils::create_network_interface,
//    WifiError,
    WifiStaDevice,
    WifiState,
};
use esp_wifi::wifi_interface::WifiStack;
use esp_wifi::{current_millis, initialize, EspWifiInitFor};
use heapless::String;
use smoltcp::iface::SocketStorage;
use smoltcp::wire::{IpAddress, Ipv4Address};

use dht_sensor::{dht22, DhtReading};

mod config;
mod dht_adapter;
use dht_adapter::{DelayAdapter, SensorAdapter};

mod mqtt_client;
use mqtt_client::MqttClient;

static DOOR_SWITCH: Mutex<RefCell<Option<Input<Gpio3>>>> = Mutex::new(RefCell::new(None));
static DEBOUNCE_TIMER: Mutex<RefCell<Option<Timer<Timer0<TIMG0>, esp_hal::Blocking>>>> = Mutex::new(RefCell::new(None));
static SWITCH_UPDATED: AtomicBool = AtomicBool::new(false);

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    // Set the interrupt handler for GPIO interrupts.
    io.set_interrupt_handler(gpio_int_handler);

    // Set GPIO3 as an input with internal pulldown resistor
    let mut door_switch = Input::new(io.pins.gpio3, Pull::Down);

    // Move switch gpio to the mutex refcells
    critical_section::with(|cs| {
        door_switch.listen(Event::AnyEdge);
        DOOR_SWITCH.borrow_ref_mut(cs).replace(door_switch);
    });

    // Sets GPIO4 as an input/output
    let mut sensor_pin = SensorAdapter::new(io.pins.gpio4);

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

    // Setup wifi
    let wifi_timer = PeriodicTimer::new(SystemTimer::new(peripherals.SYSTIMER).alarm0.into());
    let wifi_init = initialize(
        EspWifiInitFor::Wifi,
        wifi_timer,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
        &clocks
    ).unwrap();

    let wifi = peripherals.WIFI;
    let mut socket_set_entries: [SocketStorage; 3] = Default::default();
    let (iface, device, mut controller, sockets) = create_network_interface(&wifi_init, wifi, WifiStaDevice, &mut socket_set_entries).unwrap();

    let client_config = Configuration::Client(ClientConfiguration {
        ssid: config::WIFI_SSID.try_into().unwrap(),
        password: config::WIFI_PASS.try_into().unwrap(),
        auth_method: AuthMethod::WPA2Personal,
        ..Default::default()
    });

    let set_cfg_res = controller.set_configuration(&client_config);
    println!("Attempted Wifi configuration. Result: {:?}", set_cfg_res);

    controller.start().unwrap();
    println!("Attempted to start Wifi. State: {:?}", controller.is_started());

    println!("Wifi connected: {:?}", controller.connect());

    // Wait to get connected
    println!("Wait to get connected");
    loop {
        let res = controller.is_connected();
        match res {
            Ok(connected) => {
                if connected {
                    break;
                }
            }
            Err(err) => {
                println!("{:?}", err);
                loop {}
            }
        }
    }
    println!("{:?}", controller.is_connected());

    // Wait for getting an ip address
    let wifi_stack = WifiStack::new(iface, device, sockets, current_millis);
    println!("Wait to get an ip address");
    loop {
        wifi_stack.work();

        if wifi_stack.is_iface_up() {
            println!("Got ip: {:?}", wifi_stack.get_ip_info());
            break;
        }
    }

    // Get network socket and connect mqtt client
    let mut rx_buffer = [0u8; 1536];
    let mut tx_buffer = [0u8; 1536];
    let socket = wifi_stack.get_socket(&mut rx_buffer, &mut tx_buffer);
    let mut mqtt_client = MqttClient::new("esp_no_std", socket, current_millis);
    let broker_addr = IpAddress::Ipv4(Ipv4Address::from_bytes(&config::BROKER_IP_ADDRESS));

    let delay = Delay::new(&clocks);
    let mut delay_adapter = DelayAdapter::new(&clocks);

    // Main application loop
    loop {
        // Set time for next publish based on entry to this loop
        let wait_until = current_millis() + (config::PUBLISH_INTERVAL_S as u64 * 1000);

        // Verify Wifi is still connected
        let wifi_state = esp_wifi::wifi::get_wifi_state();
        println!("Wifi state: {wifi_state:?}");
        match wifi_state {
            WifiState::StaConnected => {},
            _ => controller.connect().unwrap(),
        }

        // Connect to MQTT broker
        println!("Attempting to connect to MQTT broker");
        let conn_err = mqtt_client.connect(broker_addr, config::BROKER_PORT, 15, Some(config::BROKER_USER), Some(config::BROKER_PASS.as_bytes()));
        match conn_err {
            Ok(()) => println!("Connected to MQTT broker"),
            Err(_) => println!("MQTT Connection error"),
        }

        // Read door state
        let mut door_state = false;
        critical_section::with(|cs| {
            door_state = DOOR_SWITCH
                .borrow_ref(cs)
                .as_ref()
                .unwrap()
                .is_high();
        });

        // Publish door state
        println!("Door is closed?: {}", door_state);
        let mut door_string: String<32> = String::new();
        write!(door_string, "{}", door_state).unwrap();
        let pub_err = mqtt_client.publish(config::DOOR_TOPIC, door_string.as_bytes(), mqttrust::QoS::AtMostOnce);
        match pub_err {
            Ok(()) => println!("Publish success"),
            Err(_) => println!("Publish error"),
        }

        // Read and publish DHT22 sensor
        match dht22::Reading::read(&mut delay_adapter, &mut sensor_pin) {
            Ok(read) => {
                println!("Temperature: {}, Humidity: {}", read.temperature, read.relative_humidity);
                let mut temperature_string: String<32> = String::new();
                write!(temperature_string, "{:.2}", read.temperature).unwrap();
                let pub_err = mqtt_client.publish(config::TEMP_TOPIC, temperature_string.as_bytes(), mqttrust::QoS::AtMostOnce);
                match pub_err {
                    Ok(()) => println!("Publish success"),
                    Err(_) => println!("Publish error"),
                }
                let mut humidity_string: String<32> = String::new();
                write!(humidity_string, "{:.2}", read.relative_humidity).unwrap();
                let pub_err = mqtt_client.publish(config::HUMI_TOPIC, humidity_string.as_bytes(), mqttrust::QoS::AtMostOnce);
                match pub_err {
                    Ok(()) => println!("Publish success"),
                    Err(_) => println!("Publish error"),
                }
            },
            Err(e) => {
                println!("Failed to read DHT sensor. Reason: {:?}", e);
            },
        }

        // Busy loop
        while current_millis() < wait_until {

            // Check if door has been updated
            let mut updated = false;
            let mut door_state = false;
            critical_section::with(|cs| {
                updated = SWITCH_UPDATED.load(Ordering::SeqCst);
                if updated {
                    SWITCH_UPDATED.store(false, Ordering::SeqCst);
                    door_state = DOOR_SWITCH
                        .borrow_ref(cs)
                        .as_ref()
                        .unwrap()
                        .is_high();
                }
            });

            // Publish door state if update was observed
            if updated {
                let mut door_string: String<32> = String::new();
                write!(door_string, "{}", door_state).unwrap();
                let pub_err = mqtt_client.publish(config::DOOR_TOPIC, door_string.as_bytes(), mqttrust::QoS::AtMostOnce);
                match pub_err {
                    Ok(()) => println!("Publish success"),
                    Err(_) => println!("Publish error"),
                }
            } else {
                mqtt_client.poll();
            }
            delay.delay_millis(1000);
        }
    }

    // Read and log sensor results periodically
    // Application code START
    //loop {
    //    delay.delay_millis(300_000);
    //}
    // Application code END
}

// GPIO Interrupt handler unlistens, clears interrupt and starts debounce timer
#[handler]
fn gpio_int_handler() {
    critical_section::with(|cs| {
        // Clear GPIO interrupts
        let mut door_switch = DOOR_SWITCH.borrow_ref_mut(cs);
        let door_switch = door_switch.as_mut().unwrap();
        door_switch.clear_interrupt();
        door_switch.unlisten();

        // Start debounce timer
        let mut debounce_timer = DEBOUNCE_TIMER.borrow_ref_mut(cs);
        let debounce_timer = debounce_timer.as_mut().unwrap();
        debounce_timer.load_value(config::DEBOUNCE_MS.millis()).unwrap();
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
        SWITCH_UPDATED.store(true, Ordering::SeqCst);
        println!("Interrupt triggered. Door is closed?: {}", door_switch.is_high());

        // Relisten for GPIO interrupts
        door_switch.listen(Event::AnyEdge);
    });
}
