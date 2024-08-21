#![no_std]
#![no_main]

use core::cell::RefCell;
use critical_section::Mutex;

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{Event, Gpio3, Input, Io, Pull},
    interrupt::{self, Priority},
    peripherals::{Interrupt, Peripherals, TIMG0},
    prelude::*,
    system::SystemControl,
    timer::timg::{Timer, Timer0, TimerGroup},
};
use esp_println::println;

use dht_sensor::{dht22, DhtReading};

mod dht_adapter;
use dht_adapter::{DelayAdapter, SensorAdapter};

static DOOR_SWITCH: Mutex<RefCell<Option<Input<Gpio3>>>> = Mutex::new(RefCell::new(None));
static DEBOUNCE_TIMER: Mutex<RefCell<Option<Timer<Timer0<TIMG0>, esp_hal::Blocking>>>> = Mutex::new(RefCell::new(None));

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

    // Read and log sensor results periodically
    let delay = Delay::new(&clocks);
    let mut delay_adapter = DelayAdapter::new(&clocks);
    loop {
        // Read door state
        let mut door_state = false;
        critical_section::with(|cs| {
            door_state = DOOR_SWITCH
                .borrow_ref(cs)
                .as_ref()
                .unwrap()
                .is_high();
        });

        // Read DHT22 sensor
        match dht22::Reading::read(&mut delay_adapter, &mut sensor_pin) {
            Ok(read) => {
                println!("Temperature: {}, Humidity: {}", read.temperature, read.relative_humidity);
            },
            Err(e) => {
                println!("Failed to read DHT sensor. Reason: {:?}", e);
            },
        }

        println!("Door is closed?: {}", door_state);
        delay.delay_millis(300_000);
    }
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
