#![no_std]
#![no_main]

use core::cell::RefCell;
use critical_section::Mutex;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{Event, Gpio3, Input, Io, Pull},
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
};
use esp_println::println;

static DOOR_SWITCH: Mutex<RefCell<Option<Input<Gpio3>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    // Set the interrupt handler for GPIO interrupts.
    io.set_interrupt_handler(handler);

    // Set GPIO3 as an input with internal pulldown resistor
    let mut door_switch = Input::new(io.pins.gpio3, Pull::Down);

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
