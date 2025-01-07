#ifndef SDK_H
#define SDK_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define BIT(x) ((uint32_t) 1U << (x))
#define REG_RW(base, offset) (*(volatile uint32_t *) ((base) + (offset)))

// Register base addresses
#define GPIO          0x60004000
#define LOW_POWER_MGT 0x60008000
#define IO_MUX        0x60009000
#define TIMER_GROUP_0 0x6001F000
#define TIMER_GROUP_1 0x60020000
#define SYSTEM_TIMER  0x60023000
#define USB_SERIAL    0x60043000

// USB register offsets
#define USB_SERIAL_JTAG_EP1_REG      0x0000
#define USB_SERIAL_JTAG_EP1_CONF_REG 0x0004

// Gpio numbers
#define GPIO_NUM_2 2
#define GPIO_NUM_3 3

/////////////////////////////////////////
// Wrapper functions for timers        //
/////////////////////////////////////////

// Disable all watchdog timers
static inline void disable_wdt(void) {
    // Disable RTC WDT (TRM 12.2.2.3)
    REG_RW(LOW_POWER_MGT, 0xA8) = 0x50D83AA1;    // Disable write protection
    REG_RW(LOW_POWER_MGT, 0x90) &= ~BIT(31);     // Clear BIT31 to disable RTC WDT
    REG_RW(LOW_POWER_MGT, 0xA8) = 0x0;           // Re-enable write protection

    // Configure super WDT auto feed (TRM 12.3.2.2)
    REG_RW(LOW_POWER_MGT, 0xB0) = 0x8F1D312A;    // Disable write protection
    REG_RW(LOW_POWER_MGT, 0xAC) |= BIT(31);      // Set BIT31 to enable auto feed
    REG_RW(LOW_POWER_MGT, 0xB0) = 0x0;           // Re-enable write protection

    // Disable Timer Group 0 WDT (TRM 12.2.2.3)
    // TG0 and TG1 are write enabled by default
    REG_RW(TIMER_GROUP_0, 0xFC) &= ~BIT(29);     // Clear BIT29 to disable WDT clock
    REG_RW(TIMER_GROUP_0, 0x48) &= ~BIT(31);     // Clear BIT31 to disable TG0 WDT

    // Disable Timer Group 1 WDT (TRM 12.2.2.3)
    REG_RW(TIMER_GROUP_1, 0xFC) &= ~BIT(29);     // Clear BIT29 to disable WDT's clock
    REG_RW(TIMER_GROUP_1, 0x48) &= ~BIT(31);     // Clear BIT 31 to disable TG1 WDT
}

/////////////////////////////////////////
// Wrapper functions for GPIO          //
/////////////////////////////////////////

// Enable / disable gpio pin
static inline void gpio_enable(int pin, bool enable) {
    REG_RW(GPIO, 0x20) &= ~BIT(pin);             // Clear BIT(pin)
    REG_RW(GPIO, 0x20) |= enable << pin;         // Set BIT(pin)
}

// TRM 5.5.3:
// Set GPIO matrix GPIO_FUNCn_OUT_SEL with a special peripheral index 128 (0x80)
static inline void gpio_set_output(int pin) {
    REG_RW(GPIO, (0x554 + (4 * pin))) = BIT(9) | 128;
    gpio_enable(pin, true);
}

// TRM 5.5.3:
// Set the corresponding bit in GPIO_OUT_REG register to the desired GPIO output value
static inline void gpio_set_level(int pin, bool level) {
    REG_RW(GPIO, 0x4) &= ~BIT(pin);              // Clear BIT(pin)
    REG_RW(GPIO, 0x4) |= level << pin;           // Set BIT(pin)
}

// TRM 5.4.4:
// Enable input and enable pulldown (IO_MUX_GPIOx_FUN_IE IO_MUX_GPIOx_FUN_WPD)
static inline void gpio_set_input_pulldown(int pin) {
    REG_RW(GPIO, 0x20) &= ~BIT(pin);
    REG_RW(IO_MUX, (0x0004 + (4 * pin))) = BIT(9) | BIT(10);
}

// Return the current digital state of a gpio input
static inline bool gpio_read(int pin) {
    return REG_RW(GPIO, 0x3C) & BIT(pin);
}

/////////////////////////////////////////
// Functions for blocking delays       //
/////////////////////////////////////////

// Perform `count` "NOP" operations
static inline void spin(volatile uint64_t count) {
    while (count--) asm volatile("nop");
}

// Get system ticks from system timer
static inline uint64_t systick(void) {
    // Request systicks (TRM 10.5)
    REG_RW(SYSTEM_TIMER, 0x4) = BIT(30);
    // Wait for systicks to be available
    while((REG_RW(SYSTEM_TIMER, 0x4) & BIT(29)) == 0) {
        spin(1);
    }
    // Upper bits from offset 0x0040, lower bits from 0x0044
    return ((uint64_t) REG_RW(SYSTEM_TIMER, 0x40) << 32) | REG_RW(SYSTEM_TIMER, 0x44);
}

// Convert 16MHz systicks to microseconds
static inline uint64_t uptime_us(void) {
    return systick() >> 4;
}

// Blocking delay in us
static inline void delay_us(uint64_t us) {
    uint64_t until = uptime_us() + us;
    while (uptime_us() < until) spin(1);
}

// Blocking delay in ms
static inline void delay_ms(uint64_t ms) {
    delay_us(ms * 1000);
}

/////////////////////////////////////////
// Wrapper functions for USB interface //
/////////////////////////////////////////

// Check if the USB serial fifo buffer is full
static inline bool usb_fifo_full() {
    return (REG_RW(USB_SERIAL, USB_SERIAL_JTAG_EP1_CONF_REG) & BIT(1)) == 0;
}

// Force a flush of the USB serial fifo buffer
static inline void usb_fifo_flush() {
    REG_RW(USB_SERIAL, USB_SERIAL_JTAG_EP1_CONF_REG) |= BIT(0);
}

// Wait for the usb fifo buffer to drain (blocking)
static inline bool usb_wait_for_flush() {
    int timeout_iterations = 10000;
    while (timeout_iterations--) {
        if (!usb_fifo_full()) {
            return true;
        }
    }
    return false;
}

// Push byte onto the usb serial fifo buffer
static inline void usb_write_byte(uint8_t byte_to_send) {
    REG_RW(USB_SERIAL, USB_SERIAL_JTAG_EP1_REG) = byte_to_send;
}

// Print a cstring to the USB serial device
static inline int usb_print(char *bytes_to_send) {
    int res = -1;
    // Wait for buffer to be ready
    if (!usb_wait_for_flush()) {
        return res;
    }

    // Write each byte to the buffer
    for (int i = 0; i < (int)strlen(bytes_to_send); i++) {
        if (usb_fifo_full()) {
            usb_fifo_flush();

            if (!usb_wait_for_flush()) {
                return res;
            }
        }
        usb_write_byte(bytes_to_send[i]);
        res = i;
    }

    // Force a flush to ensure all characters are processed
    usb_fifo_flush();
    return res;
}

#endif /* SDK_H */
