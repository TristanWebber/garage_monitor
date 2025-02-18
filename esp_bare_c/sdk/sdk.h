#ifndef SDK_H
#define SDK_H

#include <stdbool.h>
#include <stdint.h>

// Macros for normal register and DMA operations
#define BIT(x) ((uint32_t) 1U << (x))
#define REG_RW(base, offset) (*(volatile uint32_t *) ((base) + (offset)))

// Macros for CSR register operations
#define CSR_WRITE(reg, val) ({ asm volatile("csrw " #reg ", %0" ::"rK"(val)); })
#define CSR_READ(reg) ({                       \
    uint32_t v_;                               \
    asm volatile("csrr %0, " #reg : "=r"(v_)); \
    v_;                                        \
})
#define CSR_SETBITS(reg, cm, sm) CSR_WRITE(reg, (CSR_READ(reg) & ~(cm)) | (sm))

#define MAX_IRQ           32
#define MWDT_TICKS_PER_US 500
#define MWDT_PRESCALER    20000

// Register base addresses
#define GPIO          0x60004000
#define LOW_POWER_MGT 0x60008000
#define IO_MUX        0x60009000
#define TIMER_GROUP_0 0x6001F000
#define TIMER_GROUP_1 0x60020000
#define SYSTEM_TIMER  0x60023000
#define USB_SERIAL    0x60043000
#define INTERRUPT     0x600C2000

// USB register offsets
#define USB_SERIAL_JTAG_EP1_REG      0x0000
#define USB_SERIAL_JTAG_EP1_CONF_REG 0x0004

// Gpio numbers
#define GPIO_NUM_2 2
#define GPIO_NUM_3 3

/////////////////////////////////////////
// Functions for timers                //
/////////////////////////////////////////

// Disable all watchdog timers
void disable_wdt(void);

// Set TIMG0 as digital single stage WDT with timeout_ms timer
void init_wdt(uint32_t timeout_ms);

// Feed TIMG0 WDT
void feed_wdt(void);

/////////////////////////////////////////
// Functions for GPIO                  //
/////////////////////////////////////////

// Enable / disable gpio pin
void gpio_enable(uint32_t pin, bool enable);

// Set gpio pin as an output
void gpio_set_output(uint32_t pin);

// Set the digital output level of gpio pin
void gpio_set_level(uint32_t pin, bool level);

// Enable gpio pin as an input with internal pulldown resistor
void gpio_set_input_pulldown(uint32_t pin);

// Return the current digital state of a gpio input
bool gpio_read(uint32_t pin);

/////////////////////////////////////////
// Functions for blocking delays       //
/////////////////////////////////////////

// Perform `count` "NOP" operations
void spin(volatile uint64_t count);

// Get system ticks from system timer
uint64_t systick(void);

// Convert 16MHz systicks to microseconds
uint64_t uptime_us(void);

// Blocking delay in us
void delay_us(uint64_t us);

// Blocking delay in ms
void delay_ms(uint64_t ms);

/////////////////////////////////////////
// Functions for USB interface         //
/////////////////////////////////////////

// Check if the USB serial fifo buffer is full
bool usb_fifo_full();

// Force a flush of the USB serial fifo buffer
void usb_fifo_flush();

// Wait for the usb fifo buffer to drain (blocking)
bool usb_wait_for_flush();

// Push byte onto the usb serial fifo buffer
void usb_write_byte(uint8_t byte_to_send);

// Print a cstring to the USB serial device
int usb_print(char *bytes_to_send);

// Print the binary representation of a 32bit value
void usb_print_reg_bits(uint32_t reg_val);

/////////////////////////////////////////
// Functions for interrupts            //
/////////////////////////////////////////

struct irq_data {
    void (*handler)(void *);
    void *param;
};

void interrupt_init(void);

int32_t cpu_alloc_interrupt(uint32_t prio);

void gpio_clear_interrupt(uint32_t pin);

void gpio_set_irq_handler(uint32_t pin, void (*handler)(void *), void *param);

#endif /* SDK_H */
