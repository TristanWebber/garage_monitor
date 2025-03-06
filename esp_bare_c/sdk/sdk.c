#include "sdk.h"
#include <stdint.h>

/////////////////////////////////////////
// Functions for timers                //
/////////////////////////////////////////

// Disable all watchdog timers
void disable_wdt(void) {
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

// Set TIMG0 as digital single stage WDT with timeout_ms timer
void init_wdt(uint32_t timeout_ms) {
    // Disable write protect
    REG_RW(TIMER_GROUP_0, 0x0064) = 0x50D83AA1;

    // Disable WDT and WDT stages (clear bits 23 to 31)
    REG_RW(TIMER_GROUP_0, 0x48) &= ~(0x1FF << 23);
    // Force register update (set bit 22)
    REG_RW(TIMER_GROUP_0, 0x48) |= BIT(22);

    // Set values
    // Increase reset signal pulses to max duration (set bits 15 to 20)
    REG_RW(TIMER_GROUP_0, 0x48) |= (0x3F << 0xF);

    // Set clock source as XTAL (set bit 21)
    REG_RW(TIMER_GROUP_0, 0x48) |= BIT(21);

    // Enable the clock (set bit 29)
    REG_RW(TIMER_GROUP_0, 0xFC) |= BIT(29);

    // Set the Stage 0 WDT to reset system after timeout (set bits 29 and 30)
    REG_RW(TIMER_GROUP_0, 0x48) |= (3 << 29);

    // Set the prescaler
    REG_RW(TIMER_GROUP_0, 0x4C) |= (MWDT_PRESCALER << 16);

    // Set the timeout value to timeout_ms in MWDT ticks
    REG_RW(TIMER_GROUP_0, 0x50) = timeout_ms * 1000 / MWDT_TICKS_PER_US;

    // Enable the MWDT timer (set bit 31)
    REG_RW(TIMER_GROUP_0, 0x48) |= BIT(31);

    // Force register update (set bit 22)
    REG_RW(TIMER_GROUP_0, 0x48) |= BIT(22);

    // Enable write protect
    REG_RW(TIMER_GROUP_0, 0x0064) = 1;
}

// Feed TIMG0 WDT
void feed_wdt(void) {
    // Disable write protect
    REG_RW(TIMER_GROUP_0, 0x0064) = 0x50D83AA1;
    // Feed WDT
    REG_RW(TIMER_GROUP_0, 0x0060) = 1;
    // Enable write protect
    REG_RW(TIMER_GROUP_0, 0x0064) = 1;
}

/////////////////////////////////////////
// Functions for GPIO                  //
/////////////////////////////////////////

// Enable / disable gpio pin
void gpio_enable(uint32_t pin, bool enable) {
    REG_RW(GPIO, 0x20) &= ~BIT(pin);             // Clear BIT(pin)
    REG_RW(GPIO, 0x20) |= enable << pin;         // Set BIT(pin)
}

// TRM 5.5.3:
// Set GPIO matrix GPIO_FUNCn_OUT_SEL with a special peripheral index 128 (0x80)
void gpio_set_output(uint32_t pin) {
    REG_RW(GPIO, (0x554 + (4 * pin))) = BIT(9) | 128;
    gpio_enable(pin, true);
}

// TRM 5.5.3:
// Set the corresponding bit in GPIO_OUT_REG register to the desired GPIO output value
void gpio_set_level(uint32_t pin, bool level) {
    REG_RW(GPIO, 0x4) &= ~BIT(pin);              // Clear BIT(pin)
    REG_RW(GPIO, 0x4) |= level << pin;           // Set BIT(pin)
}

// TRM 5.4.4:
// Enable input and enable pulldown (IO_MUX_GPIOx_FUN_IE IO_MUX_GPIOx_FUN_WPD)
void gpio_set_input_pulldown(uint32_t pin) {
    REG_RW(GPIO, 0x20) &= ~BIT(pin);
    REG_RW(IO_MUX, (0x0004 + (4 * pin))) = BIT(9) | BIT(10);
}

// Return the current digital state of a gpio input
bool gpio_read(uint32_t pin) {
    return REG_RW(GPIO, 0x3C) & BIT(pin);
}

/////////////////////////////////////////
// Functions for blocking delays       //
/////////////////////////////////////////

// Perform `count` "NOP" operations
void spin(volatile uint64_t count) {
    while (count--) asm volatile("nop");
}

// Get system ticks from system timer
uint64_t systick(void) {
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
uint64_t uptime_us(void) {
    return systick() >> 4;
}

// Blocking delay in us
void delay_us(uint64_t us) {
    uint64_t until = uptime_us() + us;
    while (uptime_us() < until){
        // Feed every 0x2000 us. Bit bashing to avoid the compiler linking 64bit division function
        if ((uptime_us() & 0x3FFF) == 0x2000) {
            feed_wdt();
        }
        spin(1);
    }
}

// Blocking delay in ms
void delay_ms(uint64_t ms) {
    delay_us(ms * 1000);
}

/////////////////////////////////////////
// Functions for USB interface         //
/////////////////////////////////////////

// Check if the USB serial fifo buffer is full
bool usb_fifo_full() {
    return (REG_RW(USB_SERIAL, USB_SERIAL_JTAG_EP1_CONF_REG) & BIT(1)) == 0;
}

// Force a flush of the USB serial fifo buffer
void usb_fifo_flush() {
    REG_RW(USB_SERIAL, USB_SERIAL_JTAG_EP1_CONF_REG) |= BIT(0);
}

// Wait for the usb fifo buffer to drain (blocking)
bool usb_wait_for_flush() {
    uint32_t timeout_iterations = 10000;
    while (timeout_iterations--) {
        if (!usb_fifo_full()) {
            return true;
        }
    }
    return false;
}

// Push byte onto the usb serial fifo buffer
void usb_write_byte(uint8_t byte_to_send) {
    REG_RW(USB_SERIAL, USB_SERIAL_JTAG_EP1_REG) = byte_to_send;
}

// Print a cstring to the USB serial device
int usb_print(char *bytes_to_send) {
    int res = -1;
    // Wait for buffer to be ready
    if (!usb_wait_for_flush()) {
        return res;
    }

    // Write each byte to the buffer
    for (int i = 0; bytes_to_send[i] != '\0'; i++) {
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

// Print the binary representation of a 32bit value
void usb_print_reg_bits(uint32_t reg_val) {
    usb_print("Register values: 0b");
    char reg_char[33] = {'1'};
    reg_char[32] = '\0';
    for(uint32_t i = 32; i > 0; i--) {
        reg_char[i - 1] = '0' + (uint8_t)(reg_val & 0x1);
        reg_val >>= 1;
    }
    usb_print(reg_char);
    usb_print("\r\n");
}

/////////////////////////////////////////
// Functions for interrupts            //
/////////////////////////////////////////

// C handlers associated with CPU interrupts, with their arguments
struct irq_data irq_data[MAX_IRQ];

// Create 32-entry vector table with correct byte alignment and no epilogue
__attribute__((aligned(256), __naked__)) void vector_table(void) {
    asm("j panic_handler");
    asm(".rept 31");
    asm("j interrupt_handler");
    asm(".endr");
}

// Set machine trap vector, and set vector mode
void interrupt_init(void) {
    CSR_WRITE(mtvec, (uint32_t)vector_table | 1);

    // Globally enable interrupts: Set 4th bit of mstatus register
    CSR_WRITE(mstatus, 0x8);
}

// Print exception code and loop
__attribute__((interrupt)) void panic_handler(void) {
    // Exception cause is last 5 bits of mcause.
    uint32_t mcause = CSR_READ(mcause) & 0b11111;
    char *panic_template = "00\r\n";

    // Convert code to ascii
    panic_template[0] = (mcause % 10) + '0';
    mcause /= 10;
    panic_template[1] = (mcause % 10) + '0';

    // Print message, loop forever
    usb_print("Panic occurred. Exception code: ");
    usb_print(panic_template);
    while(1);
}

// Clear interrupt and call handler if it exists
__attribute__((interrupt)) void interrupt_handler(void) {
    // Exception cause is last 5 bits of mcause.
    uint32_t trig_irq = CSR_READ(mcause) & 0b11111;

    if (trig_irq < MAX_IRQ) {
        struct irq_data *handler_item = &irq_data[trig_irq];

        // Call user handler if it exists
        if (handler_item->handler){
            handler_item->handler(handler_item->param);
        }
    }
}

int32_t cpu_alloc_interrupt(uint32_t prio) {
    // Track the number of interrupts allocated
    static uint32_t allocated;

    // Return early if no free CPU interrupts are available
    if (allocated == (MAX_IRQ - 1)) {
        return -1;
    }

    // Increment to new interrupt index
    allocated++;

    // Save and clear global interrupt enable (bit 4)
    uint32_t prev_mstatus = CSR_READ(mstatus);
    CSR_WRITE(mstatus, prev_mstatus |= ~(0x8));

    // Enable the interrupt
    REG_RW(INTERRUPT, 0x104) |= BIT(allocated);
    // Set interrupt priority
    REG_RW(INTERRUPT, 0x118 + (4 * (allocated - 1))) = prio;

    // Wait for pending write instructions to complete
    asm volatile ("fence");
    // Restore previous global interrupt state
    CSR_WRITE(mstatus, prev_mstatus);
    return allocated;
}

void gpio_clear_interrupt(uint32_t pin) {
    REG_RW(GPIO, 0x0044) &= ~BIT(pin);
}

void gpio_set_irq_handler(uint32_t pin, void (*handler)(void *), void *param) {
    // Allocate a CPU interrupt, attach handler
    uint32_t no = cpu_alloc_interrupt(1);
    irq_data[no].handler = handler;
    irq_data[no].param = param;

    // Enable GPIO interrupt to detect any edge
    REG_RW(GPIO, (0x74 + 4 * pin)) |= (3U << 7) | BIT(13);

    // Save and clear global interrupt enable (bit 4)
    uint32_t prev_mstatus = CSR_READ(mstatus);
    CSR_WRITE(mstatus, prev_mstatus |= ~(0x8));

    // Map GPIO IRQ to CPU
    REG_RW(INTERRUPT, 0x40) = (uint32_t) no;
    // Wait for pending write instructions to complete
    asm volatile ("fence");
    // Restore previous global interrupt state
    CSR_WRITE(mstatus, prev_mstatus);
}
