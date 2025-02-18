# C Bare Metal Implementation

- [Introduction](#introduction)
- [Quickstart](#quickstart)
- [The Strategy](#the-strategy)
- [Getting Started](#getting-started)
- [Blinky base example](#blinky-base-example)
- [ESP32-C3 Boot Process](#esp32-c3-boot-process)
- [Build System](#build-system)
- [GPIO read and logging](#gpio-read-and-logging)
- [Introducing interrupts](#introducing-interrupts)
- [Housekeeping](#housekeeping)
- [Building and Flashing](#building-and-flashing)
- [Observations and Next Steps](#observations-and-next-steps)

## Introduction

This is the final version of the garage monitor application. To date, the application has been implemented to a minimally functional state in a number of ways, demonstrating the relative strengths, weaknesses and level of maturity in those development pathways. This final implementation is an exception, in that it will NOT reach the point of feature parity with the other options. This is simply an experiment in truly starting from scratch.

This version of the project has some philosophical similarity with the `no-std` Rust version of the project, in that our user code starts as early in the ESP32 boot process as possible, and does not rely on an operating system to orchestrate tasks. As a result, the project will be entirely impractical, and will serve no purpose aside from the satisfaction of curiosity and a bit of education.

The topics that will be covered will be:
- The ESP32-C3 boot process
- Building and linking from scratch
- Controlling the ESP32-C3 and its peripherals by reading and writing from registers
- Hardware interrupts from scratch

## Quickstart

This series of projects is presented as a demonstration and discussion piece, however if you simply want to take some or all of the repository and use it for your own projects without suffering through the narrative you can:

0. [Acquire and assemble](https://github.com/TristanWebber/garage_monitor/blob/main/README.md#hardware) the appropriate hardware
1. Ensure you have the [ESP-IDF and toolchain](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html) installed
2. Clone the repository. If you are only interested in this subproject, use `sparse-checkout`
```bash
git clone --no-checkout https://github.com/TristanWebber/garage_monitor.git && \
cd garage_monitor && \
git sparse-checkout esp_bare_c && \
git checkout
```
3. [Build and flash](#building-and-flashing) the project to your microcontroller

## The strategy

This project will start off by reviewing the documentation for the ESP32C3, and the documentation and code in the ESP-IDF. The first task will be understanding the path of least resistance to getting our own code running on the chip. This will serve as the initial proof of concept for a from-scratch application. Once our minimal example is ready, the build tools will be set up. To simplify the process, this will use some of the ESP-IDF tools, so whilst the application is independent of the ESP-IDF, the build tools will not be.

Next, the basic example will be built upon to achieve the gpio features similar to the previous versions of the project. Our LED output will be converted to an indicator to show the state of an input, and the reed switch will be configured as the input to a hardware interrupt. We will reference the documentation to understand how hardware interrupts work for this chip (and RISC-V more generally). We will then use the hardware interrupts to implement the desired functionality.

## Getting started

As a path of least resistance, this project will use ESP-IDF tools to compile and flash. So consider the prerequisites of this project as having the [ESP toolchain](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html) installed for the ESP32C3. This subproject was developed in a linux environment, and doesn't explicitly identify any other system dependencies. At the time of writing, v5.3.1 is the stable release.

The items that will be borrowed from the ESP toolchain will be:
- The `riscv-esp-elf-gcc` cross compiler.
- The `esptool.py` tool for flashing and monitoring

To follow along, it will also be handy to have a copy of the [ESP32-C3 Technical Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32-c3_technical_reference_manual_en.pdf) downloaded.

## Blinky base example

The first task will be proving out the concept with as little code as possible using Blinky - flashing an LED on and off by using the GPIO driver. We'll consider this as the path of least resistance to prove that our toolchain works and we have bare metal control of the processor and peripherals. Since this is an embedded project, we don't have `stdio.h` functions like `printf`, so using GPIOs is more straightforward than printing a 'hello world' message to the host. Note that the ESP32 family _does_ actually have `printf` (and other stdio and stdlib functions) burned in to the chip ROM from the factory, however using those functions is not entirely consistent with the bare metal philosophy so we will ignore them.

The starting point of this test is heavily inspired by [mdk](https://github.com/cpq/mdk) - the repo that convinced me this was an idea worth playing with. We will work towards making the following code work:

#### **`main.c`**
```C
#include "sdk.h"

#define LED_PIN GPIO_NUM_2

int led_state = 0;

int main(void) {
    gpio_set_output(LED_PIN);

    for(;;) {
        gpio_set_level(LED_PIN, led_state);
        led_state = !led_state;
        delay_ms(500);
    }

    return 0;
}
```

In order to get to the point where this can work, there are some steps that need to be done:
- Work with the existing boot process
- Write the gpio and delay functions
- Create scripts for our build system

## ESP32-C3 Boot Process

The first hurdle to getting our blinky code on to the chip and running will be to get the CPU to acknowledge its existence and load the first instructions. We can familiarise ourselves, at a high level with the ESP32-C3 boot process by checking the [documentation](https://docs.espressif.com/projects/esp-idf/en/v5.3.1/esp32c3/api-guides/startup.html). Here we see that a first stage bootloader is located on the ROM of the chip and cannot be modified. `The first stage bootloader loads the second-stage bootloader image to RAM (IRAM & DRAM) from flash offset 0x0.`

So on power-up, the following steps occur:
1. Hardware startup and checks occur, then the program counter is directed to memory address 0x4000_0000 - the location of the ROM bootloader
2. The ROM bootloader performs basic configuration tasks, loads code from flash offset 0x0 to IRAM memory and points the program counter to that location<sup>1</sup>
3. Instructions that were located at flash offset 0x0 begin executing

If we were using ESP-IDF, Step 3 would start the ESP-IDF second-stage bootloader, and eventually our `app_main()` would be called. But there's nothing stopping us from placing our own code at that location in flash. For this particular chip, that is the closest to 'bare metal' we can be (the ROM bootloader is immutable and cannot be modified by us).

For our use case, there is no reason to have a second-stage bootloader. Our application is small, so it will be entirely in memory. This means that there's very little that needs to be done to facilitate our own `main.c` being loaded on to the chip. The bare minimum that needs to be done is to point to our main function. However, it makes sense to hide away the low level code, including the tasks to be performed at boot, so let's create a basic `init` file so that can all be hidden away from the user code.

As a first pass, the only thing to be done before calling main is to disable the watchdog timers (TRM 12) to prevent our chip from getting stuck in a reset loop. We *should* do a bunch of other checks and tasks before loading our user code, but let's just forget about most of that for now because it won't stop our project from working. We would ideally be implementing code to feed the watchdog timers but that's not part of our objective just yet. Our first pass at the init code looks like this, borrowing the function name `call_start_cpu0` from ESP-IDF:

#### **`init.c`**
```C
#include "sdk.h"

extern int main(void);

void __attribute__((noreturn)) call_start_cpu0(void) {
    disable_wdt();
    main();
    for(;;) {}
}
```

And now we get in to our first low level operations to disable the watchdog timers. Looking at Section 12 of the Technical Reference Manual, there's all the detail we need in order to prevent the watchdog timers from intervening in our application:
- There are three digital and one analog watchdogs for us to disable (TRM Figure 12-1). To be safe, let's disable all of them.
- Settings can be modified by writing to bits in the Timer Group and RTC registers (TRM 12.5)
- Write protection is implemented to prevent inadvertent modification of WDT configurations (TRM 12.2.2.3)

This means that we will be interacting with registers - as low as we can go. First, let's assume we'll be doing this a lot, and set some macros to make life easier. From the docs, we know the registers are 32 bit wide, and that we will need to read and write individual bits so we define a BIT macro and a register read/write macro. Also from the documentation, we know that registers for certain peripherals are accessed via base addresses, and defined in the technical reference manual by their offsets, so let's name those base addresses while we're there.

#### **`sdk.h`**
```C
#define BIT(x) ((uint32_t) 1U << (x))
#define REG_RW(base, offset) (*(volatile uint32_t *) ((base) + (offset)))

#define LOW_POWER_MGT 0x60008000
#define TIMER_GROUP_0 0x6001F000
#define TIMER_GROUP_1 0x60020000
```

Following the approach described in the docs, we'll disable write protection, set the bits we need to get our desired behaviour, and re-enable write protection:

#### **`sdk.h`**
```C
void wdt_disable(void) {
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
```

That should be enough setup for now. The other functions for our blinky example (`gpio_set_output()`, `gpio_set_level()`, `delay_ms()`) follow the same pattern - read the manual, define the registers, write a function to manipulate the registers. The only thing worth mentioning is that our blocking delay reads `systicks` from a register, and calls the assembly `nop` pseudo-instruction to spin the CPU until it's time to do work again.

> [!NOTE]
> <sup>1</sup>I reversed the ROM bootloader out of curiosity to see what was happening in that first stage of boot. For reference, the steps are roughly:
> 1. Set a default interrupt and exception handler
> 2. Configure bootloader memory regions
> 3. Configure interface peripherals and CPU settings
> 4. Attach flash by SPI, place user code in memory and start executing user code

## Build system

Our blinky application code is done. In theory, this should work if assembled to riscv instructions and linked in a way that allows the esp32c3 to find the starting point of the application. So to finish off the prototype, let's setup a Makefile and linker script to facilitate that happening.

For these, again we can draw on the reference material of the Technical Reference Manual, and the ESP-IDF Programming Guide to understand how these need to be laid out. Let's start with the linker script. This will make sure the code we have written will be linked with addresses that make sense to the CPU. The important parts of this script are:

- Our iram_seg will start at the address 0x4038_0000 and will be 32kB in size
- Our dram_seg will start immediately after the iram_seg and will be 32kB in size
- ENTRY tells the linker call_start_cpu0 will be the program entry point
- Instructions (anything with .text.* labels) will be placed in the iram_seg
- Data will be placed in the dram_seg

```Linker Script
/* ----snip---- */

MEMORY
{
  iram_seg (RWX) :                  org = 0x40380000, len = 32k
  dram_seg (RW) :                   org = 0x3FC80000 + LENGTH(iram_seg), len = 32k
}

ENTRY(call_start_cpu0)

SECTIONS {
  .text :{
    . = ALIGN(16);
    *(.text)
    *(.text*)
  } > iram_seg

  /* ----snip---- */
}
```

Finally, we can put together a Makefile to automate the build process. This allows some rather verbose command line instructions to be called with the make commands. This will build a elf file with the gcc riscv32 cross compiler provided by esp-idf, create a .bin image with esptool.py, flash the image to the chip using esptool.py and monitor using cu. Nothing magical, but it saves a lot of time and effort.


```make
CFLAGS      ?= -Wall -Wextra -Werror=all \
               -march=rv32imc -mabi=ilp32 \
               -Og -ffunction-sections -fdata-sections \
               -I. -I$(SDK) $(EXTRA_CFLAGS)
LINKFLAGS   ?= -T$(SDK)/link.ld -nostartfiles $(EXTRA_LINKFLAGS)
FLASHFLAGS  ?= --baud 460800 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size 4MB 0x0
TOOLCHAIN   ?= riscv32-esp-elf
SRCS        ?= $(SDK)/*.c $(SOURCES)

build: $(SDK)/../build/firmware.bin

$(SDK)/../build/firmware.elf: $(SRCS)
	$(TOOLCHAIN)-gcc  $(CFLAGS) $(SRCS) $(LINKFLAGS) -o $@

$(SDK)/../build/firmware.bin: $(SDK)/../build/firmware.elf
	esptool.py --chip esp32c3 elf2image $(SDK)/../build/firmware.elf

flash:
	esptool.py -p $(PORT) $(FLASHFLAGS) $(SDK)/../build/firmware.bin

monitor:
	cu -s 115200 -l $(PORT)

clean:
	@rm -rf $(SDK)/../build/*.{bin,elf} $(SDK)/../build/firmware*

erase-flash:
	esptool.py -p $(PORT) -b 460800 --before default_reset --after hard_reset --chip esp32c3 erase_flash
```

For this to work, we need to run the esp_idf export.sh script, and export the path of the SDK and upload port before using. So... The moment of truth... With a LED and 220R resistor wired to GPIO2, we can do the following:

```bash
export SDK=~/your_path_to/garage_monitor/esp_bare_c/sdk
export PORT=/dev/ttyACM0
. $HOME/esp/esp-idf/export.sh

make erase-flash clean build flash
```

...and there's a LED flashing on/off over a 1 second cycle. It's a small reward for all that work setting up our bare metal environment, but it's visual proof that our own instructions are being executed on the chip, from scratch. Let's push through and explore how to read from a GPIO and work with tasks and interrupts.

## GPIO read and logging

If we want to poll a GPIO to read the state of a reed switch, the process is very similar to the previously applied techniques of configuring the GPIO driver to treat that pin as an input by setting register values, and read the state of the pin from another register.

#### **`sdk.h`**
```C
// Set gpio pin as input with internal pulldown resistor
void gpio_set_input_pulldown(int pin) {
    // Clear the output configuration bit
    REG_RW(GPIO, 0x20) &= ~BIT(pin);
    // Configure pin as input (bit 9) and enable pulldown (bit 10)
    REG_RW(IO_MUX, (0x0004 + (4 * pin))) = BIT(9) | BIT(10);
}

// Return the current digital state of a gpio input
static inline bool gpio_read(uint32_t pin) {
    return REG_RW(GPIO, 0x3C) & BIT(pin);
}
```

Using this new code, our `main.c` can easily be updated to poll the input pin and output the state to our output pin containing the LED. That's cool, but why use a microcontroller to do something that a LED in series can also achieve?? One way to justify the use of a microcontroller for such a basic task is to use one of the communication interfaces to output a message to the host. In the case of the Seeed XIAO ESP32C3, the a USB driver is wired up and ready to go, so let's have a look at using that.

By now, the process of using the reference manual should be pretty familiar. Once again, we find the USB driver on the chip is a peripheral that we access by reading and writing to registers. Section 30.3.2 contains the information we need, and some information we don't. The important bits:

```
When the firmware has data to send, it can do so by putting it in the send buffer and triggering a flush, allowing the host to receive the data in a USB packet. In order to do so, there needs to be space available in the send buffer. Firmware can check this by reading USB_REG_SERIAL_IN_EP_DATA_FREE; a one in this register field indicates there is still free room in the buffer. While this is the case, firmware can fill the buffer by writing bytes to the USB_SERIAL_JTAG_EP1_REG register.

Writing the buffer doesn’t immediately trigger sending data to the host. This does not happen until the buffer is flushed; a flush causes the entire buffer to be readied for reception by the USB host at once. A flush can be triggered in two ways: after the 64th byte is written to the buffer, the USB hardware will automatically flush the buffer to the host. Alternatively, firmware can trigger a flush by writing a one to USB_REG_SERIAL_WR_DONE.
```

OK, so a simplistic approach to output a single character to a host will be to:

- Check if there is room in the USB FIFO buffer by reading a bit
- Write a byte (ASCII char) to the USB FIFO buffer
- Write to a register to force the buffer to flush and make the data available to the host

This all seems pretty straightforward - each of those operations can be performed with literally one line of code. However it would be tedious for us to write to this peripheral one byte at a time. We have a couple of options here - we could write a couple of helper functions for newlib and have all of the stdio.h functions available to us (including plenty we won't use), or we could continue with the 'from scratch' mentality. Using newlib would exponentially increase the footprint of our application, so let's go with the 'from scratch' option!

It will be possible to achieve all we need with our own version of `puts` - this would allow writing a whole null terminated string to the buffer. Our higher level function could look like this:

#### **`sdk.h`**
```C
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
```

Note that there is a `usb_wait_for_flush()` that is simply a timeout to prevent this functionality from blocking forever in the event that there is no host connected. This function will return the number of bytes successfully written to the buffer (or -1 if no bytes were written), and this results in some useful feedback to the caller of the function. This is not a particularly efficient function due to all the branching, however it does the job and this processor has plenty of power, so the inefficiencies aren't worth too much of our attention.

Now that we have our own simplistic version of puts, we can update our main code:

#### **`main.c`**
```C
/* ----snip---- */
int led_state = 0;
char *message = "LED state: 0\r\n";

int main(void) {
    gpio_set_output(LED_PIN);
    gpio_set_input_pulldown(SW_PIN);

    for(;;) {
        led_state = gpio_read(SW_PIN);
        gpio_set_level(LED_PIN, led_state);
        message[11] = led_state + '0';
        usb_print(message);
        delay_ms(500);
    }

    return 0;
}
```

 Again, we are reminded that stdio is not present in the embedded environment when we format our message: manipulating a byte in the message directly isn't particularly ergonomic but we're doing limited logging so again, this a tradeoff we're accepting. We can test this and see two things:

- We have some logging capability
- This synchronous code makes it possible that events will be missed

If a short event were to occur during the delay, there would be nothing to notify us it ever occurred. This leads us back to the same solution we have previously considered to ensure we capture transient events on our gpio input - interrupts. Only this time, we will need to implement them at a low level.

## Introducing interrupts

Let's start with the outcome we want to achieve, then move towards how to implement interrupts on RISC-V, and then create a solution.

For our use case we want to:
- Allow an edge-triggered interrupt from a GPIO
- When an interrupt occurs, perform tasks:
    - Read the GPIO state
    - Logging the state
    - Change the indicator LED state

This tells us that we can focus simply on the GPIO peripherals, so our scope can be quite narrow. However, it would still be wise to hide the implementation details away in the sdk, and provide an API that lets a user attach their own handler function. We'll work towards making the following code work:

#### **`main.c`**
```C
/* ----snip---- */
int led_state = 0;
char *message = "LED state: 0\r\n";
char *switch_msg = "Switch: 0\r\n";

static void switch_handler(void *param) {
    uint32_t pin = (uint32_t) (uintptr_t) param;

    // Clear interrupt to prevent repeat triggers of handler
    gpio_clear_interrupt(pin);

    // Read switch state into our global status variable
    led_state = gpio_read(pin);
    switch_msg[8] = led_state + '0';

    // Log data, set LED
    usb_print(switch_msg);
    gpio_set_level(LED_PIN, led_state);
}

int main(void) {
    /* ----snip---- */
    gpio_set_irq_handler(SW_PIN, switch_handler, (void *) SW_PIN);
    /* ----snip---- */
}
```

That is, to attach any edge-triggered interrupt handler to a GPIO pin, all the user will need to do is register the handler against the pin. This satisfies our specific use case, and allows for future addition of more interrupts handlers if we so desire.

### Initiate interrupt controller

Now that the objective is defined, next is to figure out how to achieve this on this specific architecture. Section 1.5 and Chapter 8 of the Technical Reference Manual contain the information we need. The ESP32-C3 has an interrupt controller supporting 31 CPU interrupts, to handle 62 interrupt input sources. This means that the bulk of our work will be in making sure that when an interrupt occurs, our code will determine the cause and then use the appropriate handler.

The 31 CPU interrupts map to a _trap vector_ - effectively a series of function pointers. When a CPU interrupt is triggered, the CPU jumps to the corresponding offset in the trap vector to start handling the interrupt. Our straightforward use case allows us to just use a single handler, so all 31 entries in the vector can be identical. There is also a 0th CPU interrupt reserved for exceptions so let's create a second handler for that one. Our trap vector could be defined in a separate assembly file, or using inline assembly. In this case, we choose the latter to keep the build process as simple as possible. We want the instruction `j panic_handler` as the first row and then `j interrupt_handler` repeated 31 times. The Technical Reference Manual states that the base address needs to be aligned to 256 bytes:

#### **`sdk.c`**
```C
__attribute__((aligned(256))) void vector_table(void) {
    asm("j panic_handler");
    asm(".rept 31");
    asm("j interrupt_handler");
    asm(".endr");
}
```

The CPU requires a pointer to the vector table to be loaded to a control and status register (CSR) `mtvec`, and the address of this vector table must have the first bit set to `1` to indicate vector mode. We can access the CSRs using more inline assembly, via a macro, and we will call this from our init function so the vector table is loaded before any user code starts:

#### **`sdk.c`**
```C
void interrupt_init(void) {
    CSR_WRITE(mtvec, (uint32_t)vector_table | 1);
    // Globally enable interrupts: Set 4th bit of mstatus register
    CSR_WRITE(mstatus, 0x8);
}
```

With this vector in place, if a CPU exception occurs, the `panic_handler` will be triggered, or if a CPU interrupt is triggered, the CPU will jump to the `interrupt_handler` function, so let's move on to that next. We'll keep the panic handler simple - print the exception and loop. Right now, this will result in the processor hanging forever, but when we re-enable our watchdog timer, this behaviour will result in a reset. When entering the interrupt handler, the cpu jumps from the current program flow so we need to preserve registers so that when the normal program flow resumes, data is not corrupted. This is doable by manually saving and restoring the register contents at the entry and exit of the interrupt handler using assembly, or we can lean on the gcc compiler to insert those instructions for us using `__attribute__((interrupt))` on the handler.

#### **`sdk.c`**
```C
__attribute__((interrupt)) void panic_handler(void) {
    // Exception cause is last 5 bits of mcause.
    uint32_t mcause = CSR_READ(mcause) & 0b11111;

    // Convert code to ascii
    char ls_digit = (mcause % 10) + '0';
    mcause /= 10;
    char ms_digit = (mcause % 10) + '0';

    // Print message, loop forever
    usb_print("Panic occurred. Exception code: ");
    usb_write_byte(ls_digit); usb_write_byte(ms_digit);
    usb_write_byte('\r'); usb_write_byte('\n');
    while(1);
}
```

Here, we simply read the value of the exception, log it and loop. When an exception occurs, the kind of exception is placed in the last 5 bits of mcause. Since an exception occurring means that something has likely gone very wrong, no attempt is made to handle it.

The interrupt handler, on the other hand, needs a way to direct the appropriate interrupt number (1-31) to the appropriate user-defined handler. Having an array of up to 32 user-defined handlers and an optional argument to pass to the handler gives the flexibility to achieve this:

#### **`sdk.h`**
```C
struct irq_data {
    void (*handler)(void *);
    void *param;
};
```

#### **`sdk.c`**
```C
struct irq_data irq_data[MAX_IRQ];

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
```

### Setup interrupt user API

The groundwork for interrupts has been set, so finally it's time to make the user API functions.

The setter function needs to be able to attach a handler to a free position in the irq_data array and configure the registers as needed to enable the CPU interrupt and peripheral interrupt. We'll keep it simple and ignore the potential to unset handlers for now. This will mean that we don't need to keep track of what handler was allocated to what position. The technical reference manual provides details on how to set the registers to get the desired outcomes, but additionally provides guidance on additional steps required when modifying the interrupt registers in Section 1.5.3. This involves temporarily disabling interrupts and forcing the CPU to wait for any write operations to be completed while changing the interrupt registers, to avoid synchronisation issues in this state.

#### **`sdk.c`**
```C
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
    asm("fence");
    // Restore previous global interrupt state
    CSR_WRITE(mstatus, prev_mstatus);
}
```

Finally, the gpio_clear_interrupt function is just a wrapper over a single register write operation:

#### **`sdk.c`**
```C
void gpio_clear_interrupt(uint32_t pin) {
    REG_RW(GPIO, 0x0044) &= ~BIT(pin);
}
```

With that in place, our code will finally achieve the original objectives. There is, of course, and endless list of other things that could be added to this project, but this is a good stopping point for the exercise, in terms of functionality.

## Housekeeping

In the interests of getting quickly to something that works, some useful steps have been skipped. Here are some explanations of some final bits of housekeeping.

### Check the processor health before booting

Before booting, we can attempt to read a control and status register that will have a known and constant value, to make sure the processor is working before continuing the boot process. This can be easily done by reading the control and status register `mhartid`, which should return the value 0. If any other value is returned, an infinite blocking loop will be entered.


#### **`init.c`**
```C
// Check CPU health by reading hartid. Value should be 0
uint32_t hartid = CSR_READ(mhartid);
while (hartid);
```

### Clear bss memory

Any variables that are not initialised to a specific value have the potential to be some random state when the memory powers up. To avoid potential issues associated with this, it is common practice to explicitly set any uninitialised values to 0. This would often be achieved using `memset`, however in our bare metal environment, stdlib is not available so we can just loop through bss and manually zero all words in that section of memory.

#### **`init.c`**
```C
uint32_t *this_word = &_sbss;
while (this_word < &_ebss) {
    *this_word = 0;
    this_word++;
}
```

### Enable wdt

Our previous approach of disabling the WDT in order to quickly mobilise the project is not the greatest idea in the long run. A better approach is to introduce a mechanism of feeding the watchdog. One way to achieve this is to feed it using an often-called function. In this case, feeding the wdt in the blocking delays is an ideal place, because the program flow is dependent on the delays, and while in the delay, the processor is just executing 'nop' instructions.

To achieve this functionality, we configure the wdt by calling `init_wdt(timeout_ms)` in the `init.c` file. This function simply sets the registers as required to initiate the wdt and set suitable defaults. Feeding the wdt is a straightforward exercise of writing any value to the appropriate register. We can call this in the `delay_us` function by adding a conditional in the blocking loop:

```C
// Blocking delay in us
void delay_us(uint64_t us) {
    uint64_t until = uptime_us() + us;
    while (uptime_us() < until){
        if (!(uptime_us() % 10000)) {
            feed_wdt();
        }
        spin(1);
    }
}
```

### Print register contents

It's often useful to dump the contents of a register to see what it contained at a particular point in time. A helper function to print individual bits of any 32bit register has been added:

```C
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
```

## Building and flashing

Commands to build, flash and monitor the project are as follows:

```bash
# Set variables for your locations, espidf to PATH
export SDK=~/your_path_to/garage_monitor/esp_bare_c/sdk
export PORT=/dev/ttyACM0
. $HOME/esp/esp-idf/export.sh

# clean and build
make clean build

# flash
make flash

# monitor
make monitor

# erase flash
make erase-flash
```

## Observations and Next Steps

