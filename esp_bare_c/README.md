# C Bare Metal Implementation

- [Introduction](#introduction)
- [Quickstart](#quickstart)
- [The Strategy](#the-strategy)
- [Getting Started](#getting-started)
- [Blinky base example](#blinky-base-example)
- [Second Stage Bootloader](#second-stage-bootloader)
- [Build System](#build-system)
- [Introducing interrupts](#introducting-interrupts)
- [Extending the bootloader](#extending-the-bootloader)
- [Building and Flashing](#building-and-flashing)
- [Observations and Next Steps](#observations-and-next-steps)

## Introduction

This is the final version of the garage monitor application. To date, the application has been implemented to a minimally functional state in a number of ways, demonstrating the relative strengths, weaknesses and level of maturity in those development pathways. This final implementation is an exception, in that it will NOT reach the point of feature parity with the other options. This is simply an experiment in truly starting from scratch.

This version of the project has some philosophical similarity with the `no-std` Rust version of the project, in that our user code starts as early in the ESP32 boot process as possible, and does not rely on an operating system to orchestrate tasks. As a result, the project will be entirely impractical, and will serve no purpose aside from the satisfaction of curiosity and a bit of education.

The topics that will be covered will be:
- The ESP32 boot process
- Building and linking from scratch
- Controlling the ESP32C3 and its peripherals by reading and writing from registers
- Hardware interrupts from scratch

## Quickstart

This series of projects is presented as an demonstration and discussion piece, however if you simply want to take some or all of the repository and use it for your own projects without suffering through the narrative you can:

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

As a path of least resistance, this project will use ESP-IDF tools to compile and flash. So consider the prerequisites of this project as having the [ESP toolchain](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html) installed for the ESP32C3. This subproject is based on a linux environment, and doesn't explicitly identify any other system dependencies. At the time of writing, v5.3.1 is the stable release.

The items that will be borrowed from the ESP toolchain will be:
- The `riscv-esp-elf-gcc` cross compiler.
- The `esptool.py` tool for flashing and monitoring

To follow along, it will also be handy to have a copy of the [ESP32-C3 Technical Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32-c3_technical_reference_manual_en.pdf) downloaded.

## Blinky base example

Our minimalistic example will be Blinky - flashing an LED on and off by using the GPIO driver. We'll consider this as the path of least resistance to prove that our toolchain works and we have bare metal control of the processor and peripherals. Since this is an embedded project, we don't have `stdio.h` functions like `printf`, so using GPIOs is more straightforward than printing a message to the host. Note that the ESP32 family _does_ actually have `printf` (and other stdio and stdlib functions) burned in to the chip ROM from the factory, however using those functions is not entirely consistent with the bare metal philosophy so we will ignore them.

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
- Create a second stage bootloader
- Write the gpio and delay functions
- Create scripts for our build system

## Second stage bootloader

The first hurdle to getting our blinky code on to the chip and running will be to get the CPU to acknowledge its existence and load the first instructions. We can familiarise ourselves, at a high level with the ESP32C3 boot process by checking the [documentation](https://docs.espressif.com/projects/esp-idf/en/v5.3.1/esp32c3/api-guides/startup.html). Here we see that a first stage bootloader is located on the ROM of the chip and cannot be modified. `The first stage bootloader loads the second-stage bootloader image to RAM (IRAM & DRAM) from flash offset 0x0.`

This means that for our example, there's very little that needs to be done to facilitate our own `main.c` being loaded on to the chip. The bare minimum that needs to be done is to point the second stage bootloader to our main function. We *should* do a bunch of other checks and tasks before loading our user code, but let's just forget about most of that for now because it won't stop our project from working. The only other thing we will tackle intially will be to disable the watchdog timers (TRM 12) to prevent our chip from getting stuck in a reset loop. We would ideally be implementing code to feed the watchdog timers but that's not part of our objective just yet. Our first pass at a second stage bootloader will look like this, borrowing the function name from ESP-IDF:

#### **`bootloader.c`**
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
static inline void wdt_disable(void) {
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

That should be enough for our first attempt at a second stage bootloader to load our code and start executing. The other functions for our blinky example follow the same pattern - read the manual, define the registers, write a function to manipulate the registers. The only thing worth mentioning is that our blocking delay reads `systicks` from a register, and calls the assembly `nop` pseudo-instruction to spin the CPU until it's time to do work again.

## Build system

Our application code is done. In theory, this should work if assembled to riscv instructions and linked in a way that allows the esp32c3 to find the starting point of the application. So to finish off the prototype, let's setup a Makefile and linker script to facilitate that happening.

For these, again we can draw on the reference material of the Technical Reference Manual, and the ESP-IDF Programming Guide to understand how these need to be laid out. Let's start with the linker script. For the proof of concept, the same general layout as the ESP-IDF linker can be used, however we can simplify a little and ignore the iram_loader_seg for now.

```Linker Script
/* ----snip---- */

MEMORY
{
  iram_seg (RWX) :                  org = bootloader_iram_seg_start, len = bootloader_iram_seg_len
  iram_loader_seg (RWX) :           org = bootloader_iram_loader_seg_start, len = bootloader_iram_loader_seg_len
  dram_seg (RW) :                   org = bootloader_dram_seg_start, len = bootloader_dram_seg_len
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

So as horrible as this all looks, all it is really doing is:
- Defining the boundaries of some regions of memory with the `MEMORY` flag (iram for instructions, dram for variables and constants)
- Telling the linker that the `call_start_cpu0` function needs to be the first code that is executed with the `ENTRY` flag
- Ensuring the linker places instructions with certain labels in the appropriate part of the memory (eg all text sections are placed in iram_seg)

Finally, we can put together a Makefile to automate the build process. This allows some rather verbose command line instructions to be called with the make commands:

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

So... The moment of truth... With a LED and 220R resistor wired to GPIO2, we can do the following:

```bash
export SDK=~/your_path_to/garage_monitor/esp_bare_c/sdk
export PORT=/dev/ttyACM0
. $HOME/esp/esp-idf/export.sh

make erase-flash clean build flash
```

...and there's a LED flashing on/off over a 1 second cycle. A cool proof of concept, but just a starting point. Let's push through and explore how to read from a GPIO and work with tasks and interrupts.

## GPIO read and logging

If we want to poll a GPIO to read the state of a reed switch, the process is very similar to the previously applied techniques of configuring the GPIO driver to treat that pin as an input by setting register values, and read the state of the pin from another register.

#### **`sdk.h`**
```C
// Set gpio pin as input with internal pulldown resistor
static inline void gpio_set_input_pulldown(int pin) {
    // Clear the output configuration bit
    REG_RW(GPIO, 0x20) &= ~BIT(pin);
    // Configure pin as input (bit 9) and enable pulldown (bit 10)
    REG_RW(IO_MUX, (0x0004 + (4 * pin))) = BIT(9) | BIT(10);
}

// Return the current digital state of a gpio input
static inline bool gpio_read(int pin) {
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

This all seems pretty straightforward - each of those operations can be performed with literally one line of code. However it would be tedious for us to write to this peripheral one byte at a time. Let's also implement our own version of `puts`, so we can write a whole null terminated string to the buffer. Our higher level function could look like this:

#### **`sdk.h`**
```C
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
```

Note that there is a `usb_wait_for_flush()` that is simply a timeout to prevent this functionality from blocking forever in the event that there is no host connected. This function will return the number of bytes successfully written to the buffer (or -1 if no bytes were written), and this results in some useful feedback to the caller of the function. However, it is worth pausing to consider some of the performance implications of this higher level function. Thinking about how this will be represented in assembly, there are a lot of branches here, repeated calls to the strlen function, a loop that counts up, and comparison between signed integers. These choices all introduce multiple extra instructions. Our processor is plenty fast enough to deal with this code without any issues, but it's also helpful to be aware of how our design choices influence the amount of memory and instructions that will be required.

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

Again, we are reminded that stdio is not present in the embedded environment when we format our message, but we can test this and see two things:

- We have some logging capability
- This synchronous code makes it possible that events will be missed

This leads us back to the same solution we have previously considered to ensure we capture transient events on our gpio input - interrupts. Only this time, we will need to implement them at a low level.

## Introducing interrupts

Our use case is to:
- Read from a GPIO
- Output to a different GPIO, depending on the state of the read

This fundamentally simple use case can be addressed with blocking code by simply referring to Section 5.4 of the TRM and adding a gpio_get_level function to our SDK. But it seems a more educational process to do things the hard way and in the process, learn a bit more about the inner workings of the SOC.

## Extending the bootloader

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

