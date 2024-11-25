# C Bare Metal Implementation

- [Introduction](#introduction)
- [Quickstart](#quickstart)
- [The Strategy](#the-strategy)
- [Getting Started](#getting-started)
- [Blinky base example](#blinky-base-example)
- [Second Stage Bootloader](#second-stage-bootloader)
- [Build System](#build-system)
- [Building and Flashing](#building-and-flashing)
- [Observations and Next Steps](#observations-and-next-steps)

## Introduction

This is the final version of the garage monitor application. To date, the application has been implemented to a minimally functional state in a number of ways, demonstrating the relative strengths, weaknesses and level of maturity in those development pathways. This final implementation is an exception, in that it will NOT reach the point of feature parity with the other options. This is simply an experiment in truly starting from scratch.

This version of the project has some philosophical similarity with the `no-std` Rust version of the project, in that our user code starts as early in the ESP32 boot process as possible, and does not rely on an operating system to orchestrate tasks. As a result, the project will be entirely impractical, and will serve no purpose aside from the satisfaction of curiosity and a bit of education.

The topics that will be covered will be:
- The ESP32 boot process
- Building and linking from scratch
- Controlling the ESP32C3 by reading and writing from registers

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

This project will start off by reviewing the documentation for the ESP32C3, and the documentation and code in the ESP-IDF. The first task will be understanding the path of least resistance to getting our own code running on the chip. This will serve as the initial proof of concept for a from-scratch application.

Once our minimal example is ready, the build tools will be set up. To simplify the process, this will use some of the ESP-IDF tools, so whilst the application is independent of the ESP-IDF, the build tools will not be.

Finally, the basic example will be built upon to achieve the gpio features similar to the previous versions of the project.

## Getting started

As a path of least resistance, this project will use ESP-IDF tools. So consider the prerequisites of this project as having the [ESP toolchain](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html) installed for the ESP32C3. This subproject is based on a linux environment, and doesn't explicitly identify any other system dependencies. At the time of writing, v5.3.1 is the stable release.

The items that will be borrowed from the ESP toolchain will be:
- The `riscv-esp-elf-gcc` cross compiler.
- The `esptool.py` tool for flashing and monitoring

To follow along, it will also be handy to have a copy of the [ESP32-C3 Technical Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32-c3_technical_reference_manual_en.pdf) downloaded.

## Blinky base example

Our minimalistic example will be Blinky. This allows the proof of concept to ignore logging to UART (which would be required for hello world). We will make a LED blink and use that as proof that our code has been flashed on to the device, has been loaded by the first stage bootloader, and operates correctly. We will work towards making the following code work:


#### **`main.c`**
```C
#include "sdk.h"

#define LED_PIN GPIO_NUM_3
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

The first hurdle to getting our blinky on to the chip and running will be to get the CPU to acknowledge its existence and load the first instructions. We can familiarise ourselves, at a high level with the ESP32C3 boot process by checking the [documentation](https://docs.espressif.com/projects/esp-idf/en/v5.3.1/esp32c3/api-guides/startup.html). Here we see that a first stage bootloader is located on the ROM of the chip and cannot be modified. The first stage bootloader loads the second-stage bootloader image to RAM (IRAM & DRAM) from flash offset 0x0.

This means that for our example, there's very little that needs to be done to facilitate our own main.c being loaded on to the chip. The bare minimum that needs to be done is to point the bootloader to our main function. We *should* do a bunch of other checks and tasks before loading our user code, but let's just forget about most of that for now because it won't stop our project from working. The only other thing we will tackle intially will be to disable the watchdog timers (TRM 12) to prevent our chip from getting stuck in a reset loop. We would ideally be implementing code to feed the watchdog timers but that's not part of our objective just yet. Our first pass at a second stage bootloader will look like this, borrowing the function name from ESP-IDF:

#### **`bootloader.c`**
```C
#include "sdk.h"

extern int main(void);

void __attribute__((noreturn)) call_start_cpu0(void) {
    disable_wdt();
    main();
}
```

And now we get in to our first low level operations to disable the watchdog timers! Looking at Section 12 of the Technical Reference Manual, there's all the detail we need in order to prevent the watchdog timers from intervening in our application:
- There are three digital and one analog watchdogs for us to disable (TRM Figure 12-1). To be safe, let's disable all of them.
- Settings can be modified by writing to bits in the Timer Group and RTC registers (TRM 12.5)
- Write protection is implemented to prevent inadvertent modification of WDT configurations (TRM 12.2.2.3)

This means that we will be interacting with registers - as low as we can go. First, let's assume we'll be doing this a lot, and set some macros to make life easier. From the docs, we know the registers are 32 bit wide, and that we will need to read and write individual bits so we define a BIT macro and a register read/write macro. Also from the documentation, we know that registers for certain peripherals are accessed via base addresses, and defined in the technical reference manual by their offsets, so let's name those base addresses while we're there.

#### **`sdk.c`**
```C
#define BIT(x) ((uint32_t) 1U << (x))
#define REG_RW(base, offset) (*(volatile uint32_t *) ((base) + (offset)))

#define LOW_POWER_MGT 0x60008000
#define TIMER_GROUP_0 0x6001F000
#define TIMER_GROUP_1 0x60020000
```

Following the approach described in the docs, we'll disable write protection, set the bits we need to get our desired behaviour, and re-enable write protection:

#### **`sdk.c`**
```C
static inline void wdt_disable(void) {
    // Disable RTC WDT (TRM 12.2.2.3)
    REG_RW(LOW_POWER_MGT, 0xA8) = 0x50D83AA1;    // Disable write protection
    REG_RW(LOW_POWER_MGT, 0x90) &= ~BIT(31);     // Clear BIT31 to disable RTC WDT
    REG_RW_RW(LOW_POWER_MGT, 0xA8) = 0x0;        // Re-enable write protection

    // Configure super WDT auto feed (TRM 12.3.2.2)
    REG_RW(LOW_POWER_MGT, 0xB0) = 0x8F1D312A;    // Disable write protection
    REG_RW(LOW_POWER_MGT, 0xAC) |= BIT(31);      // Set BIT31 to enable auto feed
    REG_RW(LOW_POWER_MGT, 0xB0) = 0x0;           // Re-enable write protection

    // Disable Timer Group 0 WDT (TRM 12.2.2.3)
    // TG0 and TG1 are write enabled by default
    REG_RW(TIMER_GROUP_0, 0xFC) &= ~BIT(29);     // Clear BIT29 to disable WDT clock
    REG_RW(TIMER_GROUP_0, 0x48) &= ~BIT(31);     // Clear BIT31 to disable TG0 WDT

    // Disable Timer Group 1 WDT (TRM 12.2.2.3)
    REG_RW(TIMER_GROUP_0, 0xFC) &= ~BIT(29);     // Clear BIT29 to disable WDT's clock
    REG_RW(TIMER_GROUP_1, 0x48) &= ~BIT(31);     // Clear BIT 31 to disable TG1 WDT
}
```

That should be enough for our first attempt at a second stage bootloader to load our code and start executing. The other functions for our blinky example follow the same pattern - read the manual, define the registers, write a function to manipulate the registers.

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

Finally, we can put together a Makefile to automate the build process:

```make

```

## Building and flashing

Commands to build, flash and monitor the project are as follows:

```bash
# build
make build

# flash
make flash

# monitor
make monitor

# clean, build, flash and monitor
make all
```

## Observations and Next Steps

