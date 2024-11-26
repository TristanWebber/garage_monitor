#ifndef SDK_H
#define SDK_H

#include <stdbool.h>
#include <stdint.h>

#define BIT(x) ((uint32_t) 1U << (x))
#define REG_RW(base, offset) (*(volatile uint32_t *) ((base) + (offset)))

#define GPIO          0x60004000
#define LOW_POWER_MGT 0x60008000
#define TIMER_GROUP_0 0x6001F000
#define TIMER_GROUP_1 0x60020000
#define SYSTEM_TIMER  0x60023000

#define GPIO_NUM_2 2

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

// Perform `count` "NOP" operations
static inline void spin(volatile uint64_t count) {
    while (count--) asm volatile("nop");
}

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

// 16MHz to 1MHz
static inline uint64_t uptime_us(void) {
    return systick() >> 4;
}

static inline void delay_us(uint64_t us) {
    uint64_t until = uptime_us() + us;
    while (uptime_us() < until) spin(1);
}

static inline void delay_ms(uint64_t ms) {
    delay_us(ms * 1000);
}

#endif /* SDK_H */
