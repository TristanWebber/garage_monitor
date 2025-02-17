#include "sdk.h"
#include <stdbool.h>
#include <stdint.h>

#define LED_PIN GPIO_NUM_2
#define SW_PIN  GPIO_NUM_3

bool led_state = 0;
char *led_msg = "LED state: 0\r\n";
char *switch_msg = "Switch: 0\r\n";

static void switch_handler(void *param) {
    uint32_t pin = (uint32_t) (uintptr_t) param;
    gpio_clear_interrupt(pin);
    led_state = gpio_read(pin);
    switch_msg[8] = led_state + '0';
    usb_print(switch_msg);
    gpio_set_level(LED_PIN, led_state);
}

int main(void) {
    gpio_set_output(LED_PIN);
    gpio_set_input_pulldown(SW_PIN);
    gpio_set_irq_handler(SW_PIN, switch_handler, (void *) SW_PIN);

    for(;;) {
        led_state = gpio_read(SW_PIN);
        gpio_set_level(LED_PIN, led_state);
        led_msg[11] = led_state + '0';
        usb_print(led_msg);
        delay_ms(500);
    }

    return 0;
}
