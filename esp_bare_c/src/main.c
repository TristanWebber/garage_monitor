#include "sdk.h"

#define LED_PIN GPIO_NUM_2
#define SW_PIN  GPIO_NUM_3

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
