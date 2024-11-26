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
