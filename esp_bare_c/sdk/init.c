#include "sdk.h"
#include <stdint.h>

extern int main(void);
extern uint32_t _sbss, _ebss;

void __attribute__((noreturn)) call_start_cpu0(void) {
    // Check CPU health by reading hartid. Value should be 0
    uint32_t hartid = CSR_READ(mhartid);
    while (hartid);

    // Clear .bss
    volatile uint32_t *this_word = &_sbss;
    while (this_word < &_ebss) {
        *this_word = 0;
        this_word++;
    }

    //disable_wdt();
    init_wdt(1000);
    interrupt_init();
    main();
    for(;;) {}
}
