#include "sdk.h"

extern int main(void);

void __attribute__((noreturn)) call_start_cpu0(void) {
    disable_wdt();
    interrupt_init();
    main();
    for(;;) {}
}
