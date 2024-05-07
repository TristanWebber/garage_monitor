#ifndef DEBUG_PRINT_H
#define DEBUG_PRINT_H

// Macros for debug functionality. Only uses serial output if DEBUG_SERIAL is defined.
#if defined(DEBUG_SERIAL)
    #define DBG_PRINTTIME()       \
        {                           \
            Serial.print(millis()); \
        }
    #define DBG_PRINT(x)          \
        {                           \
            Serial.print(x);        \
        }
    #define DBG_PRINTLN(x)        \
        {                           \
            Serial.println(x);      \
        }
#else
    #define DBG_PRINTTIME()
    #define DBG_PRINT(x)
    #define DBG_PRINTLN(x)
#endif /* DEBUG_SERIAL */

#endif /* DEBUG_PRINT_H */
