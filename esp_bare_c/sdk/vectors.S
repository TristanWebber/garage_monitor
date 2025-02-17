    .global _interrupt_handler
    .global _panic_handler

    .section .vectors.text
    .global vector_table

    .align 8
vector_table:
    j panic_handler
    .rept 31
        j exception_handler
    .endr
