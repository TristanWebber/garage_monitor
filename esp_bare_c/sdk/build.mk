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
