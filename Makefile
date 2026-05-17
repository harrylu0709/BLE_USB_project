CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
MACH=cortex-m4
CFLAGS= -c -mcpu=$(MACH) -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -std=gnu11 -Wall -O0 -I./drivers/Inc -I./include -I./USB_bare/Class/CDC/Inc -I./USB_bare/Core/Inc -I./USB_bare/Core/Inc/CMSIS/Include -I./USB_bare/Core/Inc/CMSIS/Device/ST/STM32F4xx/Include -I./ThirdParty/FreeRTOS/include -I./ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F -I./ThirdParty/FreeRTOS -I./ThirdParty/SEGGER/Config -I./ThirdParty/SEGGER/OS -I./ThirdParty/SEGGER/SEGGER
#CFLAGS= -c -mcpu=$(MACH) -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -std=gnu11 -Wall -O0 -I./drivers/Inc -I./include -I./USB_bare/Class/CDC/Inc -I./USB_bare/Core/Inc -I./USB_bare/Core/Inc/CMSIS/Include -I./USB_bare/Core/Inc/CMSIS/Device/ST/STM32F4xx/Include -I./ThirdParty/FreeRTOS/include -I./ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F -I./ThirdParty/FreeRTOS
#CFLAGS= -c -mcpu=$(MACH) -mthumb -mfloat-abi=soft -std=gnu11 -Wall -O0 -I./drivers/Inc -I./include -I./USB/Class/CDC/Inc -I./USB/Core/Inc 
LDFLAGS= -mcpu=$(MACH) -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 --specs=nano.specs -T stm32_ls.ld -Wl,-Map=final.map
LDFLAGS_SH= -mcpu=$(MACH) -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 --specs=rdimon.specs -T stm32_ls.ld -Wl,-Map=final_sh.map
# Directories
SRC_DIR = .
DRIVERS_DIR = drivers/Src
Core_DIR = USB_bare/Core/Src
CDC_DIR = USB_bare/Class/CDC/Src
PORT_DIR = ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F
HEAP_DIR = ThirdParty/FreeRTOS/portable/MemMang
RTOS_DIR = ThirdParty/FreeRTOS
SEG_DIR_1 = ThirdParty/SEGGER/Config
SEG_DIR_2 = ThirdParty/SEGGER/OS
SEG_DIR_3 = ThirdParty/SEGGER/SEGGER
SEG_DIR_4 = ThirdParty/SEGGER/SEGGER/Syscalls
SEG_DIR_5 = ThirdParty/SEGGER/Rec
#Core_DIR = USB/Core/Src
#CDC_DIR = USB/Class/CDC/Src

# Find all .c files
SRCS = $(wildcard $(SRC_DIR)/*.c) $(wildcard $(DRIVERS_DIR)/*.c) $(wildcard $(CDC_DIR)/*.c) $(wildcard $(Core_DIR)/*.c) $(wildcard $(PORT_DIR)/*.c) $(wildcard $(HEAP_DIR)/*.c) $(wildcard $(RTOS_DIR)/*.c) $(wildcard $(SEG_DIR_1)/*.c) $(wildcard $(SEG_DIR_2)/*.c) $(wildcard $(SEG_DIR_3)/*.c) $(wildcard $(SEG_DIR_4)/*.c) $(wildcard $(SEG_DIR_5)/*.c) 
#SRCS = $(wildcard $(SRC_DIR)/*.c) $(wildcard $(DRIVERS_DIR)/*.c) $(wildcard $(CDC_DIR)/*.c) $(wildcard $(Core_DIR)/*.c) $(wildcard $(PORT_DIR)/*.c) $(wildcard $(HEAP_DIR_DIR)/*.c) $(wildcard $(RTOS_DIR)/*.c)

ASRCS = $(wildcard $(SEG_DIR_3)/*.S)

# Convert .c to .o
OBJS = $(SRCS:.c=.o) $(ASRCS:.S=.o)
# Output binary
TARGET_NAME = final
ELF = $(TARGET_NAME).elf
ELF_SH = final_sh.elf
BIN = $(TARGET_NAME).bin

all: $(BIN)
semi: $(ELF_SH)
# Compile .c to .o
%.o: %.c
	$(CC) $(CFLAGS) -o $@ $^

%.o: %.S
	$(CC) $(CFLAGS) -o $@ $^

# Build the target
$(ELF):$(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^

# Build the target
$(ELF_SH):$(filter-out ./syscalls.o, $(OBJS))
	$(CC) $(LDFLAGS_SH) -o $@ $^

# Convert elf to bin
$(BIN): $(ELF)
	$(OBJCOPY) -O binary $^ $@

clean:
	rm -rf $(DRIVERS_DIR)/*.o
	rm -rf $(Core_DIR)/*.o
	rm -rf $(CDC_DIR)/*.o
	rm -rf $(PORT_DIR)/*.o
	rm -rf $(RTOS_DIR)/*.o
	rm -rf $(HEAP_DIR)/*.o
	rm -rf $(SEG_DIR_1)/*.o
	rm -rf $(SEG_DIR_2)/*.o
	rm -rf $(SEG_DIR_3)/*.o
	rm -rf $(SEG_DIR_4)/*.o
	rm -rf $(SEG_DIR_5)/*.o
	rm -rf *.elf
	rm -rf *.o
	rm -rf *.map
	rm -rf *.bin
load:
	openocd -f board/stm32f4discovery.cfg

elf:
	openocd -f board/stm32f4discovery.cfg -c init -c halt -c "flash write_image erase $(ELF)" -c reset -c shutdown

elf_sh:
	openocd -f board/stm32f4discovery.cfg -c init -c halt -c "flash write_image erase $(ELF_SH)" -c init -c "arm semihosting enable" -c reset

bin:
	openocd -f board/stm32f4discovery.cfg -c "program $(BIN) verify reset exit 0x08000000"
#openocd -f board/stm32f4discovery.cfg -c "program final.elf verify reset exit"
#openocd -f board/stm32f4discovery.cfg -c "program final.bin exit 0x08000000"
#https://openocd.org/doc/html/Flash-Programming.html