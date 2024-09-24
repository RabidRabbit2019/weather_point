.PHONY: clean

release:
	arm-none-eabi-g++ -c -Wall -Wextra -Os -flto -s \
	  main.cpp \
	  crc16.cpp \
	  bme280.cpp \
	  radiomodem.cpp \
	  stdio.c \
	  string.c \
	  trace.c \
	  -ffunction-sections -fdata-sections -mcpu=cortex-m3 -mthumb -lnosys \
	  -nostartfiles -ffreestanding -fno-exceptions -fno-rtti -specs=nosys.specs -pipe
	arm-none-eabi-g++ -c -Wall -Wextra -Os -s \
	  vectors.c \
	  -ffunction-sections -fdata-sections -mcpu=cortex-m3 -mthumb -lnosys \
	  -nostartfiles -ffreestanding -fno-exceptions -fno-rtti -specs=nosys.specs -pipe
	arm-none-eabi-g++ \
	  vectors.o \
	  trace.o \
	  stdio.o \
	  string.o \
	  radiomodem.o \
	  bme280.o \
	  crc16.o \
	  main.o \
	  -Wl,-gc-sections -mcpu=cortex-m3 -mthumb -lnosys -nostartfiles -ffreestanding \
	  -specs=nosys.specs -fno-exceptions -fno-rtti -TSTM32F103C8Tx_FLASH.ld -flto -pipe -s -o a.elf

debug:
	arm-none-eabi-g++ -c -Wall -Wextra -O0 -g \
	  vectors.c \
	  main.cpp \
	  crc16.cpp \
	  bme280.cpp \
	  radiomodem.cpp \
	  stdio.c \
	  string.c \
	  trace.c \
	  -ffunction-sections -fdata-sections -mcpu=cortex-m3 -mthumb -lnosys -nostartfiles \
	  -ffreestanding -specs=nosys.specs -fno-exceptions -fno-rtti -pipe
	arm-none-eabi-g++ \
	  vectors.o \
	  trace.o \
	  stdio.o \
	  string.o \
	  radiomodem.o \
	  bme280.o \
	  crc16.o \
	  main.o \
	  -Wl,-gc-sections -mcpu=cortex-m3 -mthumb -lnosys -nostartfiles -ffreestanding \
	  -specs=nosys.specs -TSTM32F103C8Tx_FLASH.ld -fno-exceptions -fno-rtti -o a.elf

clean:
	rm *.elf
	rm *.o
