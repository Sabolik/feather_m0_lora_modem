################################################################################
# Re-factored Automatically-generated file. Generated by Atmel Studio 7
################################################################################

COMPILER := arm-none-eabi-
CC := $(COMPILER)gcc
OBJCOPY := $(COMPILER)objcopy

RM := rm -rf

C_SRCS :=  \
Device_Startup/startup_samd21.c \
Device_Startup/system_samd21.c \
modem/buffer.c \
modem/frame.c \
modem/hal/hal.c \
modem/hex.c \
modem/lmic/aes.c \
modem/lmic/lmic.c \
modem/lmic/oslmic.c \
modem/lmic/radio.c \
modem/main.c \
modem/modem.c \
modem/queue.c \
modem/sensor/bme280.c \
modem/sensor/sensor.c \
modem/sensor/weather_click.c \
xprintf/xprintf.c

OBJDIR := build
C_SRCSDIR := .
LIB_MFILEDIR := $(C_SRCSDIR)/driver

OUTPUT_FILE_PATH := $(OBJDIR)/feather_m0_lora_modem.bin

LIBS := -L$(C_SRCSDIR)/Device_Startup \
        -L$(C_SRCSDIR)/driver/build

INC := -I$(C_SRCSDIR)/driver/packs/CMSIS/Core/Include \
       -I$(C_SRCSDIR)/driver/packs/samd21a/include

CFLAGS := -x c -mthumb -D__SAMD21G18A__ -DNDEBUG $(INC) -Os -ffunction-sections -mlong-calls -Wall -mcpu=cortex-m0plus -std=gnu99
LDFLAGS := -mthumb -Wl, -Wl,--start-group -lm -lfeather_m0_lora_driver  -Wl,--end-group $(LIBS) -Wl,--gc-sections -mcpu=cortex-m0plus -Tsamd21g18a_flash.ld

_OBJS := $(patsubst %.c,%.o,$(C_SRCS))
OBJS := $(addprefix $(OBJDIR)/, $(_OBJS))

# All Target
all: build_driver $(OUTPUT_FILE_PATH)

# AVR32/GNU C Compiler
$(OBJS): $(OBJDIR)/%.o: $(C_SRCSDIR)/%.c
	mkdir -p $(@D)
	@echo Building file: $<
	@echo Invoking: ARM/GNU C Compiler : 6.3.1
	$(CC) -c $(CFLAGS) -o "$@" "$<" 
	@echo Finished building: $<

$(OUTPUT_FILE_PATH): $(OBJS)
	@echo Building target: $@
	@echo Invoking: ARM/GNU Linker : 6.3.1
	$(CC) -o$(OUTPUT_FILE_PATH) $(OBJS) $(LDFLAGS)
	@echo Finished building target: $@
	$(OBJCOPY) -O binary $(OUTPUT_FILE_PATH)

build_driver:
	$(MAKE) -C $(LIB_MFILEDIR)

# Other Targets
clean:
	$(RM) $(OBJS)
	$(RM) $(OUTPUT_FILE_PATH)
	$(MAKE) -C $(LIB_MFILEDIR) clean
	