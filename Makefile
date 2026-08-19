TARGET := nyush_rm_control_h723
BUILD_DIR := build
PREFIX := arm-none-eabi-
CC := $(PREFIX)gcc
AS := $(PREFIX)gcc -x assembler-with-cpp
CP := $(PREFIX)objcopy
SZ := $(PREFIX)size

ROBOT_TYPE ?= infantry
ifeq ($(strip $(ROBOT_TYPE)),)
override ROBOT_TYPE := infantry
endif

PROJECT_DIRS := Src application modules bsp
PROJECT_C_SOURCES := $(foreach dir,$(PROJECT_DIRS),$(shell find $(dir) -type f -name '*.c'))

HAL_C_SOURCES := $(wildcard Drivers/STM32H7xx_HAL_Driver/Src/*.c)
FREERTOS_C_SOURCES := \
	Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
	Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
	Middlewares/Third_Party/FreeRTOS/Source/list.c \
	Middlewares/Third_Party/FreeRTOS/Source/queue.c \
	Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c \
	Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
	Middlewares/Third_Party/FreeRTOS/Source/timers.c \
	Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c \
	Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c \
	Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c
USB_C_SOURCES := \
	Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
	Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
	Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c \
	Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
RTT_C_SOURCES := \
	Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT.c \
	Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT_printf.c
C_SOURCES := $(PROJECT_C_SOURCES) $(HAL_C_SOURCES) $(FREERTOS_C_SOURCES) $(USB_C_SOURCES) $(RTT_C_SOURCES)

ASM_SOURCES := startup_stm32h723vghx.s Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT_ASM_ARMv7M.s
OBJECTS := $(addprefix $(BUILD_DIR)/,$(C_SOURCES:.c=.o) $(ASM_SOURCES:.s=.o))

CPU := -mcpu=cortex-m7
FPU := -mfpu=fpv5-d16
FLOAT_ABI := -mfloat-abi=hard
MCU := $(CPU) -mthumb $(FPU) $(FLOAT_ABI)

C_DEFS := \
	-DUSE_HAL_DRIVER \
	-DSTM32H723xx \
	-DARM_MATH_CM7 \
	-DROBOT_TYPE_$(ROBOT_TYPE)

INCLUDE_ROOTS := Inc application modules bsp \
	Drivers/STM32H7xx_HAL_Driver/Inc \
	Drivers/CMSIS/Device/ST/STM32H7xx/Include \
	Drivers/CMSIS/Include \
	Middlewares/ST/ARM/DSP/Inc \
	Middlewares/Third_Party/FreeRTOS/Source/include \
	Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS \
	Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \
	Middlewares/ST/STM32_USB_Device_Library/Core/Inc \
	Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc \
	Middlewares/Third_Party/SEGGER/RTT \
	Middlewares/Third_Party/SEGGER/Config
ALL_INCLUDE_DIRS := $(sort $(foreach dir,$(INCLUDE_ROOTS),$(shell find $(dir) -type d)))
C_INCLUDES := $(addprefix -I,$(ALL_INCLUDE_DIRS))

CFLAGS := $(MCU) $(C_DEFS) $(C_INCLUDES) -std=gnu11 -Og -g3 -Wall -Wextra \
	-ffunction-sections -fdata-sections -MMD -MP
ASFLAGS := $(MCU) $(C_DEFS) $(C_INCLUDES) -g3

LDSCRIPT := STM32H723VGHX_FLASH.ld
LIBDIR := -LMiddlewares/ST/ARM/DSP/Lib
LIBS := -Wl,--start-group -lc -lm -lnosys -l:libCMSISDSP.a -Wl,--end-group
LDFLAGS := $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) \
	-Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

.PHONY: all clean flash_dfu flash_pyocd flash_dap flash_stlink flash_jlink
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

$(BUILD_DIR)/%.o: %.c Makefile
	@mkdir -p $(dir $@)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile
	@mkdir -p $(dir $@)
	$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS)
	@mkdir -p $(dir $@)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	$(CP) -O ihex $< $@

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(CP) -O binary -S $< $@

clean:
	rm -rf $(BUILD_DIR)

FLASH_START ?= 0x08000000
PYOCD_TARGET ?= STM32H723VG
PYOCD_FREQUENCY ?=
PYOCD_CONNECT ?= under-reset
DAP_SERIAL ?=
JLINK_DEVICE ?= STM32H723VG
JLINK_SPEED ?= 4000
JLINK_SERIAL ?=

flash_dfu: $(BUILD_DIR)/$(TARGET).bin
	dfu-util -a 0 -s $(FLASH_START):leave -D $<

flash_pyocd: $(BUILD_DIR)/$(TARGET).elf
	pyocd load $< $(if $(DAP_SERIAL),-u $(DAP_SERIAL),) -t $(PYOCD_TARGET) \
		$(if $(PYOCD_FREQUENCY),-f $(PYOCD_FREQUENCY),) -M $(PYOCD_CONNECT)

flash_dap flash_stlink: flash_pyocd

flash_jlink: $(BUILD_DIR)/$(TARGET).elf
	@printf 'r\nloadfile %s\nr\ng\nexit\n' "$<" | \
		JLinkExe -NoGui 1 -AutoConnect 1 -Device $(JLINK_DEVICE) -If SWD -Speed $(JLINK_SPEED) \
		$(if $(JLINK_SERIAL),-SelectEmuBySN $(JLINK_SERIAL),)

-include $(OBJECTS:.o=.d)
