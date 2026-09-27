# PPStoAVS 固件（CH32M030K9U7）
# 工具链：MounRiver Studio 2 自带 RISC-V GCC12（riscv-wch-elf-），可用 TOOLCHAIN= 覆盖
#   make            编译
#   make clean      清理

MRS_ROOT  ?= C:/MounRiver/MounRiver_Studio2/resources/app/resources/win32/components/WCH
TOOLCHAIN ?= $(MRS_ROOT)/Toolchain/RISC-V Embedded GCC12/bin
PREFIX    := riscv-wch-elf-

CC      := "$(TOOLCHAIN)/$(PREFIX)gcc"
OBJCOPY := "$(TOOLCHAIN)/$(PREFIX)objcopy"
OBJDUMP := "$(TOOLCHAIN)/$(PREFIX)objdump"
SIZE    := "$(TOOLCHAIN)/$(PREFIX)size"

TARGET := PPStoAVS
BUILD  := build

SRC_C := $(wildcard src/*.c) \
         $(wildcard sdk/Core/*.c) \
         $(wildcard sdk/Peripheral/src/*.c) \
         sdk/User/system_ch32m030.c \
         sdk/User/ch32m030_it.c
SRC_S := sdk/Startup/startup_ch32m030.S

INC := -Isrc -Isdk/Core -Isdk/Peripheral/inc -Isdk/User

ARCH := -march=rv32imc_zba_zbb_zbc_zbs_xw -mabi=ilp32 -msmall-data-limit=8 -mno-save-restore

CFLAGS := $(EXTRA_DEFS) $(ARCH) -Os -std=gnu99 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections \
          -fno-common -Wall -Wextra -Wno-unused-parameter -g $(INC)
ASFLAGS := $(ARCH) -x assembler-with-cpp $(INC)
LDFLAGS := $(ARCH) -T $(BUILD)/link.ld -nostartfiles -Xlinker --gc-sections -Wl,-Map,$(BUILD)/$(TARGET).map \
           --specs=nano.specs --specs=nosys.specs -Wl,--print-memory-usage

OBJS := $(addprefix $(BUILD)/,$(SRC_C:.c=.o)) $(addprefix $(BUILD)/,$(SRC_S:.S=.o))

all: $(BUILD)/$(TARGET).hex $(BUILD)/$(TARGET).bin
	@$(SIZE) $(BUILD)/$(TARGET).elf

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo CC $<
	@$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

$(BUILD)/%.o: %.S
	@mkdir -p $(dir $@)
	@echo AS $<
	@$(CC) $(ASFLAGS) -c $< -o $@

# 程序区 0xBF00，其上为设置（2 页）与记录（128 页），见 src/flash_io.h
$(BUILD)/link.ld: sdk/Ld/Link.ld
	@mkdir -p $(dir $@)
	@sed 's/LENGTH = 64K/LENGTH = 0xBF00/' $< > $@
	@grep -q "LENGTH = 0xBF00" $@

$(BUILD)/$(TARGET).elf: $(OBJS) $(BUILD)/link.ld
	@echo LD $@
	@$(CC) $(LDFLAGS) $(OBJS) -o $@

$(BUILD)/$(TARGET).hex: $(BUILD)/$(TARGET).elf
	@$(OBJCOPY) -O ihex $< $@

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	@$(OBJCOPY) -O binary $< $@

clean:
	rm -rf $(BUILD)

-include $(OBJS:.o=.d)

.PHONY: all clean
