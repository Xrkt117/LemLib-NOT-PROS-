# Official VEX V5 C++ build, compatible with the VEX Robotics VS Code extension.

PROJECT ?= LemLib-NOT-PROS
ifneq ($(P),)
PROJECT := $(P)
endif
PLATFORM := vexv5
BUILD := build
VEX_SDK_PATH := $(T)

CC := clang
CXX := clang
OBJCOPY := arm-none-eabi-objcopy
SIZE := arm-none-eabi-size
LINK := arm-none-eabi-ld

SOURCES := $(shell find src -type f \( -name '*.cpp' -o -name '*.c' \))
HEADERS := $(shell find include -type f \( -name '*.hpp' -o -name '*.h' \))
OBJECTS := $(patsubst %.cpp,$(BUILD)/%.o,$(filter %.cpp,$(SOURCES)))
OBJECTS += $(patsubst %.c,$(BUILD)/%.o,$(filter %.c,$(SOURCES)))

SDK := $(VEX_SDK_PATH)/$(PLATFORM)
TOOL_INC := -I"$(SDK)/clang/8.0.0/include" -I"$(SDK)/gcc/include/c++/4.9.3" \
            -I"$(SDK)/gcc/include/c++/4.9.3/arm-none-eabi/armv7-ar/thumb" -I"$(SDK)/gcc/include"
COMMON := -target thumbv7-none-eabi -fshort-enums -Wno-unknown-attributes \
          -U__INT32_TYPE__ -U__UINT32_TYPE__ -D__INT32_TYPE__=long -D__UINT32_TYPE__='unsigned long' \
          -march=armv7-a -mfpu=neon -mfloat-abi=softfp -Os -Wall -DVexV5
CXXFLAGS := $(COMMON) -Werror=return-type -fno-rtti -fno-threadsafe-statics -fno-exceptions \
            -std=gnu++11 -ffunction-sections -fdata-sections -Iinclude -I"$(SDK)/include" $(TOOL_INC)
CFLAGS := $(COMMON) -Werror=return-type -ansi -std=gnu99 -Iinclude -I"$(SDK)/include" $(TOOL_INC)
LDFLAGS := -nostdlib -T "$(SDK)/lscript.ld" -R "$(SDK)/stdlib_0.lib" \
           -Map="$(BUILD)/$(PROJECT).map" --gc-section -L"$(SDK)" -L"$(SDK)/gcc/libs"
LIBS := --start-group -lv5rt -lstdc++ -lc -lm -lgcc --end-group

.PHONY: all clean
all: $(BUILD)/$(PROJECT).bin

$(BUILD)/%.o: %.cpp $(HEADERS) Makefile
	@mkdir -p "$(@D)"
	@echo "CXX $<"
	@$(CXX) $(CXXFLAGS) -c -o "$@" "$<"

$(BUILD)/%.o: %.c $(HEADERS) Makefile
	@mkdir -p "$(@D)"
	@echo "CC  $<"
	@$(CC) $(CFLAGS) -c -o "$@" "$<"

$(BUILD)/$(PROJECT).elf: $(OBJECTS)
	@echo "LINK $@"
	@$(LINK) $(LDFLAGS) -o "$@" $^ $(LIBS)
	@$(SIZE) "$@"

$(BUILD)/$(PROJECT).bin: $(BUILD)/$(PROJECT).elf
	@$(OBJCOPY) -O binary "$<" "$@"

clean:
	@rm -rf "$(BUILD)"
