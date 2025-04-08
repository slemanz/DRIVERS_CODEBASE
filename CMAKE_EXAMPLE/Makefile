.PHONY: all build cmake clean

BUILD_DIR := Build
TARGET = flash

all: build

${BUILD_DIR}/Makefile:
	cmake \
		-B${BUILD_DIR} \
		-DCMAKE_TOOLCHAIN_FILE=gcc-arm-none-eabi.cmake

cmake: ${BUILD_DIR}/Makefile

build: cmake
	$(MAKE) -C ${BUILD_DIR} --no-print-directory

load:
	openocd -f interface/jlink.cfg -c "transport select swd" -f target/stm32f4x.cfg -c init -c "reset init" \
	-c "flash write_image erase Build/$(TARGET).elf" -c "reset run" -c shutdown

clean:
	rm -rf $(BUILD_DIR)