cmake_minimum_required(VERSION 3.12)
project(flash)

# define C language
enable_language(C ASM)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

# set executable, linker and startup code
set(EXECUTABLE ${CMAKE_PROJECT_NAME})
set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/Linkers/STM32F401xC.ld)
set(STARTUP_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/Linkers/startup.c)

# include directories
set(INCLUDE_DIRECTORIES
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/Inc)

# sources files

set(SOURCES_DRIVERS
    Drivers/Src/driver_clock.c
    Drivers/Src/driver_systick.c
    Drivers/Src/driver_interrupt.c
    Drivers/Src/driver_gpio.c
    Drivers/Src/driver_uart.c)

set(SOURCES_APP
    App/Src/main.c
    App/Src/config.c)

set(SOURCES_SYSCALL 
    Linkers/startup.c
    Linkers/syscalls.c)

# set source files and include to build
add_executable(${EXECUTABLE}
               ${SOURCES_DRIVERS}
               ${SOURCES_APP}
               ${SOURCES_SYSCALL})

target_include_directories(${EXECUTABLE} PRIVATE
                           ${INCLUDE_DIRECTORIES})


# set compile and linking options
target_compile_options(${EXECUTABLE} PRIVATE
        -c
        -mcpu=cortex-m4
        -mthumb
        -mfloat-abi=soft
        -std=gnu99
        -Wall
        -O0)


target_link_options(${EXECUTABLE} PRIVATE
    -T ${CMAKE_SOURCE_DIR}/Linkers/STM32F401xC.ld
    -mcpu=cortex-m4
    -mthumb
    -mfloat-abi=soft
    -specs=nano.specs
    -Wl,-Map=${PROJECT_NAME}.map
    -Wl,--print-memory-usage) 

## post build commands (show memory and compile do .bin and .ihex)
add_custom_command(TARGET ${EXECUTABLE}
    POST_BUILD
    COMMAND arm-none-eabi-size ${EXECUTABLE}.elf)

add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${EXECUTABLE}>
    ${EXECUTABLE}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${EXECUTABLE}>
    ${EXECUTABLE}.bin)
