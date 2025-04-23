# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
# CMakeLists.txt
# ——————————————————————————————————————————————————————————————————————————————
cmake_minimum_required(VERSION 3.13)

# —————— Project-wide settings ——————
set(CMAKE_C_STANDARD            11)
set(CMAKE_CXX_STANDARD          17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Add optimization / fixed‑point flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Ofast -ffixed-point")

# —————— VS Code Pico Extension support ——————
if (WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()

set(sdkVersion       2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion  2.1.0)

set(picoVscode "${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake")
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()

# —————— Pico SDK ——————
set(PICO_BOARD pico CACHE STRING "Board type")
include(pico_sdk_import.cmake)
project(audio_triangulation C CXX ASM)
pico_sdk_init()

add_executable(${PROJECT_NAME})

pico_generate_pio_header(audio_triangulation
    ${CMAKE_CURRENT_LIST_DIR}/src/lib/pio/hsync.pio
)
pico_generate_pio_header(audio_triangulation
    ${CMAKE_CURRENT_LIST_DIR}/src/lib/pio/vsync.pio
)
pico_generate_pio_header(audio_triangulation
    ${CMAKE_CURRENT_LIST_DIR}/src/lib/pio/rgb.pio
)

# —————— Source discovery ——————
# Recursively grab all .c/.cpp/.h under src/
file(GLOB_RECURSE PROJECT_PINOUTS
    "${CMAKE_CURRENT_LIST_DIR}/src/*.pio"
)
file(GLOB_RECURSE PROJECT_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/src/*.c"
)
file(GLOB_RECURSE PROJECT_HEADERS
    "${CMAKE_CURRENT_LIST_DIR}/src/*.h"
)

# —————— Executable ——————
target_sources(${PROJECT_NAME}
    PRIVATE
        ${PROJECT_SOURCES}
)

# Make “#include <foo.h>” pick up anything in src/ or src/
target_include_directories(${PROJECT_NAME}
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/src"
)

# —————— stdio setup ——————
pico_enable_stdio_usb (${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

# —————— Link libraries ——————
target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    pico_bootsel_via_double_reset
    pico_multicore

    hardware_pwm
    hardware_dma
    hardware_irq
    hardware_adc
    hardware_pio
    hardware_i2c
    hardware_spi
    hardware_sync
    hardware_uart
    hardware_timer
)

if (PICO_CYW43_SUPPORTED)
    target_link_libraries(${PROJECT_NAME}
        pico_cyw43_arch_none
    )
endif()

# —————— Outputs ——————
pico_add_extra_outputs(${PROJECT_NAME})
# pico_set_program_url(${PROJECT_NAME} YOUR_PROJECT_URL_HERE)
