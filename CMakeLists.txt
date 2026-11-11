cmake_minimum_required(VERSION 3.13)

# Built on its own: pull in the SDK here.  Inside pico-examples the top-level
# CMakeLists.txt has already done this (add_subdirectory(tof4_c)).
if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    include(pico_sdk_import.cmake)
    project(tof4_c C CXX ASM)
    set(CMAKE_C_STANDARD 11)
    pico_sdk_init()
endif()

set(TOF4_DRIVERS
    src/machine.c
    src/pyexc.c
    src/st7735.c
    src/sysfont.c
    src/vl53l0x.c
    src/vl53l1x.c
    src/depth_color.c
    src/lidar_map.c
    src/lidar_view.c
)

# tof4_c: the scanner.  tof4_c_lcdtest (lcd_test.c) shows which step stops the
# LCD; tof4_c_bench (bench.c) prints latency, update rate and noise on serial.
foreach(target IN ITEMS tof4_c tof4_c_lcdtest tof4_c_bench)
    if(target STREQUAL tof4_c)
        add_executable(${target} src/main.c ${TOF4_DRIVERS})
    elseif(target STREQUAL tof4_c_lcdtest)
        add_executable(${target} src/lcd_test.c ${TOF4_DRIVERS})
    else()
        add_executable(${target} src/bench.c ${TOF4_DRIVERS})
    endif()

    target_link_libraries(${target} pico_stdlib hardware_gpio)
    target_include_directories(${target} PRIVATE src)
    target_compile_options(${target} PRIVATE -Wall -Wextra)

    # print() goes to the USB serial port, like the Thonny shell
    pico_enable_stdio_usb(${target} 1)
    pico_enable_stdio_uart(${target} 0)

    pico_add_extra_outputs(${target})
endforeach()
