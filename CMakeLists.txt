cmake_minimum_required(VERSION 3.20.0)

# Build the PB1 reset task into MCUboot only (not the app)
set(mcuboot_EXTRA_ZEPHYR_MODULES "${CMAKE_CURRENT_LIST_DIR}/mcuboot_pb1_reset")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(untitled)

target_sources(app PRIVATE src/main.c src/co5300.c)