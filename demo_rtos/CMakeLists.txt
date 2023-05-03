cmake_minimum_required(VERSION 3.13)

set(PICO_SDK_PATH "../../pico-sdk")

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

# Pull in FreeRTOS
include(FreeRTOS_Kernel_import.cmake)

project(example C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(demo_rtos main.c)

target_include_directories(demo_rtos PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(demo_rtos pico_stdlib FreeRTOS-Kernel FreeRTOS-Kernel-Heap1)
pico_add_extra_outputs(demo_rtos)
