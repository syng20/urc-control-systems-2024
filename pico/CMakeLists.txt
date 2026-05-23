# Copyright 2024 Khalil Estell
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.25)
    set(PICO_CXX_ENABLE_EXCEPTIONS 1)
    # set(PICO_CLIB picolibc)
    set(CMAKE_ASM_FLAGS_INIT "-mcpu=cortex-m33 -mfloat-abi=hard")
    include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(foc-driver LANGUAGES CXX C ASM)
set(CMAKE_EXECUTABLE_SUFFIX .elf)

find_package(libhal-picosdk REQUIRED CONFIG)
find_package(libhal-util REQUIRED CONFIG)

pico_sdk_init()

add_executable(simple src/simple_main.cpp)

target_include_directories(simple PRIVATE include)

target_link_libraries(
    simple
    PRIVATE hardware_adc hardware_dma hardware_pwm hardware_i2c
)

set(TARGET simple)

pico_enable_stdio_usb(${TARGET} 1)
pico_enable_stdio_uart(${TARGET} 0)
pico_add_extra_outputs(${TARGET})
pico_set_binary_type(${TARGET} copy_to_ram)
target_link_libraries(${TARGET} PRIVATE pico_stdlib)
target_compile_options(${TARGET} PRIVATE -g -Wall -Wextra -flto=auto)
target_link_options(${TARGET} PRIVATE -flto=auto)
target_include_directories(${TARGET} PUBLIC include src)
target_link_libraries(
    ${TARGET}
    PRIVATE libhal::$ENV{LIBHAL_PLATFORM_LIBRARY} libhal::util
)