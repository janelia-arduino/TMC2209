cmake_minimum_required(VERSION 3.13)

# When used as an ESP-IDF component this file only registers the sources.
if(ESP_PLATFORM)
  idf_component_register(
    SRCS "src/tmc2209.c"
    INCLUDE_DIRS "src"
  )
  return()
endif()

project(tmc2209 LANGUAGES C)

add_library(tmc2209 STATIC src/tmc2209.c)
target_include_directories(tmc2209 PUBLIC src)
target_compile_features(tmc2209 PUBLIC c_std_17)

# The STM32 CMSIS port needs the target's CMSIS device header. Enable it
# with -DTMC2209_BUILD_STM32_PORT=ON when cross compiling and provide
# TMC2209_STM32_DEVICE_HEADER plus the CMSIS include directories yourself,
# for example:
#   target_compile_definitions(tmc2209_stm32 PUBLIC
#     TMC2209_STM32_DEVICE_HEADER="stm32f4xx.h" STM32F401xC)
option(TMC2209_BUILD_STM32_PORT "Build the STM32 CMSIS port" OFF)
if(TMC2209_BUILD_STM32_PORT)
  add_library(tmc2209_stm32 STATIC ports/stm32/tmc2209_stm32.c)
  target_include_directories(tmc2209_stm32 PUBLIC ports/stm32)
  target_link_libraries(tmc2209_stm32 PUBLIC tmc2209)
endif()

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  enable_testing()
  add_executable(test_tmc2209 tests/test_tmc2209.c)
  target_link_libraries(test_tmc2209 PRIVATE tmc2209)
  add_test(NAME tmc2209 COMMAND test_tmc2209)
endif()
