# Set the minimum version of CMake that can be used
cmake_minimum_required(VERSION 3.20)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
set(CMAKE_C_COMPILER_FORCED TRUE)
set(CMAKE_CXX_COMPILER_FORCED TRUE)

# Set target definition variables required by the toolchain file
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/target.cmake)

# Set the toolchain file for cross-compilation
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake)

# Project definition
project(ec)

# Enable CMake support for ASM and C/CPP languages
enable_language(C ASM)

set(PROJECT_NAME ec)

# Disable with: cmake --preset Release -DEC_ENABLE_OLED=OFF
# Existing build caches keep their value; changing the default does not override it.
option(EC_ENABLE_OLED "Enable the SSD1306 OLED display and u8g2" OFF)

# Create an executable object type
add_executable(${PROJECT_NAME})

# Include additional CMake files for flags, components, and files
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/flags.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/components.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/files.cmake)

# Add sources to executable/library
target_sources(${PROJECT_NAME} PRIVATE
  # System initialization
  "${CMAKE_CURRENT_SOURCE_DIR}/Src/system_stm32l0xx.c"
  # BQ27220 fuel gauge driver
  "${CMAKE_CURRENT_SOURCE_DIR}/Src/bq27220.c"
  # EC platform and state machine
  "${CMAKE_CURRENT_SOURCE_DIR}/Src/ec_platform_stm32l051.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/Src/ec_state_machine.c"
  # LL Utils (SysTick delay functions)
  "${CMAKE_CURRENT_SOURCE_DIR}/../STM32CubeL0/Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_ll_utils.c"
)

if(EC_ENABLE_OLED)
target_sources(${PROJECT_NAME} PRIVATE
  # 0.91" OLED 128x32 (SSD1306) driver — shared I2C1 bus
  "${CMAKE_CURRENT_SOURCE_DIR}/Src/oled_ssd1306.c"
  # OLED page controller (dashboard + electrical details)
  "${CMAKE_CURRENT_SOURCE_DIR}/Src/oled_display.c"
  # Minimal u8g2 2.36.19 subset: full-buffer drawing + SSD1306 128x32
  "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/u8g2/csrc/u8g2_bitmap.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/u8g2/csrc/u8g2_box.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/u8g2/csrc/u8g2_buffer.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/u8g2/csrc/u8g2_circle.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/u8g2/csrc/u8g2_font.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/u8g2/csrc/u8g2_font_ec_ui_12.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/u8g2/csrc/u8g2_font_ec_values_9x15.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/u8g2/csrc/u8g2_fonts_selected.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/u8g2/csrc/u8g2_hvline.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/u8g2/csrc/u8g2_intersection.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/u8g2/csrc/u8g2_kerning.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/u8g2/csrc/u8g2_line.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/u8g2/csrc/u8g2_ll_hvline.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/u8g2/csrc/u8g2_setup.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/u8g2/csrc/u8x8_8x8.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/u8g2/csrc/u8x8_byte.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/u8g2/csrc/u8x8_cad.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/u8g2/csrc/u8x8_d_ssd1306_128x32.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/u8g2/csrc/u8x8_display.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/u8g2/csrc/u8x8_gpio.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/u8g2/csrc/u8x8_setup.c"
)
target_include_directories(${PROJECT_NAME} PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/u8g2/csrc"
)
endif()

# Add include directories
target_include_directories(${PROJECT_NAME} PRIVATE
  # Project headers
  "${CMAKE_CURRENT_SOURCE_DIR}/Inc"
  # CMSIS Core
  "${CMAKE_CURRENT_SOURCE_DIR}/../STM32CubeL0/Drivers/CMSIS/Core/Include"
  # CMSIS Device
  "${CMAKE_CURRENT_SOURCE_DIR}/../STM32CubeL0/Drivers/CMSIS/Device/ST/STM32L0xx/Include"
  # LL/HAL Driver headers
  "${CMAKE_CURRENT_SOURCE_DIR}/../STM32CubeL0/Drivers/STM32L0xx_HAL_Driver/Inc"
)

# Compiler options
target_compile_options(${PROJECT_NAME} PRIVATE
  # User defined compiler options
)

# Add additional defined symbols
target_compile_definitions(${PROJECT_NAME} PRIVATE
  # Target device: STM32L051C8T6
  STM32L051xx
  EC_ENABLE_OLED=$<BOOL:${EC_ENABLE_OLED}>
)

# Linker options
target_link_options(${PROJECT_NAME} PRIVATE 
  # User defined linker options
)

# Add additional linker search directories
target_link_directories(${PROJECT_NAME} PRIVATE
  # Add additional linker search directories here
)

# Add additional linker libraries
target_link_libraries(${PROJECT_NAME} PRIVATE
  # Add additional linker libraries here
)
