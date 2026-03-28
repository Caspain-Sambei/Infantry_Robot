cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

set(CMAKE_SYSTEM_NAME Generic)##新添加，强制设置系统为 Generic
set(CMAKE_SYSTEM_PROCESSOR ARM)##

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME gimbal)

# 添加 C++ 标准配置（兼容 STM32）
set(CMAKE_CXX_STANDARD 17)  # 选11/14/17都可以，17兼容性更好
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 设置系统为 Generic，避免 Windows 特定选项
set(CMAKE_SYSTEM_NAME Generic)##

# 在包含任何其他内容之前设置
set(CMAKE_EXECUTABLE_SUFFIX ".elf")##---设置目标类型为嵌入式---
set(CMAKE_EXECUTABLE_SUFFIX_C .elf)##
set(CMAKE_EXECUTABLE_SUFFIX_CXX .elf)##
set(CMAKE_EXECUTABLE_SUFFIX_ASM .elf)##

# 禁用共享库
set(BUILD_SHARED_LIBS OFF)##
set(CMAKE_POSITION_INDEPENDENT_CODE OFF)##

# 强制设置输出类型
set(CMAKE_EXECUTABLE_SUFFIX ".elf")##

# 覆盖链接命令，移除Windows选项
set(CMAKE_C_LINK_EXECUTABLE
"<CMAKE_C_COMPILER> <FLAGS> <CMAKE_C_LINK_FLAGS> <LINK_FLAGS> <OBJECTS> -o <TARGET> <LINK_LIBRARIES>"
)##
set(CMAKE_CXX_LINK_EXECUTABLE
"<CMAKE_CXX_COMPILER> <FLAGS> <CMAKE_CXX_LINK_FLAGS> <LINK_FLAGS> <OBJECTS> -o <TARGET> <LINK_LIBRARIES>"
)##

include("cmake/gcc-arm-none-eabi.cmake") #傻逼CubeMX别删我这行

# 设置链接器命令格式
set(CMAKE_C_LINK_EXECUTABLE
"${CMAKE_C_COMPILER} <FLAGS> <CMAKE_C_LINK_FLAGS> <LINK_FLAGS> <OBJECTS> -o <TARGET> <LINK_LIBRARIES>"
)##

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        Module/reg.h
        Module/reg.c
        bsp/Inc/MyCAN.h
        bsp/Src/MyCAN.c
        bsp/Inc/MyUSB.h
        bsp/Src/MyUSB.c
        App/gimbal/gimbal.h
        App/gimbal/gimbal.c
        bsp/Inc/bsp_rc.h
        bsp/Src/bsp_rc.c
        App/rc/rc.h
        App/rc/rc.c
        Module/KalmanFilter.h
        Module/KalmanFilter.c
        bsp/Src/bsp_uart.c
        bsp/Inc/bsp_uart.h
        bsp/Src/bsp_Motor.c
        bsp/Inc/bsp_Motor.h
        App/chassis/chassis.c
        App/chassis/chassis.h
        App/rc/Mecanum_wheel.h
        App/rc/Mecanum_wheel.c
        App/rc/Omni_wheel.h
        App/rc/Omni_wheel.c)

# 禁用共享库
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE OFF)

# ========== 关键：设置链接脚本 ==========
target_link_options(${PROJECT_NAME} PRIVATE
-T "${CMAKE_SOURCE_DIR}/STM32F405XX_FLASH.ld"  # 对应F405RGT6的链接脚本
)

# 确保生成正确的.elf文件
set_target_properties(${PROJECT_NAME} PROPERTIES
OUTPUT_NAME "${PROJECT_NAME}"
SUFFIX ".elf"
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# 生成hex和bin文件
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
COMMAND ${CMAKE_OBJCOPY} -O binary -S ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
COMMENT "生成Hex和Bin文件"
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
        Module/PID.c
        Module/PID.h
        bsp/Src/BMI088driver.c
        bsp/Src/BMI088Middleware.c
        bsp/Src/bsp_delay.c
        bsp/Src/bsp_spi.c
        Module/MahonyAHRS.c
        Module/MahonyAHRS.h
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        bsp/Inc/
        bsp/Src/
        Module/
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
