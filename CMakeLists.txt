#此文件从模板自动生成! 请勿更改!
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.20)

# specify cross-compilers and tools
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER  arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# project settings
project(uav_baice_framework_v1.6 C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

# --- 1. 宏定义 (关键：解决 __FPU_PRESENT 报错) ---
add_definitions(-DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx)
add_definitions(-D__FPU_PRESENT=1 -DARM_MATH_CM7 -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING)

# --- 2. 编译选项 (关键：修复 unrecognized argument 错误) ---
# 注意：这里不要加引号，确保每个参数是独立的
add_compile_options(-mcpu=cortex-m7 -mthumb -mthumb-interwork)
add_compile_options(-mfloat-abi=hard)
add_compile_options(-mfpu=fpv5-d16)

add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)

# --- 3. 链接选项 ---
add_link_options(-mcpu=cortex-m7 -mthumb -mthumb-interwork)
add_link_options(-mfloat-abi=hard)
add_link_options(-mfpu=fpv5-d16)

# Enable assembler files preprocessing
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g)
endif ()

include_directories(Core/Inc Drivers/STM32H7xx_HAL_Driver/Inc Drivers/STM32H7xx_HAL_Driver/Inc/Legacy Middlewares/Third_Party/FreeRTOS/Source/include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F Drivers/CMSIS/Device/ST/STM32H7xx/Include Drivers/CMSIS/Include
        bsp bsp/bsp_usart bsp/bsp_dwt bsp/bsp_usb bsp/bsp_log
        modules modules/BMI088 modules/Message_center modules/Algorithm/pose_calculating modules/Algorithm/Filter modules/IMU modules/BMI270 modules/Algorithm/PID modules/Algorithm/User_math modules/SPL06 modules/SD_Card modules/QMC5883L
        modules/Remote/Sbus modules/TFmini_Plus modules/Remote/ibus modules/Peripheral_device_testing modules/Monitor
        application application/quadrotor application/SensorHub application/uav_cmd
        USB_DEVICE USB_DEVICE/App USB_DEVICE/Target
        FATFS FATFS/App FATFS/Target
        Middlewares/ST/ARM/DSP/Inc
        Middlewares/ST/STM32_USB_Device_Library/Core/Inc
        Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
        Middlewares/Third_Party/SEGGER/RTT Middlewares/Third_Party/SEGGER/Config Middlewares/Third_Party/SEGGER/SysView
        Middlewares/Third_Party/FatFs/src
)
link_directories(Middlewares/ST/ARM/DSP/Lib)

file(GLOB_RECURSE SOURCES "Core/*.*"
        "Middlewares/*.*"
        "Drivers/*.*"
        "bsp/*.*"
        "modules/*.*"
        "application/*.*"
        "USB_DEVICE/*.*"
        "FATFS/*.*"
)

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32H743VITX_FLASH.ld)

add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-T ${LINKER_SCRIPT})

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})
# 链接数学库
target_link_libraries(${PROJECT_NAME}.elf PRIVATE arm_cortexM7lfsp_math)

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")