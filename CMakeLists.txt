# STM32 Minimal CMake project for C/C++ projects
cmake_minimum_required(VERSION 3.12)
###############################################################################

include(./gcc-arm-none-eabi.cmake)

# Set project name and some defaults
if ("${PROJECT_NAME}" STREQUAL "")
    set(PROJECT_NAME firmware)
endif()
project(${PROJECT_NAME})
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set debug mode as default
if ("${CMAKE_BUILD_TYPE}" STREQUAL "")
    set(CMAKE_BUILD_TYPE Debug)
endif()
message("Build type: ${CMAKE_BUILD_TYPE}")

# Set microcontroller information
set(MCU_FAMILY STM32G4xx)
set(MCU_MODEL STM32G431xx)
set(MCSDK_VERSION 6.1.2)
set(CPU_PARAMETERS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard)

set(STARTUP_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/startup_stm32g431xx.s)
set(MCU_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/STM32G431CBUX_FLASH.ld)

###############################################################################
set(EXECUTABLE ${CMAKE_PROJECT_NAME})

enable_language(C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})
set(CMAKE_CXX_EXTENSIONS ON)
###############################################################################

set(STM32CUBEMX_INCLUDE_DIRECTORIES
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/${MCU_FAMILY}_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/${MCU_FAMILY}_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v${MCSDK_VERSION}-Full/MotorControl/MCSDK/MCLib/Any/Inc 
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v${MCSDK_VERSION}-Full/MotorControl/MCSDK/MCLib/G4xx/Inc 
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/${MCU_FAMILY}/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
   # ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/RTOS2/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/include
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    )

set(PROJECT_INCLUDE_DIRECTORIES 
    ${CMAKE_CURRENT_SOURCE_DIR}/Project/Inc
    )

file(GLOB_RECURSE STM32CUBEMX_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_utils.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_exti.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_adc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_dma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ramfunc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_cordic.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_opamp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_usart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_rcc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_comp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_dac.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/tasks.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/list.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v${MCSDK_VERSION}-Full/MotorControl/MCSDK/MCLib/Any/Src/bus_voltage_sensor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v${MCSDK_VERSION}-Full/MotorControl/MCSDK/MCLib/Any/Src/circle_limitation.c
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v${MCSDK_VERSION}-Full/MotorControl/MCSDK/MCLib/Any/Src/digital_output.c
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v${MCSDK_VERSION}-Full/MotorControl/MCSDK/MCLib/Any/Src/ntc_temperature_sensor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v${MCSDK_VERSION}-Full/MotorControl/MCSDK/MCLib/Any/Src/open_loop.c
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v${MCSDK_VERSION}-Full/MotorControl/MCSDK/MCLib/Any/Src/pid_regulator.c
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v${MCSDK_VERSION}-Full/MotorControl/MCSDK/MCLib/Any/Src/pqd_motor_power_measurement.c
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v${MCSDK_VERSION}-Full/MotorControl/MCSDK/MCLib/Any/Src/pwm_common.c
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v${MCSDK_VERSION}-Full/MotorControl/MCSDK/MCLib/Any/Src/r_divider_bus_voltage_sensor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v${MCSDK_VERSION}-Full/MotorControl/MCSDK/MCLib/Any/Src/ramp_ext_mngr.c
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v${MCSDK_VERSION}-Full/MotorControl/MCSDK/MCLib/Any/Src/revup_ctrl.c
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v${MCSDK_VERSION}-Full/MotorControl/MCSDK/MCLib/Any/Src/speed_pos_fdbk.c
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v${MCSDK_VERSION}-Full/MotorControl/MCSDK/MCLib/Any/Src/speed_torq_ctrl.c
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v${MCSDK_VERSION}-Full/MotorControl/MCSDK/MCLib/Any/Src/sto_cordic_speed_pos_fdbk.c
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v${MCSDK_VERSION}-Full/MotorControl/MCSDK/MCLib/Any/Src/sto_pll_speed_pos_fdbk.c
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v${MCSDK_VERSION}-Full/MotorControl/MCSDK/MCLib/Any/Src/virtual_speed_sensor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v${MCSDK_VERSION}-Full/MotorControl/MCSDK/MCLib/G4xx/Src/r3_2_g4xx_pwm_curr_fdbk.c
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v${MCSDK_VERSION}-Full/MotorControl/MCSDK/MCLib/Any/Src/mcp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v${MCSDK_VERSION}-Full/MotorControl/MCSDK/MCLib/Any/Src/mcpa.c
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v${MCSDK_VERSION}-Full/MotorControl/MCSDK/MCLib/Any/Src/feed_forward_ctrl.c
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v${MCSDK_VERSION}-Full/MotorControl/MCSDK/MCLib/Any/Src/encoder_speed_pos_fdbk.c
    ${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v${MCSDK_VERSION}-Full/MotorControl/MCSDK/MCLib/Any/Src/enc_align_ctrl.c
    #${CMAKE_CURRENT_SOURCE_DIR}/MCSDK_v${MCSDK_VERSION}-Full/MotorControl/MCSDK/MCLib/Any/Src/esc.c
    )

set(PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Project)
file(GLOB_RECURSE PROJECT_SOURCES CONFIGURE_DEPENDS
    ${PROJECT_DIR}/*.cpp
    ${PROJECT_DIR}/*.c)

add_executable(${EXECUTABLE}
    ${STM32CUBEMX_SOURCES} 
    ${PROJECT_SOURCES}
    ${STARTUP_SCRIPT})

target_compile_definitions(${EXECUTABLE} PRIVATE
    #$<$<CONFIG:Debug>:DEBUG>
    ${MCU_MODEL}
    USE_HAL_DRIVER
    USE_FULL_LL_DRIVER
    ARM_MATH_CM4)

target_include_directories(${EXECUTABLE} SYSTEM PRIVATE
    ${STM32CUBEMX_INCLUDE_DIRECTORIES})

target_include_directories(${EXECUTABLE} PRIVATE
    ${PROJECT_INCLUDE_DIRECTORIES})
###############################################################################
target_compile_options(${EXECUTABLE} PRIVATE
    ${CPU_PARAMETERS}
    -Wall
    -Wextra
    -Wpedantic
    -Wshadow
    -Wdouble-promotion
    -Wformat=2 -Wformat-truncation
    -Wundef
    -fno-common
    -Wno-unused-parameter
    $<$<COMPILE_LANGUAGE:CXX>:
        -Wconversion
        -Wno-volatile
        -Wold-style-cast
        -Wuseless-cast
        -Wsuggest-override>
    $<$<CONFIG:Debug>:-O0 -g3 -ggdb>
    $<$<CONFIG:Release>:-Og -g0>)

target_link_options(${EXECUTABLE} PRIVATE
    -T${MCU_LINKER_SCRIPT}
    ${CPU_PARAMETERS}
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
    $<$<VERSION_GREATER:$<C_COMPILER_VERSION>,10.3.1>:-Wl,--no-warn-rwx-segments>
    -Wl,--start-group
    -lc
    -lm
    -lstdc++
    -lnosys
    -u _printf_float
    -Wl,--end-group
    -Wl,--print-memory-usage)
###############################################################################

# The last command can take a couple of seconds on larger project, usefull for debugging
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${EXECUTABLE}>
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${EXECUTABLE}> ${EXECUTABLE}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${EXECUTABLE}> ${EXECUTABLE}.bin)

option(DUMP_ASM "Create full assembly of final executable" OFF)
if (${DUMP_ASM})
    add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
        COMMAND ${CMAKE_OBJDUMP} -D -C $<TARGET_FILE:${EXECUTABLE}> > ${EXECUTABLE}.s)
endif()

