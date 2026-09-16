cmake_minimum_required(VERSION 3.20)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(ARM_TOOLCHAIN_ROOT "C:/Program Files (x86)/GNU Arm Embedded Toolchain/10 2021.10/bin" CACHE PATH "GNU Arm toolchain bin directory")
set(CMAKE_C_COMPILER "${ARM_TOOLCHAIN_ROOT}/arm-none-eabi-gcc.exe")
set(CMAKE_CXX_COMPILER "${ARM_TOOLCHAIN_ROOT}/arm-none-eabi-g++.exe")
set(CMAKE_ASM_COMPILER "${ARM_TOOLCHAIN_ROOT}/arm-none-eabi-gcc.exe")
set(CMAKE_OBJCOPY "${ARM_TOOLCHAIN_ROOT}/arm-none-eabi-objcopy.exe")
set(CMAKE_SIZE "${ARM_TOOLCHAIN_ROOT}/arm-none-eabi-size.exe")

project(SentryUC LANGUAGES C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)

set(CUBE_ROOT "" CACHE PATH "Path to STM32CubeF4 SDK")
if(NOT EXISTS "${CUBE_ROOT}/Drivers/CMSIS")
  message(FATAL_ERROR "Set -DCUBE_ROOT to an existing STM32CubeF4 SDK")
endif()
set(APP_DIR "${CMAKE_CURRENT_LIST_DIR}/STM32F405")
set(CMSIS_DIR "${CUBE_ROOT}/Drivers/CMSIS")
set(HAL_DIR "${CUBE_ROOT}/Drivers/STM32F4xx_HAL_Driver")
set(FREERTOS_DIR "${CUBE_ROOT}/Middlewares/Third_Party/FreeRTOS/Source")

set(APP_SOURCES
  ${APP_DIR}/can.cpp
  ${APP_DIR}/control.cpp
  ${APP_DIR}/CRC.cpp
  ${APP_DIR}/DMmotor.cpp
  ${APP_DIR}/gpio.cpp
  ${APP_DIR}/imu.cpp
  ${APP_DIR}/judgement.cpp
  ${APP_DIR}/label.cpp
  ${APP_DIR}/motor.cpp
  ${APP_DIR}/PowerLimiter.cpp
  ${APP_DIR}/Power_read.cpp
  ${APP_DIR}/RC.cpp
  ${APP_DIR}/slidingmodec.cpp
  ${APP_DIR}/STM32F405.cpp
  ${APP_DIR}/supercap.cpp
  ${APP_DIR}/system_stm32f4xx.c
  ${APP_DIR}/taskslist.cpp
  ${APP_DIR}/tim.cpp
  ${APP_DIR}/usart.cpp
  ${APP_DIR}/nuc.cpp
)

file(GLOB HAL_SOURCES CONFIGURE_DEPENDS "${HAL_DIR}/Src/*.c")
list(FILTER HAL_SOURCES EXCLUDE REGEX ".*_template\\.c$")

set(FREERTOS_SOURCES
  ${FREERTOS_DIR}/croutine.c
  ${FREERTOS_DIR}/event_groups.c
  ${FREERTOS_DIR}/list.c
  ${FREERTOS_DIR}/queue.c
  ${FREERTOS_DIR}/stream_buffer.c
  ${FREERTOS_DIR}/tasks.c
  ${FREERTOS_DIR}/timers.c
  ${FREERTOS_DIR}/CMSIS_RTOS/cmsis_os.c
  ${FREERTOS_DIR}/portable/GCC/ARM_CM3/port.c
  ${FREERTOS_DIR}/portable/MemMang/heap_4.c
)

add_executable(sentry_uc.elf
  ${APP_SOURCES}
  ${HAL_SOURCES}
  ${FREERTOS_SOURCES}
  ${CMSIS_DIR}/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f405xx.s
)

target_include_directories(sentry_uc.elf PRIVATE
  ${APP_DIR}
  ${HAL_DIR}/Inc
  ${HAL_DIR}/Inc/Legacy
  ${CMSIS_DIR}/Include
  ${CMSIS_DIR}/Device/ST/STM32F4xx/Include
  ${CMSIS_DIR}/RTOS/Template
  ${FREERTOS_DIR}/include
  ${FREERTOS_DIR}/CMSIS_RTOS
  ${FREERTOS_DIR}/portable/GCC/ARM_CM3
)

target_compile_definitions(sentry_uc.elf PRIVATE
  ARM_MATH_CM4
  STM32F405RG
  STM32F405xx
  LSE_STARTUP_TIMEOUT=5000U
)

target_compile_options(sentry_uc.elf PRIVATE
  -mcpu=cortex-m4
  -mthumb
  -mfloat-abi=soft
  -ffunction-sections
  -fdata-sections
  -fno-common
  $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
  $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
  $<$<CONFIG:Debug>:-Og>
  $<$<CONFIG:Release>:-O2>
  -g3
)

target_link_options(sentry_uc.elf PRIVATE
  -mcpu=cortex-m4
  -mthumb
  -mfloat-abi=soft
  -T${CMAKE_CURRENT_LIST_DIR}/STM32F405RG_FLASH.ld
  --specs=nano.specs
  --specs=nosys.specs
  -Wl,--gc-sections
  -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/sentry_uc.map
  -Wl,--print-memory-usage
)

target_link_libraries(sentry_uc.elf PRIVATE m)

add_custom_command(TARGET sentry_uc.elf POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:sentry_uc.elf> ${CMAKE_CURRENT_BINARY_DIR}/sentry_uc.hex
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:sentry_uc.elf> ${CMAKE_CURRENT_BINARY_DIR}/sentry_uc.bin
  COMMAND ${CMAKE_SIZE} $<TARGET_FILE:sentry_uc.elf>
  VERBATIM
)
