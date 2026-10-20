# ============================================================================
#  RoboMaster Sentinel RCS - STM32F405RG (FreeRTOS + STM32 HAL)
#  Linux port of the original VisualGDB (Windows) project.
#
#  Compile/link flags mirror the original VisualGDB-generated flags exactly
#  (see the old STM32F405/VisualGDB/Debug/*.gcc.rsp and *.link.rsp files).
#
#  Usage:
#    cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug
#    cmake --build build
# ============================================================================
cmake_minimum_required(VERSION 3.16)
project(RCS_STM32F405 C CXX ASM)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)   # for VS Code IntelliSense

# Toolchain: see cmake/arm-none-eabi-toolchain.cmake (pass -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-toolchain.cmake)
# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
set(PROJ_SRC_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/STM32F405)
set(BSP_DIR       ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty)

set(HAL_INC       ${BSP_DIR}/STM32F4xx_HAL/Inc)
set(HAL_SRC       ${BSP_DIR}/STM32F4xx_HAL/Src)
set(CMSIS_CORE    ${BSP_DIR}/CMSIS/Include)
set(CMSIS_DEV     ${BSP_DIR}/CMSIS/Device/ST/STM32F4xx/Include)
set(FREERTOS      ${BSP_DIR}/FreeRTOS/Source)

set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/linker/STM32F405RGTx_FLASH.ld)

# ---------------------------------------------------------------------------
# Flags (mirror VisualGDB .gcc.rsp / .link.rsp)
# ---------------------------------------------------------------------------
set(MCU_FLAGS -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -mfpu=fpv4-sp-d16)

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(OPT_FLAGS -O2 -g)
else()
  set(OPT_FLAGS -O0 -ggdb)
endif()

add_compile_definitions(
  ARM_MATH_CM4
  STM32F405RG
  stm32_flash_layout
  STM32F405xx
)
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_definitions(NDEBUG=1)
else()
  add_compile_definitions(DEBUG=1)
endif()

# ---------------------------------------------------------------------------
# Sources
# ---------------------------------------------------------------------------
set(USER_SOURCES
  ${PROJ_SRC_DIR}/CRC.cpp
  ${PROJ_SRC_DIR}/HTmotor.cpp
  ${PROJ_SRC_DIR}/Power_read.cpp
  ${PROJ_SRC_DIR}/RC.cpp
  ${PROJ_SRC_DIR}/STM32F405.cpp
  ${PROJ_SRC_DIR}/can.cpp
  ${PROJ_SRC_DIR}/control.cpp
  ${PROJ_SRC_DIR}/gpio.cpp
  ${PROJ_SRC_DIR}/imu.cpp
  ${PROJ_SRC_DIR}/judgement.cpp
  ${PROJ_SRC_DIR}/label.cpp
  ${PROJ_SRC_DIR}/motor.cpp
  ${PROJ_SRC_DIR}/supercap.cpp
  ${PROJ_SRC_DIR}/system_stm32f4xx.c
  ${PROJ_SRC_DIR}/taskslist.cpp
  ${PROJ_SRC_DIR}/tim.cpp
  ${PROJ_SRC_DIR}/usart.cpp
  ${PROJ_SRC_DIR}/xuc.cpp
)

set(FREERTOS_SOURCES
  ${FREERTOS}/croutine.c
  ${FREERTOS}/event_groups.c
  ${FREERTOS}/list.c
  ${FREERTOS}/queue.c
  ${FREERTOS}/tasks.c
  ${FREERTOS}/timers.c
  ${FREERTOS}/CMSIS_RTOS/cmsis_os.c
  ${FREERTOS}/portable/GCC/ARM_CM3/port.c
  ${FREERTOS}/portable/MemMang/heap_4.c
)

file(GLOB HAL_SOURCES ${HAL_SRC}/*.c)          # all HAL Src (same as original)
list(FILTER HAL_SOURCES EXCLUDE REGEX ".*template\\.c$")  # drop *_template.c

set(STARTUP_SOURCE
  ${BSP_DIR}/startup/startup_stm32f405xx.s
)

# ---------------------------------------------------------------------------
# Target
# ---------------------------------------------------------------------------
add_executable(STM32F405.elf
  ${USER_SOURCES}
  ${FREERTOS_SOURCES}
  ${HAL_SOURCES}
  ${STARTUP_SOURCE}
)
set_target_properties(STM32F405.elf PROPERTIES OUTPUT_NAME STM32F405.elf)

target_include_directories(STM32F405.elf PRIVATE
  ${PROJ_SRC_DIR}                          # user headers, FreeRTOSConfig.h, stm32f4xx_hal_conf.h
  ${HAL_INC}
  ${HAL_INC}/Legacy
  ${CMSIS_DEV}
  ${CMSIS_CORE}
  ${FREERTOS}/CMSIS_RTOS                    # so "#include <../CMSIS_RTOS/cmsis_os.h>" resolves
  ${FREERTOS}/include
  ${FREERTOS}/portable/GCC/ARM_CM3
)

target_compile_options(STM32F405.elf PRIVATE
  $<$<COMPILE_LANGUAGE:C>:   ${MCU_FLAGS} ${OPT_FLAGS} -ffunction-sections -fdata-sections>
  $<$<COMPILE_LANGUAGE:CXX>: ${MCU_FLAGS} ${OPT_FLAGS} -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti>
  $<$<COMPILE_LANGUAGE:ASM>: ${MCU_FLAGS}>
)

target_link_options(STM32F405.elf PRIVATE
  ${MCU_FLAGS}
  -Wl,--gc-sections
  -Wl,-Map=${CMAKE_BINARY_DIR}/STM32F405.map
  --specs=nano.specs
  --specs=nosys.specs
  -T${LINKER_SCRIPT}
)

# ---------------------------------------------------------------------------
# Post-build: produce .bin + memory usage report
# ---------------------------------------------------------------------------
add_custom_command(TARGET STM32F405.elf POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary
          $<TARGET_FILE:STM32F405.elf>
          ${CMAKE_BINARY_DIR}/STM32F405.bin
  COMMAND ${CMAKE_SIZE} $<TARGET_FILE:STM32F405.elf>
  COMMENT "objcopy -> STM32F405.bin / size report"
)
