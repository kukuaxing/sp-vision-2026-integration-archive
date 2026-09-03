cmake_minimum_required(VERSION 3.20)

project(rm_infantry_stm32f405 LANGUAGES C CXX ASM)

if(NOT DEFINED STM32_CUBE_F4_ROOT)
  message(FATAL_ERROR "Pass -DSTM32_CUBE_F4_ROOT=<STM32CubeF4 v1.24.1 path>")
endif()

cmake_path(ABSOLUTE_PATH STM32_CUBE_F4_ROOT NORMALIZE)

set(APP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/STM32F405")
if(NOT DEFINED STM32_HAL_F4_ROOT)
  set(STM32_HAL_F4_ROOT "${STM32_CUBE_F4_ROOT}/Drivers/STM32F4xx_HAL_Driver")
endif()
cmake_path(ABSOLUTE_PATH STM32_HAL_F4_ROOT NORMALIZE)
set(HAL_DIR "${STM32_HAL_F4_ROOT}")
set(CMSIS_DIR "${STM32_CUBE_F4_ROOT}/Drivers/CMSIS")
set(FREERTOS_DIR "${STM32_CUBE_F4_ROOT}/Middlewares/Third_Party/FreeRTOS/Source")

set(REQUIRED_DEPENDENCIES
  "${HAL_DIR}/Src/stm32f4xx_hal.c"
  "${CMSIS_DIR}/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f405xx.s"
  "${FREERTOS_DIR}/tasks.c"
  "${FREERTOS_DIR}/CMSIS_RTOS/cmsis_os.c"
)
foreach(REQUIRED_FILE IN LISTS REQUIRED_DEPENDENCIES)
  if(NOT EXISTS "${REQUIRED_FILE}")
    message(FATAL_ERROR "Required STM32CubeF4 dependency is missing: ${REQUIRED_FILE}")
  endif()
endforeach()

set(APP_SOURCES
  "${APP_DIR}/can.cpp"
  "${APP_DIR}/control.cpp"
  "${APP_DIR}/CRC.cpp"
  "${APP_DIR}/gpio.cpp"
  "${APP_DIR}/HTmotor.cpp"
  "${APP_DIR}/imu.cpp"
  "${APP_DIR}/judgement.cpp"
  "${APP_DIR}/label.cpp"
  "${APP_DIR}/motor.cpp"
  "${APP_DIR}/Power_read.cpp"
  "${APP_DIR}/RC.cpp"
  "${APP_DIR}/STM32F405.cpp"
  "${APP_DIR}/supercap.cpp"
  "${APP_DIR}/system_stm32f4xx.c"
  "${APP_DIR}/taskslist.cpp"
  "${APP_DIR}/tim.cpp"
  "${APP_DIR}/usart.cpp"
  "${APP_DIR}/xuc.cpp"
  "${APP_DIR}/xuc_aim.cpp"
)
file(GLOB HAL_C CONFIGURE_DEPENDS "${HAL_DIR}/Src/*.c")
list(FILTER HAL_C EXCLUDE REGEX "_template\\.c$")
list(FILTER HAL_C EXCLUDE REGEX "stm32f4xx_hal_dfsdm\\.c$")

set(FREERTOS_C
  "${FREERTOS_DIR}/croutine.c"
  "${FREERTOS_DIR}/event_groups.c"
  "${FREERTOS_DIR}/list.c"
  "${FREERTOS_DIR}/queue.c"
  "${FREERTOS_DIR}/tasks.c"
  "${FREERTOS_DIR}/timers.c"
  "${FREERTOS_DIR}/CMSIS_RTOS/cmsis_os.c"
  "${FREERTOS_DIR}/portable/GCC/ARM_CM3/port.c"
  "${FREERTOS_DIR}/portable/MemMang/heap_4.c"
)

add_executable(stm32f405_xuc_v2
  ${APP_SOURCES}
  ${HAL_C}
  ${FREERTOS_C}
  "${CMSIS_DIR}/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f405xx.s"
)

set_target_properties(stm32f405_xuc_v2 PROPERTIES
  C_STANDARD 11
  C_STANDARD_REQUIRED YES
  C_EXTENSIONS YES
  CXX_STANDARD 11
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS YES
  OUTPUT_NAME "stm32f405_xuc_v2"
  SUFFIX ".elf"
)

target_compile_definitions(stm32f405_xuc_v2 PRIVATE
  ARM_MATH_CM4
  STM32F405RG
  stm32_flash_layout
  STM32F405xx
  LSE_STARTUP_TIMEOUT=5000U
)

target_include_directories(stm32f405_xuc_v2 PRIVATE
  "${APP_DIR}"
  "${HAL_DIR}/Inc"
  "${HAL_DIR}/Inc/Legacy"
  "${CMSIS_DIR}/Device/ST/STM32F4xx/Include"
  "${CMSIS_DIR}/Include"
  "${CMSIS_DIR}/RTOS/Template"
  "${FREERTOS_DIR}/CMSIS_RTOS"
  "${FREERTOS_DIR}/include"
  "${FREERTOS_DIR}/portable/GCC/ARM_CM3"
)

set(MCU_FLAGS
  -mcpu=cortex-m4
  -mthumb
  -mfloat-abi=soft
  -mfpu=fpv4-sp-d16
)

target_compile_options(stm32f405_xuc_v2 PRIVATE
  ${MCU_FLAGS}
  -ffunction-sections
  -fdata-sections
  -fno-common
  $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
  $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
  $<$<CONFIG:Debug>:-Og>
  $<$<CONFIG:Debug>:-g3>
  $<$<CONFIG:Release>:-Os>
)

set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/linker/STM32F405RG_flash.ld")
target_link_options(stm32f405_xuc_v2 PRIVATE
  ${MCU_FLAGS}
  "-T${LINKER_SCRIPT}"
  --specs=nano.specs
  --specs=nosys.specs
  -Wl,--gc-sections
  "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/stm32f405_xuc_v2.map"
)

target_link_libraries(stm32f405_xuc_v2 PRIVATE m)

add_custom_command(TARGET stm32f405_xuc_v2 POST_BUILD
  COMMAND "${CMAKE_OBJCOPY}" -O binary
          "$<TARGET_FILE:stm32f405_xuc_v2>"
          "${CMAKE_CURRENT_BINARY_DIR}/stm32f405_xuc_v2.bin"
  COMMAND "${CMAKE_OBJCOPY}" -O ihex
          "$<TARGET_FILE:stm32f405_xuc_v2>"
          "${CMAKE_CURRENT_BINARY_DIR}/stm32f405_xuc_v2.hex"
  COMMAND "${CMAKE_SIZE}" "$<TARGET_FILE:stm32f405_xuc_v2>"
  VERBATIM
)
