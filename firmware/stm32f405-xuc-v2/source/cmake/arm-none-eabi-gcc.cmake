set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

if(NOT DEFINED ARM_GCC_ROOT AND DEFINED ENV{ARM_GCC_ROOT})
  set(ARM_GCC_ROOT "$ENV{ARM_GCC_ROOT}")
endif()
if(NOT DEFINED ARM_GCC_ROOT)
  message(FATAL_ERROR "Pass -DARM_GCC_ROOT=<GNU Arm Embedded Toolchain root>")
endif()

set(CMAKE_TRY_COMPILE_PLATFORM_VARIABLES ARM_GCC_ROOT)

cmake_path(ABSOLUTE_PATH ARM_GCC_ROOT NORMALIZE)
set(ARM_GCC_BIN "${ARM_GCC_ROOT}/bin")

set(CMAKE_C_COMPILER "${ARM_GCC_BIN}/arm-none-eabi-gcc.exe")
set(CMAKE_CXX_COMPILER "${ARM_GCC_BIN}/arm-none-eabi-g++.exe")
set(CMAKE_ASM_COMPILER "${ARM_GCC_BIN}/arm-none-eabi-gcc.exe")
set(CMAKE_AR "${ARM_GCC_BIN}/arm-none-eabi-ar.exe")
set(CMAKE_OBJCOPY "${ARM_GCC_BIN}/arm-none-eabi-objcopy.exe")
set(CMAKE_OBJDUMP "${ARM_GCC_BIN}/arm-none-eabi-objdump.exe")
set(CMAKE_SIZE "${ARM_GCC_BIN}/arm-none-eabi-size.exe")

foreach(REQUIRED_TOOL
    CMAKE_C_COMPILER
    CMAKE_CXX_COMPILER
    CMAKE_AR
    CMAKE_OBJCOPY
    CMAKE_SIZE)
  if(NOT EXISTS "${${REQUIRED_TOOL}}")
    message(FATAL_ERROR "GNU Arm tool is missing: ${${REQUIRED_TOOL}}")
  endif()
endforeach()
