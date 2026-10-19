cmake_minimum_required(VERSION 3.27)

# Roots
include("../roots.cmake")

set(CONTEXT servo_v1+servo_v1)
set(TARGET ${CONTEXT})
set(DNAME STM32G431VBTx)
set(DPACK Keil::STM32G4xx_DFP@2.2.0)
set(DPACK_DIR "${CMSIS_PACK_ROOT}/Keil/STM32G4xx_DFP/2.2.0")
set(OUT_DIR "${SOLUTION_ROOT}/out/servo_v1/servo_v1")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_COMPILE_COMMANDS ${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json)
set(COMPILE_COMMANDS ${OUT_DIR}/compile_commands.json)
set(COMPILE_MACROS_C ${OUT_DIR}/compile_macros_c.h)
set(LD_SCRIPT "${SOLUTION_ROOT}/servo_v1_servo_v1.sct")
set(LD_SCRIPT_PP ${LD_SCRIPT})

# Processor Options
set(CPU Cortex-M4)
set(FPU SP_FPU)
set(DSP DSP)
set(BYTE_ORDER Little-endian)

# Preprocessor options
set(CPP_OPTIONS_C "-xc" "-std=c99" "-fno-rtti" "-funsigned-char" "-fshort-enums" "-fshort-wchar" "-ffunction-sections" "-Wno-packed" "-Wno-missing-variable-declarations" "-Wno-missing-prototypes" "-Wno-missing-noreturn" "-Wno-sign-conversion" "-Wno-nonportable-include-path" "-Wno-reserved-id-macro" "-Wno-unused-macros" "-Wno-documentation-unknown-command" "-Wno-documentation" "-Wno-license-management" "-Wno-parentheses-equality")

# Toolchain config map
set(COMPILER AC6)
include("toolchain.cmake")

# Setup project
project(${CONTEXT} LANGUAGES ASM C)

# Enable color diagnostics
set(CMAKE_COLOR_DIAGNOSTICS ON)

# Compilation database
add_custom_target(database DEPENDS ${COMPILE_COMMANDS} ${COMPILE_MACROS_C})
add_custom_command(OUTPUT ${COMPILE_COMMANDS}
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_COMPILE_COMMANDS}" "${COMPILE_COMMANDS}"
  DEPENDS "${CMAKE_COMPILE_COMMANDS}"
)
add_custom_command(OUTPUT ${COMPILE_MACROS_C}
  COMMAND ${CPP} ${CPP_OPTIONS_C} ${CPP_DUMP_MACROS} "${COMPILE_MACROS_C}"
)
set(CMAKE_C_STANDARD_INCLUDE_DIRECTORIES ${CMAKE_C_IMPLICIT_INCLUDE_DIRECTORIES})

# Setup context
add_executable(${CONTEXT})
set_target_properties(${CONTEXT} PROPERTIES PREFIX "" SUFFIX ".axf" OUTPUT_NAME "servo_v1")
set_target_properties(${CONTEXT} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${OUT_DIR})
add_library(${CONTEXT}_GLOBAL INTERFACE)

# Includes
target_include_directories(${CONTEXT} PUBLIC
  $<$<COMPILE_LANGUAGE:C,CXX>:
    "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Inc"
    "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy"
    "${SOLUTION_ROOT}/../Drivers/CMSIS/Device/ST/STM32G4xx/Include"
    "${SOLUTION_ROOT}/../Drivers/CMSIS/Include"
  >
  "${SOLUTION_ROOT}/RTE/_servo_v1"
  "${CMSIS_PACK_ROOT}/ARM/CMSIS/6.3.0/CMSIS/Core/Include"
  "${SOLUTION_ROOT}/../Core/Inc"
  "${SOLUTION_ROOT}/../Modules/Driver_Communication/Inc"
  "${SOLUTION_ROOT}/../Modules/Three_Phase_Driver/Inc"
  "${SOLUTION_ROOT}/../Modules/Encoder/Inc"
  "${SOLUTION_ROOT}/../Tests/Inc"
  "${SOLUTION_ROOT}/../Modules/Current_Sampling/Inc"
  "${SOLUTION_ROOT}/../FOC/Inc"
  "${SOLUTION_ROOT}/../FOC/Src"
  "${SOLUTION_ROOT}/../Servo/Inc"
)

# Defines
target_compile_definitions(${CONTEXT} PUBLIC
  $<$<COMPILE_LANGUAGE:C,CXX>:
    USE_HAL_DRIVER
    STM32G431xx
    STM32G431xx
    _RTE_
  >
)

# Compile options
target_compile_options(${CONTEXT} PUBLIC
  $<$<COMPILE_LANGUAGE:ASM>:
    "SHELL:${ASM_CPU}"
    "SHELL:${ASM_FLAGS}"
    "SHELL:${ASM_BYTE_ORDER}"
    "SHELL:-masm=auto"
  >
  $<$<COMPILE_LANGUAGE:C>:
    "SHELL:${CC_CPU}"
    "SHELL:${CC_FLAGS}"
    "SHELL:${CC_BYTE_ORDER}"
    "SHELL:-std=c99"
    "SHELL:-fno-rtti"
    "SHELL:-funsigned-char"
    "SHELL:-fshort-enums"
    "SHELL:-fshort-wchar"
    "SHELL:-ffunction-sections"
    "SHELL:-Wno-packed"
    "SHELL:-Wno-missing-variable-declarations"
    "SHELL:-Wno-missing-prototypes"
    "SHELL:-Wno-missing-noreturn"
    "SHELL:-Wno-sign-conversion"
    "SHELL:-Wno-nonportable-include-path"
    "SHELL:-Wno-reserved-id-macro"
    "SHELL:-Wno-unused-macros"
    "SHELL:-Wno-documentation-unknown-command"
    "SHELL:-Wno-documentation"
    "SHELL:-Wno-license-management"
    "SHELL:-Wno-parentheses-equality"
  >
)

# Compile Options Abstractions
add_library(${CONTEXT}_ABSTRACTIONS INTERFACE)
cbuild_set_options_flags(ASM "debug" "on" "" "" ASM_OPTIONS_FLAGS_${CONTEXT})
cbuild_set_options_flags(CC "debug" "on" "" "" CC_OPTIONS_FLAGS_${CONTEXT})
target_compile_options(${CONTEXT}_ABSTRACTIONS INTERFACE
  $<$<COMPILE_LANGUAGE:ASM>:
    "SHELL:${ASM_OPTIONS_FLAGS_${CONTEXT}}"
  >
  $<$<COMPILE_LANGUAGE:C>:
    "SHELL:${CC_OPTIONS_FLAGS_${CONTEXT}}"
  >
)

# Add groups and components
include("groups.cmake")
include("components.cmake")

target_link_libraries(${CONTEXT} PUBLIC
  Group_Application_Startup
  Group_Application_Core
  Group_Modules_Driver_Communication
  Group_Modules_Three-Phase_Driver
  Group_Modules_Encoder
  Group_Tests
  Group_Modules_Current_Sampling
  Group_Modules_FOC
  Group_Application_Servo_Manager
  Group_Drivers_STM32G4xx_HAL_Driver
  Group_Drivers_CMSIS
  ARM_CMSIS_CORE_6_2_0
)

# Linker options
target_link_options(${CONTEXT} PUBLIC
  "SHELL:${LD_CPU}"
  "SHELL:${_LS}\"${LD_SCRIPT_PP}\""
  "SHELL:--map"
  "SHELL:--load_addr_map_info"
  "SHELL:--xref"
  "SHELL:--callgraph"
  "SHELL:--symbols"
  "SHELL:--info sizes"
  "SHELL:--info totals"
  "SHELL:--info unused"
  "SHELL:--info veneers"
  "SHELL:--strict"
  "SHELL:--summary_stderr"
  "SHELL:--info summarysizes"
)
set(LD_DEPS ${LD_SCRIPT})
set_target_properties(${CONTEXT} PROPERTIES LINK_DEPENDS "${LD_DEPS}")
