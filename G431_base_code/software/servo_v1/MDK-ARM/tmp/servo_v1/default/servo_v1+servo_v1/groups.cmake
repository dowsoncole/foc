# groups.cmake

# group Application/Startup
add_library(Group_Application_Startup OBJECT
  "${SOLUTION_ROOT}/startup_stm32g431xx.s"
)
target_include_directories(Group_Application_Startup PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(Group_Application_Startup PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Application_Startup_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_Application_Startup_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_Application_Startup PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_Application_Startup PUBLIC
  Group_Application_Startup_ABSTRACTIONS
)
set(COMPILE_DEFINITIONS
  STM32G431xx
  _RTE_
)
cbuild_set_defines(AS_ARM COMPILE_DEFINITIONS)
set_source_files_properties("${SOLUTION_ROOT}/startup_stm32g431xx.s" PROPERTIES
  COMPILE_FLAGS "${COMPILE_DEFINITIONS}"
)

# group Application/Core
add_library(Group_Application_Core OBJECT
  "${SOLUTION_ROOT}/../Core/Src/main.c"
  "${SOLUTION_ROOT}/../Core/Src/gpio.c"
  "${SOLUTION_ROOT}/../Core/Src/stm32g4xx_it.c"
  "${SOLUTION_ROOT}/../Core/Src/stm32g4xx_hal_msp.c"
)
target_include_directories(Group_Application_Core PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  "${SOLUTION_ROOT}/../Core/Inc"
)
target_compile_definitions(Group_Application_Core PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Application_Core_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_Application_Core_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_Application_Core PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_Application_Core PUBLIC
  Group_Application_Core_ABSTRACTIONS
)

# group Modules/Driver Communication
add_library(Group_Modules_Driver_Communication OBJECT
  "${SOLUTION_ROOT}/../Modules/Driver_Communication/Src/i2c3_bus.c"
  "${SOLUTION_ROOT}/../Modules/Driver_Communication/Src/tim6_heartbeat.c"
  "${SOLUTION_ROOT}/../Modules/Driver_Communication/Src/stspin32g4.c"
  "${SOLUTION_ROOT}/../Modules/Driver_Communication/Src/drv_monitor.c"
)
target_include_directories(Group_Modules_Driver_Communication PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  "${SOLUTION_ROOT}/../Modules/Driver_Communication/Inc"
)
target_compile_definitions(Group_Modules_Driver_Communication PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Modules_Driver_Communication_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_Modules_Driver_Communication_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_Modules_Driver_Communication PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_Modules_Driver_Communication PUBLIC
  Group_Modules_Driver_Communication_ABSTRACTIONS
)

# group Modules/Three-Phase Driver
add_library(Group_Modules_Three-Phase_Driver OBJECT
  "${SOLUTION_ROOT}/../Modules/Three_Phase_Driver/Src/tim1_pwm.c"
)
target_include_directories(Group_Modules_Three-Phase_Driver PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  "${SOLUTION_ROOT}/../Modules/Three_Phase_Driver/Inc"
)
target_compile_definitions(Group_Modules_Three-Phase_Driver PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Modules_Three-Phase_Driver_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_Modules_Three-Phase_Driver_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_Modules_Three-Phase_Driver PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_Modules_Three-Phase_Driver PUBLIC
  Group_Modules_Three-Phase_Driver_ABSTRACTIONS
)

# group Modules/Encoder
add_library(Group_Modules_Encoder OBJECT
  "${SOLUTION_ROOT}/../Modules/Encoder/Src/spi1_encoder.c"
  "${SOLUTION_ROOT}/../Modules/Encoder/Src/mt6835.c"
)
target_include_directories(Group_Modules_Encoder PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  "${SOLUTION_ROOT}/../Modules/Encoder/Inc"
)
target_compile_definitions(Group_Modules_Encoder PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Modules_Encoder_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_Modules_Encoder_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_Modules_Encoder PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_Modules_Encoder PUBLIC
  Group_Modules_Encoder_ABSTRACTIONS
)

# group Tests
add_library(Group_Tests OBJECT
  "${SOLUTION_ROOT}/../Tests/Src/tim1_pwm_verify.c"
  "${SOLUTION_ROOT}/../Tests/Src/waveform_generator.c"
  "${SOLUTION_ROOT}/../Tests/Src/mt6835_verify.c"
)
target_include_directories(Group_Tests PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  "${SOLUTION_ROOT}/../Tests/Inc"
)
target_compile_definitions(Group_Tests PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Tests_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_Tests_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_Tests PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_Tests PUBLIC
  Group_Tests_ABSTRACTIONS
)

# group Modules/Current Sampling
add_library(Group_Modules_Current_Sampling OBJECT
  "${SOLUTION_ROOT}/../Modules/Current_Sampling/Src/adc1_current_sampling.c"
  "${SOLUTION_ROOT}/../Modules/Current_Sampling/Src/current_sampling.c"
)
target_include_directories(Group_Modules_Current_Sampling PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  "${SOLUTION_ROOT}/../Modules/Current_Sampling/Inc"
)
target_compile_definitions(Group_Modules_Current_Sampling PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Modules_Current_Sampling_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_Modules_Current_Sampling_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_Modules_Current_Sampling PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_Modules_Current_Sampling PUBLIC
  Group_Modules_Current_Sampling_ABSTRACTIONS
)

# group Modules/FOC
add_library(Group_Modules_FOC OBJECT
  "${SOLUTION_ROOT}/../FOC/Src/foc_algorithm.c"
  "${SOLUTION_ROOT}/../FOC/Src/foc_interface.c"
)
target_include_directories(Group_Modules_FOC PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  "${SOLUTION_ROOT}/../FOC/Inc"
)
target_compile_definitions(Group_Modules_FOC PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Modules_FOC_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_Modules_FOC_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_Modules_FOC PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_Modules_FOC PUBLIC
  Group_Modules_FOC_ABSTRACTIONS
)

# group Drivers/STM32G4xx_HAL_Driver
add_library(Group_Drivers_STM32G4xx_HAL_Driver OBJECT
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c"
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c"
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c"
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c"
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c"
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c"
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ramfunc.c"
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c"
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c"
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c"
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c"
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c"
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c"
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_i2c.c"
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_i2c_ex.c"
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c"
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c"
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc.c"
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc_ex.c"
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_opamp.c"
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_opamp_ex.c"
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart.c"
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart_ex.c"
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi.c"
  "${SOLUTION_ROOT}/../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi_ex.c"
)
target_include_directories(Group_Drivers_STM32G4xx_HAL_Driver PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(Group_Drivers_STM32G4xx_HAL_Driver PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Drivers_STM32G4xx_HAL_Driver_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_Drivers_STM32G4xx_HAL_Driver_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_Drivers_STM32G4xx_HAL_Driver PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_Drivers_STM32G4xx_HAL_Driver PUBLIC
  Group_Drivers_STM32G4xx_HAL_Driver_ABSTRACTIONS
)

# group Drivers/CMSIS
add_library(Group_Drivers_CMSIS OBJECT
  "${SOLUTION_ROOT}/../Core/Src/system_stm32g4xx.c"
)
target_include_directories(Group_Drivers_CMSIS PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(Group_Drivers_CMSIS PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Drivers_CMSIS_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_Drivers_CMSIS_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_Drivers_CMSIS PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_Drivers_CMSIS PUBLIC
  Group_Drivers_CMSIS_ABSTRACTIONS
)
