cmake_minimum_required(VERSION 3.22)
project(board LANGUAGES C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()
add_executable(board)
add_subdirectory(cmake/stm32cubemx)

# Keep the board-owned HAL timebase after CubeMX regeneration.
get_target_property(mx_sources board SOURCES)
list(FILTER mx_sources EXCLUDE REGEX "stm32f4xx_hal_timebase_tim\\.c$")
set_property(TARGET board PROPERTY SOURCES "${mx_sources}")
# Generated handlers remain available under separate names; the RTOS owns vectors.
set_source_files_properties(Core/Src/stm32f4xx_it.c PROPERTIES COMPILE_DEFINITIONS
    "PendSV_Handler=CubeMX_PendSV_Handler;SysTick_Handler=CubeMX_SysTick_Handler;TIM6_DAC_IRQHandler=CubeMX_TIM6_DAC_IRQHandler;USART1_IRQHandler=CubeMX_USART1_IRQHandler;USART6_IRQHandler=CubeMX_USART6_IRQHandler")

set(FRAMEWORK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")
file(GLOB framework_sources CONFIGURE_DEPENDS
    "${FRAMEWORK_ROOT}/bsp/src/*.cpp"
    "${FRAMEWORK_ROOT}/libs/src/*.cpp"
    "${FRAMEWORK_ROOT}/modules/src/*.cpp"
    "${FRAMEWORK_ROOT}/threads/src/*.cpp")
target_sources(board PRIVATE ${framework_sources}
    Core/Src/threadx_port.c Core/Src/hal_timebase.c)
target_include_directories(board PRIVATE
    Core/Inc
    "${FRAMEWORK_ROOT}/bsp/include"
    "${FRAMEWORK_ROOT}/libs/include"
    "${FRAMEWORK_ROOT}/modules/include"
    "${FRAMEWORK_ROOT}/threads/include")

set(TX_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ThreadX")
file(GLOB tx_sources CONFIGURE_DEPENDS
    "${TX_ROOT}/common/src/*.c"
    "${TX_ROOT}/ports/cortex_m4/gnu/src/*.S")
add_library(threadx STATIC ${tx_sources})
target_include_directories(threadx PUBLIC
    "${TX_ROOT}/common/inc" "${TX_ROOT}/ports/cortex_m4/gnu/inc"
    "${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc")
target_compile_definitions(threadx PUBLIC TX_INCLUDE_USER_DEFINE_FILE)
target_link_libraries(board stm32cubemx threadx)
option(RM_ENABLE_HEARTBEAT_DEMO "Run the semaphore heartbeat test" ON)
option(RM_ENABLE_IMU_HEATER "Compile the optional TIM10 heater path" OFF)
target_compile_definitions(board PRIVATE
    RM_ENABLE_HEARTBEAT_DEMO=$<BOOL:${RM_ENABLE_HEARTBEAT_DEMO}>
    RM_ENABLE_IMU_HEATER=$<BOOL:${RM_ENABLE_IMU_HEATER}>)
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)
add_custom_command(TARGET board POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:board> $<TARGET_FILE_DIR:board>/board.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:board> $<TARGET_FILE_DIR:board>/board.bin)
