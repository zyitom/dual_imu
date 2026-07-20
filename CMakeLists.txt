cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME FirstFlightControler)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

set(IMU_CALIBRATION_PRODUCTION_BUILD "1" CACHE STRING
    "Production interlock for IMU calibration")
set(IMU_CALIBRATION_HIGH_ORDER_TEMPERATURE_COMPENSATION_BUILD_ENABLE "0"
    CACHE STRING "Qualification-only passive high-order temperature correction")
set(IMU_CALIBRATION_G_SENSITIVITY_BUILD_ENABLE "0" CACHE STRING
    "Qualification-only gyro g-sensitivity correction")
if(NOT IMU_CALIBRATION_PRODUCTION_BUILD STREQUAL "1" OR
   NOT IMU_CALIBRATION_HIGH_ORDER_TEMPERATURE_COMPENSATION_BUILD_ENABLE
       STREQUAL "0" OR
   NOT IMU_CALIBRATION_G_SENSITIVITY_BUILD_ENABLE STREQUAL "0")
    message(FATAL_ERROR
        "ARM firmware requires production=1, high-order=0 and g-sensitivity=0")
endif()

# The IMU DMA staging addresses must remain reserved even after CubeMX
# regenerates its default linker script. The toolchain must use our layout.
if(NOT CMAKE_EXE_LINKER_FLAGS MATCHES "dual_imu_h743\\.ld")
    message(FATAL_ERROR
        "The project-owned dual_imu_h743.ld linker script is not active")
endif()
set_property(TARGET ${CMAKE_PROJECT_NAME} APPEND PROPERTY
    LINK_DEPENDS "${DUAL_IMU_LINKER_SCRIPT}")

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

set(APPLICATION_SOURCES
    App/app.c
    App/imu/dual_imu.c
    App/imu/bmi088.c
    App/imu/icm45686.c
    App/imu/imu_calibration.c
    App/imu/imu_causal_accel_history.c
    App/imu/imu_clock_sync.c
    App/imu/imu_geometry.c
    App/imu/imu_motion_guard.c
    App/imu/imu_spi_dma.c
    App/imu/imu_stream_buffer.c
    App/imu/imu_time.c
    App/fusion/attitude_mekf.c
    App/fusion/cross_lane_calibrator.c
    App/fusion/dual_imu_estimator.c
    App/fusion/fast_attitude_predictor.c
    App/fusion/imu_preintegrator.c
    App/fusion/imu_selector.c
    App/protocol/hi91_frame.c
    App/protocol/dual_imu_usb_stream.c
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${APPLICATION_SOURCES}
    ThirdParty/BMI08x/bmi08a.c
    ThirdParty/BMI08x/bmi08g.c
    ThirdParty/BMI08x/bmi08xa.c
    ThirdParty/ICM45686/imu/inv_imu_transport.c
    ThirdParty/ICM45686/imu/inv_imu_driver.c
    ThirdParty/ICM45686/imu/inv_imu_driver_advanced.c
)

set_source_files_properties(${APPLICATION_SOURCES} PROPERTIES
    COMPILE_OPTIONS "-Wextra;-Werror;-Wpedantic;-Wshadow"
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/App
    ${CMAKE_CURRENT_SOURCE_DIR}/App/imu
    ${CMAKE_CURRENT_SOURCE_DIR}/App/fusion
    ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/BMI08x
    ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/ICM45686
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    IMU_CALIBRATION_PRODUCTION_BUILD=1
    IMU_CALIBRATION_HIGH_ORDER_TEMPERATURE_COMPENSATION_BUILD_ENABLE=0
    IMU_CALIBRATION_G_SENSITIVITY_BUILD_ENABLE=0
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin
    COMMENT "Generating HEX and BIN firmware images"
)
