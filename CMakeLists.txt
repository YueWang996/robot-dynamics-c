cmake_minimum_required(VERSION 3.16)
project(RobotDynamicsProject C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# ============================================================================
# Configuration Options (control submodule behavior)
# ============================================================================

option(RD_SINGLE_PRECISION "Use single precision (float)" ON)
option(RD_CMSIS_DSP "Enable ARM CMSIS-DSP" OFF)
option(RD_STATIC_ALLOC "Use static memory allocation" OFF)
option(RD_OPTIMIZE_SIZE "Optimize for size (-Os)" OFF)
option(RD_ENABLE_DEBUG "Enable debug logging" OFF)

# ============================================================================
# Add RobotDynamics Library Submodule
# ============================================================================

add_subdirectory(RobotDynamics)

# ============================================================================
# Test Executable
# ============================================================================

add_executable(rd_test test_main.c)
target_link_libraries(rd_test PRIVATE robot_dynamics)
target_compile_options(rd_test PRIVATE -Wall -Wextra)

# ============================================================================
# Installation
# ============================================================================

install(TARGETS robot_dynamics rd_test
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(FILES
    RobotDynamics/robot_dynamics.h
    RobotDynamics/rd_config.h
    RobotDynamics/rd_math.h
    RobotDynamics/rd_model.h
    RobotDynamics/rd_chain.h
    RobotDynamics/rd_state.h
    RobotDynamics/rd_algorithms.h
    RobotDynamics/spine_model.h
    DESTINATION include/robot_dynamics
)
