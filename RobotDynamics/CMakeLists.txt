cmake_minimum_required(VERSION 3.16)

# ============================================================================
# 1. Configuration Options
# ============================================================================
option(RD_SINGLE_PRECISION "Use single precision (float)" ON)
option(RD_CMSIS_DSP "Enable ARM CMSIS-DSP" OFF)
option(RD_STATIC_ALLOC "Use static memory allocation" OFF)
option(RD_OPTIMIZE_SIZE "Optimize for size (-Os)" OFF)
option(RD_ENABLE_DEBUG "Enable debug logging" OFF)

# ============================================================================
# 2. Library Target
# ============================================================================
add_library(robot_dynamics STATIC
    rd_algorithms.c
    rd_state.c
    rd_chain.c
    # rd_model.c # If you separated model loading logic, add it here
)

# Expose include path to the parent project
target_include_directories(robot_dynamics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# ============================================================================
# 3. Compile Definitions
# ============================================================================

# Precision
if(RD_SINGLE_PRECISION)
    target_compile_definitions(robot_dynamics PUBLIC RD_USE_SINGLE_PRECISION=1)
else()
    target_compile_definitions(robot_dynamics PUBLIC RD_USE_SINGLE_PRECISION=0)
endif()

# CMSIS-DSP Configuration
if(RD_CMSIS_DSP)
    target_compile_definitions(robot_dynamics PUBLIC RD_USE_CMSIS_DSP=1 ARM_MATH_CM4)
    if(NOT DEFINED RD_CMSIS_DSP_INCLUDE_DIR)
        message(FATAL_ERROR 
            "\n[RobotDynamics Error] RD_CMSIS_DSP is enabled, but RD_CMSIS_DSP_INCLUDE_DIR is not defined!\n"
            "Please set this variable in your top-level CMakeLists.txt before adding this subdirectory.\n"
            "Example: set(RD_CMSIS_DSP_INCLUDE_DIR \"\${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/DSP/Include\")\n"
        )
    endif()

    if(NOT EXISTS "${RD_CMSIS_DSP_INCLUDE_DIR}/arm_math.h")
        message(FATAL_ERROR 
            "\n[RobotDynamics Error] arm_math.h was NOT found in the provided directory:\n"
            "  ${RD_CMSIS_DSP_INCLUDE_DIR}\n"
            "Please check the path.\n"
        )
    endif()

    target_include_directories(robot_dynamics PRIVATE ${RD_CMSIS_DSP_INCLUDE_DIR})
    target_link_libraries(robot_dynamics PRIVATE stm32cubemx)

else()
    target_link_libraries(robot_dynamics PRIVATE m)
endif()

# Static Allocation
if(RD_STATIC_ALLOC)
    target_compile_definitions(robot_dynamics PUBLIC RD_USE_STATIC_ALLOC=1)
endif()

# Debugging
if(RD_ENABLE_DEBUG)
    target_compile_definitions(robot_dynamics PUBLIC RD_DEBUG=1)
endif()

# ============================================================================
# 4. Compiler Options
# ============================================================================
target_compile_options(robot_dynamics PRIVATE 
    -Wall -Wextra -Wpedantic
    -Wno-unused-parameter
    -Wno-missing-field-initializers
)

if(RD_OPTIMIZE_SIZE)
    target_compile_options(robot_dynamics PRIVATE -Os)
else()
    # -O3 -ffast-math is what -Ofast used to mean; spelled out because -Ofast
    # is deprecated in current clang. -ffast-math lets the compiler reassociate
    # the spatial-algebra expressions, which is where the win is, at the cost
    # of IEEE NaN/Inf semantics. Drop it if you need strict FP behaviour.
    target_compile_options(robot_dynamics PRIVATE -O3 -ffast-math)
endif()