cmake_minimum_required(VERSION 3.20.0)
project(dynamic_game_trajectory_planner VERSION 0.1.0 LANGUAGES C CXX)

# ---------------------------------------------------------------------------
# Build options.
#
# The six cache variables below select what is compiled: the arithmetic format,
# the trigonometric path and the scenario. They can be set on the command line
# with -D, but the intended way is to edit them interactively:
#
#     mkdir build && cd build
#     cmake ..
#     ccmake .        edit the options, press c to configure, g to generate
#     cmake --build .
#
# ccmake lists each option with the help text given here, and cycles the
# allowed values of REAL_BITS, POSIT_ES, TRIG_PATH and SCENARIO with Enter.
# It comes from the cmake-curses-gui package on Debian and Ubuntu.
#
# sweep.sh passes these same options with -D, so both routes build the same
# configurations.
# ---------------------------------------------------------------------------

# Set default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Choose precision at configure time: 16, 32, or 64
set(REAL_BITS "64" CACHE STRING "Width of the working arithmetic format: 16, 32 or 64 bits")
set_property(CACHE REAL_BITS PROPERTY STRINGS 16 32 64)

option(ENABLE_POSIT "ON for posit arithmetic, OFF for the IEEE 754 type of the same width" OFF)
option(ENABLE_QUANTIZATION "Nondimensionalize the state before integrating" TRUE)

set(POSIT_ES_ALLOWED 0 1 2 3 4)
set(POSIT_ES "2" CACHE STRING "Posit exponent size es, giving posit<REAL_BITS,es>. Used only when ENABLE_POSIT is ON")
set_property(CACHE POSIT_ES PROPERTY STRINGS ${POSIT_ES_ALLOWED})

if(ENABLE_POSIT)
    if(NOT POSIT_ES IN_LIST POSIT_ES_ALLOWED)
        message(FATAL_ERROR "POSIT_ES must be one of: ${POSIT_ES_ALLOWED} (got '${POSIT_ES}')")
    endif()
    add_compile_definitions(USE_POSIT=1 POSIT_ES=${POSIT_ES})
    message(STATUS "Datatype        : posit<${REAL_BITS}, ${POSIT_ES}>")
else()
    add_compile_definitions(USE_POSIT=0)
    if(REAL_BITS STREQUAL "16")
        message(STATUS "Datatype        : half (IEEE binary16)")
    elseif(REAL_BITS STREQUAL "32")
        message(STATUS "Datatype        : float (fp32)")
    else()
        message(STATUS "Datatype        : double (fp64)")
    endif()
endif()

# Trigonometric path (see include/trig.h). Either the functions are evaluated
# natively in the working format from an fdlibm minimax polynomial, or the
# argument is converted to fp64, evaluated with the C library, and rounded back.
# The polynomial path is the default, because it keeps every operation in the
# format under study and is the path the reported results are produced on.
set(TRIG_PATH_ALLOWED upscale poly)
set(TRIG_PATH "poly" CACHE STRING "How sin/cos/tan are evaluated: poly (native polynomial) or upscale (via fp64)")
set_property(CACHE TRIG_PATH PROPERTY STRINGS ${TRIG_PATH_ALLOWED})

if(NOT TRIG_PATH IN_LIST TRIG_PATH_ALLOWED)
    message(FATAL_ERROR "TRIG_PATH must be one of: ${TRIG_PATH_ALLOWED} (got '${TRIG_PATH}')")
endif()

if(TRIG_PATH STREQUAL "poly")
    add_compile_definitions(USE_POLY=1)
    message(STATUS "Trig path       : native polynomial (fdlibm coefficients)")
else()
    add_compile_definitions(USE_POLY=0)
    message(STATUS "Trig path       : fp64 conversion + C library")
endif()

# Scenario (see include/scenarios.h). Selected at configure time so switching
# scenario needs no source edit.
set(SCENARIO_ALLOWED intersection crossing merge)
set(SCENARIO "intersection" CACHE STRING "Traffic scenario to plan: intersection, crossing or merge")
set_property(CACHE SCENARIO PROPERTY STRINGS ${SCENARIO_ALLOWED})

if(NOT SCENARIO IN_LIST SCENARIO_ALLOWED)
    message(FATAL_ERROR "SCENARIO must be one of: ${SCENARIO_ALLOWED} (got '${SCENARIO}')")
endif()

# The numeric id is passed rather than the enum name, because parameters.h
# reads it to pick the scenario's nondimensionalization units and does not
# include scenarios.h. The values match the ScenarioId enum.
if(SCENARIO STREQUAL "crossing")
    add_compile_definitions(SCENARIO=1)
elseif(SCENARIO STREQUAL "merge")
    add_compile_definitions(SCENARIO=2)
else()
    add_compile_definitions(SCENARIO=0)
endif()
message(STATUS "Scenario        : ${SCENARIO}")

if(ENABLE_QUANTIZATION)
    message(STATUS "Quantization is ENABLED")
    add_compile_definitions(ENABLE_QUANTIZATION=1) # Define macro for quantization
else()
    message(STATUS "Quantization is DISABLED")
    add_compile_definitions(ENABLE_QUANTIZATION=0) # Define macro to disable quantization
endif()

message(STATUS "REAL_BITS = ${REAL_BITS}")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_compile_definitions(REAL_BITS=${REAL_BITS})

include_directories(include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/universal/include/sw)

set(source_files
    src/main.cpp
    src/dynamic_game_planner.cpp
    src/utils.cpp
    src/vehicle_state.cpp
)

add_executable(dynamic_game_trajectory_planner ${source_files})

target_compile_options(dynamic_game_trajectory_planner PRIVATE
    -O3
    -fno-fast-math -fno-associative-math -fno-reciprocal-math -fno-finite-math-only
    -ffp-contract=off)

add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/dynamic_game_planner.s
    COMMAND ${CMAKE_CXX_COMPILER}
            -S -g -fverbose-asm -O3 -std=c++${CMAKE_CXX_STANDARD}
            -DREAL_BITS=${REAL_BITS}
            -DENABLE_QUANTIZATION=$<IF:$<BOOL:${ENABLE_QUANTIZATION}>,1,0>
            -DUSE_POSIT=$<IF:$<BOOL:${ENABLE_POSIT}>,1,0>
            -DPOSIT_ES=${POSIT_ES}
            -DUSE_POLY=$<IF:$<STREQUAL:${TRIG_PATH},poly>,1,0>
            -I${CMAKE_CURRENT_SOURCE_DIR}/include
            -I${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/universal/include/sw
            -o ${CMAKE_CURRENT_BINARY_DIR}/dynamic_game_planner.s
            ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamic_game_planner.cpp
    DEPENDS src/dynamic_game_planner.cpp
    COMMENT "Generating annotated assembly for dynamic_game_planner.cpp"
)

add_custom_target(generate_cpp_asm
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/dynamic_game_planner.s
    COMMENT "Target to generate assembly code for dynamic_game_planner.cpp"
)
