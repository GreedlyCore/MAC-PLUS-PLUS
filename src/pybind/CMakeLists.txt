cmake_minimum_required(VERSION 3.22)
project(macpp_solver)

set(CMAKE_CXX_STANDARD 14)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(PCL REQUIRED)
find_package(igraph 0.10.6 EXACT QUIET)
if(NOT igraph_FOUND)
    message(STATUS "igraph 0.10.6 not found, trying 0.9.9")
    find_package(igraph 0.9.9 EXACT REQUIRED)
    set(IGRAPH_VERSION_OLD TRUE)
endif()
find_package(OpenMP REQUIRED)

find_package(pybind11 QUIET CONFIG)
if(NOT pybind11_FOUND)
    execute_process(
        COMMAND python3 -m pybind11 --cmakedir
        OUTPUT_VARIABLE pybind11_DIR
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE pybind11_CMD_RESULT
    )
    if(NOT pybind11_CMD_RESULT EQUAL 0)
        message(FATAL_ERROR
            "pybind11 not found. Install with: pip install pybind11")
    endif()
    find_package(pybind11 REQUIRED CONFIG PATHS ${pybind11_DIR})
endif()

set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

set(SRC "${CMAKE_CURRENT_SOURCE_DIR}/..")
set(MACPP_SOURCES
    ${SRC}/funcs.cpp
    ${SRC}/PCR.cpp
    ${SRC}/registration.cpp
    ${SRC}/desc_dec.cpp
    ${SRC}/visualization.cpp
    ${SRC}/macpp_bind.cpp
)

include_directories(${PCL_INCLUDE_DIRS} ${SRC})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

pybind11_add_module(macpp_solver ${MACPP_SOURCES})

target_compile_definitions(macpp_solver PRIVATE MACPP_PYBIND)
if(IGRAPH_VERSION_OLD)
    target_compile_definitions(macpp_solver PRIVATE IGRAPH_VERSION_OLD)
endif()

target_link_libraries(macpp_solver PRIVATE
    ${PCL_LIBRARIES}
    igraph::igraph
    OpenMP::OpenMP_CXX
)

set(PIPELINE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../eval")
set_target_properties(macpp_solver PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${PIPELINE_DIR}"
)

message(STATUS "macpp_solver.so will be placed in: ${PIPELINE_DIR}")
