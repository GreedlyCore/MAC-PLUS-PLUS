cmake_minimum_required(VERSION 3.22)
project(MAC_PLUS_PLUS_release)

set(CMAKE_CXX_STANDARD 14)
add_definitions("-Wall -g")

find_package(PCL REQUIRED)
find_package(igraph 0.10.6 EXACT QUIET)
if(NOT igraph_FOUND)
    message(STATUS "igraph 0.10.6 not found, trying 0.9.9")
    find_package(igraph 0.9.9 EXACT REQUIRED)
    set(IGRAPH_VERSION_OLD TRUE)
endif()
find_package(OpenMP REQUIRED)

if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(MAC desc_dec.cpp funcs.cpp PCR.cpp registration.cpp visualization.cpp Eva.h)
add_executable(Boot main.cpp)

if(IGRAPH_VERSION_OLD)
    target_compile_definitions(MAC PRIVATE IGRAPH_VERSION_OLD)
endif()

target_link_libraries(MAC ${PCL_LIBRARIES} igraph::igraph)
