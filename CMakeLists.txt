cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

if (APPLE)
    set(CMAKE_OSX_DEPLOYMENT_TARGET "10.11")
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

project(rabv)

find_package(VTK)
find_package(PCL 1.8 REQUIRED)

include(${VTK_USE_FILE})
include_directories(${PCL_INCLUDE_DIRS} ${VTK_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${VTK_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (rabv main.cpp rabv.hpp)
target_link_libraries (rabv ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

add_executable (sample sample_main.cpp rabv.hpp)
target_link_libraries (sample ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
