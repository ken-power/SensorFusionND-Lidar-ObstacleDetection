# I need to use these arguments on Mac:
# cmake -DCMAKE_C_COMPILER=/usr/bin/gcc -DCMAKE_CXX_COMPILER=/usr/bin/g++ ..

cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

add_definitions(-std=c++14)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")

if(APPLE)
    message("Making on APPLE")
    set(CMAKE_OSX_DEPLOYMENT_TARGET 10.15)
    set(CMAKE_OSX_SYSROOT "")  
endif()


project(playback)

find_package(PCL 1.3 REQUIRED COMPONENT common io VISUALIZATION)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_executable (environment src/environment.cpp src/render/render.cpp src/processPointClouds.cpp)
target_link_libraries (environment ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})


