cmake_minimum_required(VERSION 2.9)
project(VISP_EXAMPLES)

set(CMAKE_MODULE_PATH ${VISP_EXAMPLES_SOURCE_DIR}/CMakeModules ${CMAKE_MODULE_PATH})

find_package(VISP REQUIRED)
find_package(OpenNI2 REQUIRED)
include_directories(${VISP_INCLUDE_DIRS} ${OPENNI2_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES main.cpp Tracker.cpp Vector.cpp)
add_executable(VISP_EXAMPLES ${SOURCE_FILES})

target_link_libraries(VISP_EXAMPLES ${VISP_LIBRARIES} ${OPENNI2_LIBRARIES})