project(odas)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)
find_package(OpenMP)
find_package(PkgConfig)
pkg_check_modules(JSON jsoncpp REQUIRED)
if (OPENMP_FOUND)
    set (CMAKE_CXX_FLAGS "-Ofast -std=c++17 -I/usr/include/jsoncpp ${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
SET(NLOPT_LIBRARIES "/usr/local/lib/libnlopt.so")

add_library(${PROJECT_NAME} SHARED ${SRC_LIST} )
target_link_libraries (${PROJECT_NAME} ${PCL_LIBRARIES})
target_link_libraries (${PROJECT_NAME} ${NLOPT_LIBRARIES})

