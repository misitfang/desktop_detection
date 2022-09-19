cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(test)

find_package(PCL 1.5 REQUIRED)
FIND_PACKAGE(OpenMP REQUIRED)
if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (test test.cpp)
target_link_libraries (test ${PCL_LIBRARIES})

add_executable (boundary_extract boundary_extract.cpp)
target_link_libraries (boundary_extract ${PCL_LIBRARIES})

add_executable (alpha_boundary_extract alpha_boundary_extract.cpp)

target_link_libraries (alpha_boundary_extract ${PCL_LIBRARIES})
