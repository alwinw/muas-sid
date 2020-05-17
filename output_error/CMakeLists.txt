cmake_minimum_required(VERSION 3.0)
project(OutputError)

set(GCC_COVERAGE_COMPILE_FLAGS "-Wall -g")
 
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED)
 
include_directories(${Boost_INCLUDE_DIRS})

add_executable(output_error main.cpp OutputError_DragPolar.cpp)
target_link_libraries(output_error Eigen3::Eigen ${Boost_LIBRARIES})
