cmake_minimum_required(VERSION 3.9)
project(PhyBasedAnim)

set(CMAKE_CXX_STANDARD 11)


include_directories(Eigen)

add_executable(PhyBasedAnim src/main.cpp src/Components/Spring.cpp src/Components/Spring.h)