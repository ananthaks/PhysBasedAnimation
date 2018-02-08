cmake_minimum_required(VERSION 3.9)
project(PhyBasedAnim)

set(CMAKE_CXX_STANDARD 11)


include_directories(Eigen)

add_executable(PhyBasedAnim src/main.cpp src/components/Spring.cpp src/components/Spring.h src/utility/FileHelper.cpp src/utility/FileHelper.h)