cmake_minimum_required(VERSION 3.9)
project(PhyBasedAnim)

set(CMAKE_CXX_STANDARD 11)

find_package(Git)
# Enable DownloadProject CMake module

include(DownloadProject.cmake)

download_project(
        PROJ partio
        GIT_REPOSITORY https://github.com/wdas/partio.git
        GIT_TAG v1.1.0
        ${UPDATE_DISCONNECTED_IF_AVAILABLE}

)

# This seems to be causing some error. So removec it for now. This will probably make sure that only the requested patch is download
#PATCH_COMMAND ${GIT_EXECUTABLE} apply ${CMAKE_CURRENT_SOURCE_DIR}/partio.patch

add_subdirectory(${partio_SOURCE_DIR} ${partio_BINARY_DIR})
set_target_properties(partio PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${partio_SOURCE_DIR}/src/lib"
        INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${partio_SOURCE_DIR}/src/lib"
        INTERFACE_LINK_LIBRARIES "z"
        )

include_directories(Eigen)

add_executable(PhyBasedAnim src/main.cpp src/components/Spring.cpp src/components/Spring.h src/utility/FileHelper.cpp src/utility/FileHelper.h)

target_link_libraries(PhyBasedAnim partio)