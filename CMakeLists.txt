cmake_minimum_required(VERSION 3.20)
project (iksimulation)
set(CMAKE_CXX_STANDARD 17)

#set(CMAKE_PREFIX_PATH "${CMAKE_CURRENT_LIST_DIR}/vcpkg_installed/arm64-osx")

find_package(Eigen3 REQUIRED)
find_package(SDL2 CONFIG REQUIRED)
find_package(OpenGL REQUIRED)


add_executable(iksimulation main.cpp)
target_include_directories(iksimulation PRIVATE Eigen3::Eigen SDL2::SDL2 OpenGL::GL)
target_link_libraries(iksimulation Eigen3::Eigen SDL2::SDL2 OpenGL::GL)
