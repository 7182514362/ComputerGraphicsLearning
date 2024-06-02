cmake_minimum_required(VERSION 3.15)
project(vulkan-tutorial)

set(CMAKE_CXX_STANDARD_REQUIRED 17)

add_executable(demo main.cpp)

target_link_libraries(demo glfw vulkan dl pthread X11 Xxf86vm Xrandr Xi)
