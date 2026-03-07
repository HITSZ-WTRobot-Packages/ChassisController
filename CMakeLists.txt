cmake_minimum_required(VERSION 3.16)

# set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(Core)
add_subdirectory(Chassis)
add_subdirectory(Controller)
add_subdirectory(Localization)