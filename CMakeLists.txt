cmake_minimum_required(VERSION 3.26)
project(D1CY)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)
include_directories(D1CY_Control)

add_executable(D1CY
        D1CY_Control/D1CY_Control.ino
        D1CY_initial.ino)
