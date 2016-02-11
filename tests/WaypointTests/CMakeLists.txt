cmake_minimum_required(VERSION 2.8)
set(bin_name "TGL_WaypointTest")

project(${bin_name})

add_executable(${bin_name} main.cpp)
target_link_libraries(${bin_name} tgl)
