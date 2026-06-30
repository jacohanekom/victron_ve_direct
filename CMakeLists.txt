cmake_minimum_required(VERSION 3.16)
project(victron_ve_direct CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(victron_ve_direct victron_ve_direct.cpp)

target_include_directories(victron_ve_direct PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(victron_ve_direct PRIVATE -Wall -Wextra -O2)

install(TARGETS victron_ve_direct DESTINATION bin)
install(FILES config.ini DESTINATION share/victron-ve-direct)
