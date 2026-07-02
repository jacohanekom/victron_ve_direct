cmake_minimum_required(VERSION 3.16)
project(victron_ve_direct CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(AVAHI REQUIRED avahi-client)

add_executable(victron_ve_direct victron_ve_direct.cpp)

target_include_directories(victron_ve_direct PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${AVAHI_INCLUDE_DIRS})
target_link_libraries(victron_ve_direct PRIVATE ${AVAHI_LIBRARIES})
target_compile_options(victron_ve_direct PRIVATE -Wall -Wextra -O2)

install(TARGETS victron_ve_direct DESTINATION bin)
install(FILES config.ini DESTINATION share/victron-ve-direct)
