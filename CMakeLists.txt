###############################################################################
# CMakeLists.txt
#
# Defines build structure of the project and its main executable
###############################################################################

cmake_minimum_required(VERSION 3.10.0)
project(shm_server LANGUAGES CXX)

###############################################################################
# Project options
###############################################################################

option(SHM_SERVER_BUILD_TESTS "Whether to build tests or not" ON)

###############################################################################
# Project dependencies
###############################################################################

find_package(catkin REQUIRED
    cmake_modules
    COMPONENTS
        shm_comm
        roscpp
)

add_subdirectory(extern)

###############################################################################
# Package configuration
###############################################################################

catkin_package(
    DEPENDS
        shm_comm
        roscpp
)

###############################################################################
# Executable definition
###############################################################################

add_executable(shm_server
    src/main.cpp
)

target_link_libraries(shm_server
    PRIVATE
        ${catkin_LIBRARIES}
        json
)

target_include_directories(shm_server
    PRIVATE
        include
        ${catkin_INCLUDE_DIRS}
)


set_target_properties(shm_server
    PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
)

target_compile_options(shm_server
    PRIVATE
        -Wall -Wextra -pedantic
)

###############################################################################
# Tests definitions
###############################################################################

if(SHM_SERVER_BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()
