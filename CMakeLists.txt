cmake_minimum_required(VERSION 3.25)

project(
    verlet-2
    DESCRIPTION "SFML Verltet-integration physics engine"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(cmake/ccache.cmake)

add_subdirectory(modules)
add_subdirectory(src)
