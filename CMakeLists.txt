cmake_minimum_required(VERSION 3.25)

project(
    verlet-2
    DESCRIPTION "SFML Verltet-integration physics"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(cmake/ccache.cmake)

add_library(project_warnings INTERFACE)
include(cmake/compiler-warnings.cmake)
set_project_warnings(project_warnings)

add_subdirectory(lib)
add_subdirectory(src)
