message(STATUS "Fetching SFML...")
option(SFML_BUILD_AUDIO "Build audio" OFF)
option(SFML_BUILD_NETWORK "Build network" OFF)
FetchContent_MakeAvailable(SFML)