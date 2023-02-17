set(NAME verlet-2-main)

file(GLOB_RECURSE source_files
    "src/*.cpp"
)
add_executable(${NAME} ${source_files})

target_include_directories(${NAME} PRIVATE 
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_link_libraries(${NAME} PRIVATE
    sfml-system sfml-graphics sfml-window
    ImGui-SFML::ImGui-SFML
)
target_compile_features(${NAME} PRIVATE cxx_std_20)
