# cmake/CompilerWarnings.cmake

# This will define the `set_project_warnings` function
# that takes the interface name.
function(set_project_warnings project_name)
  target_compile_features(${project_name} INTERFACE cxx_std_17)

  set(gcc_like_cxx "$<COMPILE_LANG_AND_ID:CXX,ARMClang,AppleClang,Clang,Gnu,LCC>")
  set(msvc_cxx "$<COMPILE_LANG_AND_ID:CXX,MSVC>")

  target_compile_options(${project_name} INTERFACE
  "$<${gcc_like_cxx}:$<BUILD_INTERFACE:-Wall;-Wextra;-Wshadow;-Wformat=2;-Wunused>>"
  "$<${msvc_cxx}:$<BUILD_INTERFACE:-W3>>"
  )
endfunction()