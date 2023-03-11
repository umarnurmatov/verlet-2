include(FetchContent)


FetchContent_Declare(
  SFML
  GIT_REPOSITORY "https://github.com/SFML/SFML.git"
  GIT_TAG master
  )
add_subdirectory(sfml)

FetchContent_Declare(
  imgui
  GIT_REPOSITORY "https://github.com/ocornut/imgui.git"
  GIT_TAG v1.89.2
)
FetchContent_MakeAvailable(imgui)

  
FetchContent_Declare(
  imgui-sfml
  GIT_REPOSITORY "https://github.com/eliasdaler/imgui-sfml.git"
  GIT_TAG master
)
add_subdirectory(imgui-sfml)





