add_library(utils)

target_sources(
  utils
  PUBLIC
  utils.h
  utils.cpp
)

target_include_directories(
  utils
  INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}
)



target_link_libraries(
  utils
  PUBLIC
  project_options
  gtsam
)