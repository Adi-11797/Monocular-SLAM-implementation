add_library(mono-slam)

add_subdirectory(backend)
add_subdirectory(controller)
add_subdirectory(data)
add_subdirectory(frontend)
add_subdirectory(viewer)
add_subdirectory(test)

target_link_libraries(
    mono-slam 
    PUBLIC
    ${OpenCV_LIBS}
    project_options
    gtsam
    # pangolin
    ${Pangolin_LIBRARY}
    tbb
    utils
    )
target_include_directories(mono-slam PUBLIC ${Pangolin_LIBRARY})

# target_include_directories(mono-slam PUBLIC ${OpenCV_INCLUDE_DIRS})
add_executable(monocular MACOSX_BUNDLE mono-slam.cpp)

target_link_libraries(
    monocular PRIVATE mono-slam utils CLI11::CLI11
)