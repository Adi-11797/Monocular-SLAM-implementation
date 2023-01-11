cmake_minimum_required(VERSION 3.15+)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
 
# Find a name
project(mono-slam)
enable_testing()
add_library(project_options INTERFACE)
target_include_directories(project_options INTERFACE ${CMAKE_CURRENT_BINARY_DIR})

find_package(OpenCV REQUIRED PATHS "../project_libraries/opencv_install/")
find_package(GTSAM REQUIRED PATHS "../project_libraries/gtsam/build")
find_package(Eigen3 REQUIRED PATHS "../project_libraries/gtsam/gtsam/3rdparty/Eigen")
find_package(Pangolin REQUIRED PATHS "../project_libraries/Pangolin/build/")
# find_package(CLI11 CONFIG REQUIRED PATHS "../project_libraires/CLI11/build/")
# find_package(Pangolin REQUIRED)

find_package(TBB COMPONENTS tbb tbbmalloc)


include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories("../project_libraries/gtsam/gtsam/3rdparty/Eigen/")

add_subdirectory(utils)
add_subdirectory(src)

add_subdirectory(../project_libraries/CLI11 ${CMAKE_BINARY_DIR}/CLI11)
include_directories(${CLI11_INCLUDE_DIRS})

# add_subdirectory(../project_libraries/opencv/ ${CMAKE_BINARY_DIR}/opencv/)
# add_subdirectory(../project_libraries/gtsam/ ${CMAKE_BINARY_DIR}/gtsam/)

# include_directories("../project_libraries/opencv_install/include/")

