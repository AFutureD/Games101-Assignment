cmake_minimum_required (VERSION 2.8.11)
project (Transformation)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package(Eigen3 REQUIRED)

# include_directories(EIGEN3_INCLUDE_DIR)
# include_directories(${Eigen3_DIR})
# message("${Eigen3_DIR}")
# message("${EIGEN3_INCLUDE_DIR}")

add_executable (Transformation main.cpp)
target_link_libraries(Transformation PRIVATE Eigen3::Eigen)