cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 3.4 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

project (kalman)
add_executable(kalman kalman.cpp)

# https://eigen.tuxfamily.org/dox/TopicCMakeGuide.html
target_link_libraries(kalman Eigen3::Eigen)
