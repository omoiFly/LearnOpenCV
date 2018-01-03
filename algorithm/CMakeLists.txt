cmake_minimum_required(VERSION 3.9)
project(LearnOpenCV)

find_package(OpenCV REQUIRED)
set(CMAKE_CXX_STANDARD 11)

add_executable(LearnOpenCV main.cpp)
target_link_libraries(LearnOpenCV ${OpenCV_LIBS})
