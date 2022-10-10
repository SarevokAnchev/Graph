cmake_minimum_required(VERSION 3.23)

project(Graph VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)

add_library(${PROJECT_NAME} src/Graph.cpp src/Node.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC include)

install (TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
