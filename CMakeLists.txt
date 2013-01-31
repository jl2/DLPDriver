PROJECT(DlpDriver CXX)
set(CMAKE_CXX_FLAGS "-std=c++11")
cmake_minimum_required(VERSION 2.6)

add_library(DlpDriver SHARED dlpdriver.cpp)

ADD_EXECUTABLE(runtests tests/main.cpp)

TARGET_LINK_LIBRARIES(runtests DlpDriver)
