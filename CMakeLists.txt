PROJECT(DlpDriver CXX)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++11 -stdlib=libc++ -Wall")
cmake_minimum_required(VERSION 2.6)

add_library(DlpDriver SHARED dlpdriver.cpp)

ADD_EXECUTABLE(runtests tests/main.cpp)

TARGET_LINK_LIBRARIES(runtests DlpDriver)
