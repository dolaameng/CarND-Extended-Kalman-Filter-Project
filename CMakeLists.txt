project(ExtendedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x)


set(tests
	src/main_test.cpp)
add_executable(run_tests ${tests})

set(sources
    src/main.cpp)
add_executable(ExtendedKF ${sources})

