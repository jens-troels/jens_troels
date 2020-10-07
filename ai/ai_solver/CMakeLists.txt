cmake_minimum_required(VERSION 3.1)
project(ai_solver)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)

add_executable(ai_solver
        main.cpp
        Point.cpp
        Point.h
        hungarian.cpp
        hungarian.hpp
        SokobanSolver.cpp
        SokobanSolver.h
        State.cpp
        State.h
        Man.cpp
        Man.h)
