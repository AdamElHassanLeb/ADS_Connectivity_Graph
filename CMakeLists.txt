cmake_minimum_required(VERSION 3.28)
project(ADS_Midterm)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Add the source files to the project
add_executable(ADS_Midterm
        main.cpp
        Graph.cpp
        Graph.h
        config.h
)
