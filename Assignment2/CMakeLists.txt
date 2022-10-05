cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

# SET(CMAKE_BUILD_TYPE "Debug") 
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories(/usr/local/include)

add_executable(Rasterizer main.cpp rasterizer.hpp rasterizer.cpp global.hpp Triangle.hpp Triangle.cpp)
target_link_libraries(Rasterizer ${OpenCV_LIBRARIES})
