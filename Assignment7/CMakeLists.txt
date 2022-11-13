cmake_minimum_required(VERSION 3.10)
project(RayTracing)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE -Ofast)
 
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -O3  -Wall")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3  -Wall")

set(CMAKE_CXX_STANDARD 17)

add_executable(RayTracing main.cpp Object.hpp Vector.cpp Vector.hpp Sphere.hpp global.hpp Triangle.hpp Scene.cpp
        Scene.hpp Light.hpp AreaLight.hpp BVH.cpp BVH.hpp Bounds3.hpp Ray.hpp Material.hpp Intersection.hpp
        Renderer.cpp Renderer.hpp)
