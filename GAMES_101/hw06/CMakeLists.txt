add_executable(
  RayTracing2
  main.cpp
  Object.hpp
  Vector.cpp
  Vector.hpp
  Sphere.hpp
  global.hpp
  Triangle.hpp
  Scene.cpp
  Scene.hpp
  Light.hpp
  AreaLight.hpp
  BVH.cpp
  BVH.hpp
  Bounds3.hpp
  Ray.hpp
  Material.hpp
  Intersection.hpp
  Renderer.cpp
  Renderer.hpp)

set(CMAKE_BUILD_TYPE Debug)
target_compile_options(RayTracing2 PUBLIC "-O0")
