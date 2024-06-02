add_executable(Rasterizer2 main.cpp rasterizer.hpp rasterizer.cpp global.hpp Triangle.hpp Triangle.cpp)
# target_link_libraries(Rasterizer2 ${OpenCV_LIBRARIES})
target_link_libraries(Rasterizer2 ${OpenCV_LIBS})
