cmake_minimum_required(VERSION 2.8)
project( prova )
find_package( OpenCV REQUIRED )
add_executable( prova prova.cpp )
target_link_libraries(prova ${OpenCV_LIBS} )

