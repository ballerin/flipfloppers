cmake_minimum_required(VERSION 2.8)
project( proj )
find_package( OpenCV REQUIRED )
add_executable( proj prog_nuovo.cpp )
target_link_libraries( proj ${OpenCV_LIBS} )
