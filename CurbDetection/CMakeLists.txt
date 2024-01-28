cmake_minimum_required(VERSION 3.24)
project(CurbDetection)

set(CMAKE_CXX_STANDARD 17)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(CurbDetection main.cpp
        VideoPlayer.h
        curbDetect.h
        curbDetection.cpp
        define.h
)
link_directories("/usr/local/lib/")
target_link_libraries( CurbDetection ${OpenCV_LIBS})