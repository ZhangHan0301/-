cmake_minimum_required(VERSION 3.0)
project(segmentation)
add_definitions(-std=c++14)

set(CXX_FLAGS "-Wall" "-pedantic")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

find_package(PCL 1.10 REQUIRED)

find_package(OpenCV  REQUIRED)

# 添加头文件目录
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})



add_executable(segmentation segmentation.cpp)
target_link_libraries(segmentation ${PCL_LIBRARIES})
add_executable(contours findContours.cpp)
target_link_libraries(contours PRIVATE  ${PCL_LIBRARIES})
target_link_libraries(contours PRIVATE  ${OpenCV_LIBS})


#add_executable(mincutseg mincut.cpp)
#target_link_libraries(mincutseg ${PCL_LIBRARIES})
