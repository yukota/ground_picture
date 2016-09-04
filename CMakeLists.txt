cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cppnetlib REQUIRED)
find_package(PNG REQUIRED)


set(PNGPP_INCLUDE_DIR /usr/include)

include_directories(${GAZEBO_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS} ${CPPNETLIB_INCLUDE_DIRS} ${PNG_INCLUDEDIR} ${PNGPP_INCLUDE_DIR})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
message(${CMAKE_CXX_FLAGS})

add_library(ground_picture SHARED ground_picture.cpp)


# cppnetlib is used as header only mode.
target_link_libraries(ground_picture ${GAZEBO_LIBRARIES} ${OpenCV_LIBS} ${CPPNETLIB_LIBRARIES}  ${PNG_LIBRARY})
