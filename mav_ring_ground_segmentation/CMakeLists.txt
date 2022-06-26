cmake_minimum_required(VERSION 2.8)
project(depth_extract)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE  "-std=c++11 -O3 -fopenmp -pthread")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

############### dependencies ######################
# Eigen
include_directories("/usr/local/include/eigen3")

find_package(OpenCV 3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
# OpenCV
#find_package(OpenCV 3.1 REQUIRED)
include_directories("/media/lzb/Windows-SSD/ubuntu18/Master_Thesis/src/mav_ring_ground_segmentation/src")
include_directories(${OpenCV_INCLUDE_DIRS}   ${PCL_INCLUDE_DIRS} 
)
set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${GTEST_BOTH_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${GFLAGS_LIBRARIES}
        pthread
        ${CSPARSE_LIBRARY}
        )

enable_testing()

############### source and test ######################


add_library(depth_extract_lib SHARED
src/mysystem.cpp
src/image_cell.cpp
src/ring_cell.cpp
src/mypoint.cpp
      )

target_link_libraries(depth_extract_lib
        ${THIRD_PARTY_LIBS})

          
add_executable(depth_extract src/main.cpp)
target_link_libraries(depth_extract depth_extract_lib ${THIRD_PARTY_LIBS} ${PCL_LIBRARIES})