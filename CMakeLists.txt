cmake_minimum_required(VERSION 2.8)
project(myslam)

set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_FLAGS "-std=c++14 -O2 ${SSE_FLAGS} -msse4" )
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++14 -O3 -fopenmp -pthread")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories("/usr/include/eigen3")

find_package(OpenCV 3.4.6 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)
find_package(G2O REQUIRED)
find_package(Glog REQUIRED)
find_package(GFlags REQUIRED)
find_package(CSparse REQUIRED)

# gtest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

include_directories(
	${OpenCV_INCLUDE_DIRS}
	${Pangolin_INCLUDE_DIRS}
	${Sophus_INCLUDE_DIRS}
	${G2O_INCLUDE_DIRS}
	${GLOG_INCLUDE_DIRS}
	${GTEST_INCLUDE_DIRS}
	${GFLAGS_INCLUDE_DIRS}
	${CSPARSE_INCLUDE_DIR})


set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        ${Pangolin_LIBRARIES} GL GLU GLEW glut
        g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension
        ${GTEST_BOTH_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${GFLAGS_LIBRARIES}
        pthread
        ${CSPARSE_LIBRARY}
        )
enable_testing()

include_directories(${PROJECT_SOURCE_DIR}/include/myslam)
add_subdirectory(src)
add_subdirectory(runexemple)






