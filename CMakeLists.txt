cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(ceres_learning)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
FIND_PACKAGE(Sophus REQUIRED)
find_package(OpenCV 3.0 REQUIRED)

include_directories(
        ${CERES_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${Sophus_INCLUDE_DIRS}
        ${OpenCV_INClUDE_DIRS}
)

# link_libraries(${CERES_LIBRARIES})
link_libraries(${OpenCV_LIBS} ${CERES_LIBRARIES} ${Sophus_LIBRARIES})
add_subdirectory("test")