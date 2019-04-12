cmake_minimum_required(VERSION 2.6)
project(mvg_agorithm)

add_compile_options(-std=c++11)
SET(CMAKE_BUILD_TYPE Release)
find_package(OpenCV 4 REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
)

add_library(${PROJECT_NAME}_lib SHARED
    src/homography_matrix.cpp
    src/fundamental_matrix.cpp
    src/essential_matrix.cpp
    src/line_RANSAC.cpp
)

target_link_libraries(${PROJECT_NAME}_lib
    ${EIGEN3_LIBS}
    ${OpenCV_LIBS}
)


# test line ransac
add_executable(test_line_ransac
	test/test_line_ransac.cpp)
target_link_libraries(test_line_ransac
	${PROJECT_NAME}_lib
)

