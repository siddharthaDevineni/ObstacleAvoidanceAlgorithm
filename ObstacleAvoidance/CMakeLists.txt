cmake_minimum_required(VERSION 3.10)
project (ObstacleAvoidance VERSION 2.1) # VERSION is not configured

configure_file(ObstacleAvoidanceConfig.h.in ObstacleAvoidanceConfig.h)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(src)



add_executable(${PROJECT_NAME} main.cpp )



target_include_directories(${PROJECT_NAME} PRIVATE src/include
                                           ${PROJECT_BINARY_DIR}) # It is looking in the pre compile directory
target_link_directories(${PROJECT_NAME} PRIVATE src) # It is looking for the libraries in built folder

target_link_libraries(${PROJECT_NAME} PUBLIC forceAttRep obstacleLib)


# Needed for Matplotlib

include_directories(externals/matplotlib2)
find_package(Python3 COMPONENTS Development NumPy)
target_include_directories(${PROJECT_NAME} PRIVATE ${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE Python3::Python Python3::NumPy)

