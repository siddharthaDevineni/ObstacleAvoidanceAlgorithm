cmake_minimum_required(VERSION 3.10)
project (ObstacleAvoidance)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_subdirectory(src)


add_executable(${PROJECT_NAME} main.cpp )



target_include_directories(${PROJECT_NAME} PRIVATE src/include) # It is looking in the pre compile directory
target_link_directories(${PROJECT_NAME} PRIVATE src) # It is looking for the libraries in built folder

target_link_libraries(${PROJECT_NAME} forceAttRep obstacleLib)





# set(CMAKE_C_FLAGS_DEBUG "-g -DDEBUG")
# set(CMAKE_CXX_FLAGS_DEBUG "-g -DDEBUG")
# SET(CMAKE_BUILD_TYPE Debug)