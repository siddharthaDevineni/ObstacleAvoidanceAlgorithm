cmake_minimum_required(VERSION 3.10)
project (ObstacleAvoidance)


add_subdirectory(src)


add_executable(${PROJECT_NAME} main.cpp )



target_include_directories(${PROJECT_NAME} PRIVATE src/include) # It is looking in the pre compile directory
target_link_directories(${PROJECT_NAME} PRIVATE src) # It is looking for the libraries in built folder

target_link_libraries(${PROJECT_NAME} forceAttRep obstacleLib)


# Needed for Matplotlib

include_directories(externals/matplotlib-cpp)
find_package(Python2 COMPONENTS Development NumPy)
target_include_directories(${PROJECT_NAME} PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} Python2::Python Python2::NumPy)
