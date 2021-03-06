include_directories(include)
include_directories(../externals/matplotlib2)


add_library(forceAttRep functionsRepandAtt.cpp include/functionsRepandAtt.h )
add_library(obstacleLib obstacleAvoidance.cpp include/obstacleAvoidance.h)
