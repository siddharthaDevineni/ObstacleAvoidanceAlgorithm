include_directories(include)
include_directories(../externals/matplotlib2)


add_library(forceAttRep SHARED functionsRepandAtt.cpp include/functionsRepandAtt.h )
add_library(obstacleLib SHARED obstacleAvoidance.cpp include/obstacleAvoidance.h)
