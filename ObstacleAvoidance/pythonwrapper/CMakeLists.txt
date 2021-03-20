FIND_PACKAGE(SWIG REQUIRED)
INCLUDE(${SWIG_USE_FILE})

FIND_PACKAGE(PythonLibs)
INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})


INCLUDE_DIRECTORIES(../src/include)

SET(CMAKE_SWIG_FLAGS "")

SET_SOURCE_FILES_PROPERTIES(obstacleAvoidance_py.i PROPERTIES CPLUSPLUS ON)
SET_SOURCE_FILES_PROPERTIES(obstacleAvoidance_py.i PROPERTIES SWIG_FLAGS "-includeall")



SWIG_ADD_LIBRARY(obstacleAvoidance_py SHARED python obstacleAvoidance_py.i functionsRepandAtt.cpp obstacleAvoidance.cpp)

SWIG_LINK_LIBRARIES(obstacleAvoidance_py ${PYTHON_LIBRARIES} forceAttRep obstacleLib)
