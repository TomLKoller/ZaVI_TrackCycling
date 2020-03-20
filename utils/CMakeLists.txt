cmake_minimum_required(VERSION 3.0)
project(utils)

set(CMAKE_CXX_STANDARD 17)

include_directories(.)

find_package(glog REQUIRED)
find_package(libconfig REQUIRED)
#SET(BOOST_DIR "C:/Users/Public/Arbeit/boost_1_55_0" )
#SET(EIGEN_DIR "C:/Users/Public/Arbeit/eigen-3.3.7")
# Specify critical OpenSceneGraph build variables.
# Reference OpenSceneGraph includes and libraries.
#INCLUDE_DIRECTORIES(${BOOST_DIR})
#INCLUDE_DIRECTORIES(${EIGEN_DIR})
#INCLUDE_DIRECTORIES(${OSG_BUILD_DIR}/include)

add_library(utils SHARED
        OSG_UTILS.cpp
        ZaVI_Utils.cpp
        ZaviConfig.cpp
        threaded_objects.cpp
        CSV_Reader.cpp)

target_include_directories(utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
set_target_properties(utils PROPERTIES PUBLIC_HEADER *.h)


FIND_PACKAGE(OpenSceneGraph)
INCLUDE_DIRECTORIES(${OPENSCENEGRAPH_INCLUDE_DIRS})

TARGET_LINK_LIBRARIES(utils ${OPENSCENEGRAPH_LIBRARIES}    )