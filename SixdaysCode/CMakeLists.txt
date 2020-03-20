cmake_minimum_required(VERSION 3.15)
project(SixdaysCode)

set(CMAKE_CXX_STANDARD 17)

include_directories(Estimators)
include_directories(Estimators/Callbacks)
include_directories(Estimators/Models)
include_directories(Estimators/PseudoMeasurementsHelperClasses)
include_directories(Estimators/StateBoxes)
include_directories(OSG_VIZ)
include_directories(Plugins)
include_directories(Setups)
include_directories(Sixdays)

add_executable(SixdaysCode
        Estimators/Models/CalibrationModel.cpp
        Estimators/Models/XsensModelWithBias.cpp
        Estimators/PseudoMeasurementsHelperClasses/PenaltyMap.cpp
        Estimators/source/CalibrationBatch.cpp
        Estimators/source/XsensBatchWithBias.cpp
        OSG_VIZ/osg_viz.cpp
        OSG_VIZ/OSG_VIZ_Transform.cpp
        Plugins/FileIMUPlugin.cpp
        Setups/CalibrationSetup.cpp
        Setups/ExtendedLambdaSetup.cpp
        Setups/LoadResult.cpp
        Sixdays/LaserBarrier.cpp
        main.cpp)

find_package(glog REQUIRED)
find_package(libconfig++ REQUIRED)


FIND_PACKAGE(OpenThreads REQUIRED)
FIND_PACKAGE(osg REQUIRED)
FIND_PACKAGE(osgDB REQUIRED)
FIND_PACKAGE(osgUtil REQUIRED)
FIND_PACKAGE(osgGA REQUIRED)
FIND_PACKAGE(osgViewer REQUIRED)
find_package(osgTerrain REQUIRED)
find_package(Ceres REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)
#find_package(PCL REQUIRED)

target_link_libraries (SixdaysCode LINK_PUBLIC utils  ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY}  ${CERES_LIBRARIES} ${OSG_LIBRARIES} ${OSGTERRAIN_LIBRARIES} ${OSGVIEWER_LIBRARIES} ${OSGUTIL_LIBRARIES} ${OSGDB_LIBRARIES} ${OSGGA_LIBRARIES} ${OPENTHREADS_LIBRARIES}  ${GLOG_LIBRARIES} ${LIBCONFIG++_LIBRARIES})