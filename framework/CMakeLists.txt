PROJECT(framework)

cmake_minimum_required(VERSION 2.8)

find_package(Boost REQUIRED)

find_package(Eigen3)
IF(NOT EIGEN3_INCLUDE_DIR)
  set(EIGEN3_INCLUDE_DIR $ENV{EIGEN3_ROOT_DIR})
ENDIF() 

SET(fw_HDRS
    MyUtil.h
    LPoint2D.h
    Pose2D.h
    Scan2D.h
    PointCloudMap.h
    RefScanMaker.h
    ScanPointResampler.h
    ScanPointAnalyser.h
    PoseEstimatorICP.h
    PoseOptimizer.h
    CostFunction.h
    PoseGraph.h
    P2oDriver2D.h
    ScanMatcher2D.h
    PoseFuser.h
    CovarianceCalculator.h
    DataAssociator.h
    NNGridTable.h
    SensorDataReader.h
    SlamFrontEnd.h
    SlamBackEnd.h
    LoopDetector.h
)

SET(fw_SRCS 
    MyUtil.cpp
    Pose2D.cpp
    Scan2D.cpp
    ScanPointResampler.cpp
    ScanPointAnalyser.cpp
    PoseEstimatorICP.cpp
    PoseGraph.cpp
    P2oDriver2D.cpp
    ScanMatcher2D.cpp
    PoseFuser.cpp
    CovarianceCalculator.cpp
    NNGridTable.cpp
    SensorDataReader.cpp
    SlamFrontEnd.cpp
    SlamBackEnd.cpp
    LoopDetector.cpp
)

include_directories(
    ${Boost_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../p2o
)

link_directories(
)

ADD_LIBRARY(framework ${fw_SRCS} ${fw_HDRS})

