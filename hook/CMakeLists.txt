PROJECT(hook)

cmake_minimum_required(VERSION 2.8)

find_package(Boost REQUIRED)

find_package(Eigen3)
IF(NOT EIGEN3_INCLUDE_DIR)
  set(EIGEN3_INCLUDE_DIR $ENV{EIGEN3_ROOT_DIR})
ENDIF() 

SET(hk_HDRS
    RefScanMakerBS.h
    RefScanMakerLM.h
    CostFunctionED.h
    CostFunctionPD.h
    PoseOptimizerSD.h
    PoseOptimizerSL.h
    DataAssociatorLS.h
    DataAssociatorGT.h
    PointCloudMapBS.h
    PointCloudMapGT.h
    PointCloudMapLP.h
    LoopDetectorSS.h
)

SET(hk_SRCS 
    RefScanMakerBS.cpp
    RefScanMakerLM.cpp
    CostFunctionED.cpp
    CostFunctionPD.cpp
    PoseOptimizerSD.cpp
    PoseOptimizerSL.cpp
    DataAssociatorLS.cpp
    DataAssociatorGT.cpp
    PointCloudMapBS.cpp
    PointCloudMapGT.cpp
    PointCloudMapLP.cpp
    LoopDetectorSS.cpp
)

include_directories(
    ${Boost_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../framework
)

link_directories(
)

ADD_LIBRARY(hook ${hk_SRCS} ${hk_HDRS})

target_link_libraries(hook framework)

