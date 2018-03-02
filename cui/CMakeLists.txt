project(cui)

cmake_minimum_required(VERSION 2.8)

find_package(Boost REQUIRED)

find_package(Eigen3)
IF(NOT EIGEN3_INCLUDE_DIR)
  set(EIGEN3_INCLUDE_DIR $ENV{EIGEN3_ROOT_DIR})
ENDIF() 

include_directories(
 	${Boost_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}/../framework
	${CMAKE_CURRENT_SOURCE_DIR}/../hook
)

link_directories(
)

set(CUI_SRCS
    main.cpp
    SlamLauncher.cpp
    FrameworkCustomizer.cpp
    MapDrawer.cpp
)

set(CUI_HDRS
    SlamLauncher.h
    FrameworkCustomizer.h
    MapDrawer.h
)

add_executable(LittleSLAM
    ${CUI_SRCS}
    ${CUI_HDRS}
)

target_link_libraries(LittleSLAM
  framework
  hook
)

