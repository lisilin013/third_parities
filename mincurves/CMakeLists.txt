cmake_minimum_required(VERSION 2.8.3)
project(mincurves)

find_package(catkin_simple 0.1.0 REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

include_directories(${GTSAM_INCLUDE_DIR})

cs_add_library(${PROJECT_NAME} 
  src/SE3Curve.cpp
  src/DiscreteSE3Curve.cpp
  src/KeyGenerator.cpp
)

find_package(Boost REQUIRED COMPONENTS system thread)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

cs_install()
cs_export()
