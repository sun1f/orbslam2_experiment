cmake_minimum_required(VERSION 2.8)
project(pose_graph)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# Eigen
include_directories("/usr/include/eigen3")

# sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# g2o
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

#pose_graph_SE3
add_executable(pose_graph_g2o_SE3_GN ./src/pose_graph_g2o_SE3_GN.cpp)
target_link_libraries(pose_graph_g2o_SE3_GN
        g2o_core g2o_stuff g2o_types_slam3d ${CHOLMOD_LIBRARIES}
)
add_executable(pose_graph_g2o_SE3_LM ./src/pose_graph_g2o_SE3_LM.cpp)
target_link_libraries(pose_graph_g2o_SE3_LM
        g2o_core g2o_stuff g2o_types_slam3d ${CHOLMOD_LIBRARIES}
)
add_executable(pose_graph_g2o_SE3_DL ./src/pose_graph_g2o_SE3_DL.cpp)
target_link_libraries(pose_graph_g2o_SE3_DL
        g2o_core g2o_stuff g2o_types_slam3d ${CHOLMOD_LIBRARIES}
)

#pose_graph_SE2
add_executable(pose_graph_g2o_SE2_GN ./src/pose_graph_g2o_SE2_GN.cpp)
target_link_libraries(pose_graph_g2o_SE2_GN
        g2o_core g2o_stuff g2o_types_slam2d ${CHOLMOD_LIBRARIES}
)
add_executable(pose_graph_g2o_SE2_LM ./src/pose_graph_g2o_SE2_LM.cpp)
target_link_libraries(pose_graph_g2o_SE2_LM
        g2o_core g2o_stuff g2o_types_slam2d ${CHOLMOD_LIBRARIES}
)
add_executable(pose_graph_g2o_SE2_DL ./src/pose_graph_g2o_SE2_DL.cpp)
target_link_libraries(pose_graph_g2o_SE2_DL
        g2o_core g2o_stuff g2o_types_slam2d ${CHOLMOD_LIBRARIES}
)

#estimate result
add_executable(convert_g2o_2d ./estimate/convert_g2o_2d.cpp)
target_link_libraries(convert_g2o_2d
        g2o_core g2o_stuff g2o_types_slam2d ${CHOLMOD_LIBRARIES}
)
add_executable(convert_g2o_3d ./estimate/convert_g2o_3d.cpp)
target_link_libraries(convert_g2o_3d
        g2o_core g2o_stuff g2o_types_slam2d ${CHOLMOD_LIBRARIES}
)