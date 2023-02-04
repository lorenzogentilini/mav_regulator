cmake_minimum_required(VERSION 2.8.3)
project(mav_regulator)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
  geometry_msgs
  quadrotor_msgs
  message_generation
)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED) 

add_message_files(
  DIRECTORY
    msg
  FILES
   SetPoint.msg
   Bezier.msg
   BezierMultiArray.msg
   BSpline.msg
   Point.msg
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mav_regulator
  CATKIN_DEPENDS  roscpp
                  rospy
                  tf
                  std_msgs
                  geometry_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_library(mpc_solver
  thirdparty/qpoases/SRC/Bounds.cpp
  thirdparty/qpoases/SRC/Constraints.cpp
  thirdparty/qpoases/SRC/CyclingManager.cpp
  thirdparty/qpoases/SRC/Indexlist.cpp
  thirdparty/qpoases/SRC/MessageHandling.cpp
  thirdparty/qpoases/SRC/QProblem.cpp
  thirdparty/qpoases/SRC/QProblemB.cpp
  thirdparty/qpoases/SRC/SubjectTo.cpp
  thirdparty/qpoases/SRC/Utils.cpp
  thirdparty/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  model/quadrotor_mpc_model_codegen/acado_qpoases_interface.cpp
  model/quadrotor_mpc_model_codegen/acado_integrator.c
  model/quadrotor_mpc_model_codegen/acado_solver.c
  model/quadrotor_mpc_model_codegen/acado_auxiliary_functions.c
)

target_include_directories(mpc_solver PUBLIC
  thirdparty/qpoases
  thirdparty/qpoases/INCLUDE
  thirdparty/qpoases/SRC
  model/quadrotor_mpc_model_codegen/
)

add_library(${PROJECT_NAME}_lib
  src/mav_regulator.cpp
  src/performance_eval.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  mpc_solver
)

## Declare C++ executables
add_executable(mav_regulator_node src/mav_regulator_node.cpp)
target_link_libraries(mav_regulator_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_lib
)
add_dependencies(mav_regulator_node ${PROJECT_NAME}_generate_messages_cpp)

add_executable(performance_eval_node src/performance_eval_node.cpp)
target_link_libraries(performance_eval_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_lib
)
add_dependencies(performance_eval_node ${PROJECT_NAME}_generate_messages_cpp)

add_executable(sitl_test_node src/sitl_test_node.cpp)
target_link_libraries(sitl_test_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_lib
)
add_dependencies(sitl_test_node ${PROJECT_NAME}_generate_messages_cpp)