cmake_minimum_required(VERSION 2.8.3)
project(tb_digital_twin)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)
add_message_files(
        FILES
        Current.msg
        Voltage.msg
        Power.msg
)
generate_messages(
        DEPENDENCIES
        std_msgs
)
catkin_package(
        CATKIN_DEPENDS message_runtime
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(EfficiencyMap src/EfficiencyMap.cpp)
add_executable(Controller
        src/Controller.cpp
        src/pid.cpp)
add_executable(angularConverter src/AngularConverter.cpp)
add_executable(powerCalculator src/PowerCalculator.cpp)
add_executable(currentVoltageFlow
        src/currentVoltageFlow.cpp
        src/ParseDewetron.cpp
        src/PhysicalValueTemplate.cpp)
add_executable(torqueCalculator src/TorqueCalculator.cpp)
## Specify libraries to link a library or executable target against
#target_link_libraries(${PROJECT_NAME}_CCN

target_link_libraries(EfficiencyMap
  ${catkin_LIBRARIES})
target_link_libraries(angularConverter
  ${catkin_LIBRARIES})
target_link_libraries(Controller
  ${catkin_LIBRARIES})
target_link_libraries(currentVoltageFlow
  ${catkin_LIBRARIES})
target_link_libraries(powerCalculator
  ${catkin_LIBRARIES})
target_link_libraries(torqueCalculator
  ${catkin_LIBRARIES})
