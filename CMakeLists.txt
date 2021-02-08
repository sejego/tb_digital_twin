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
add_executable(Efficiency src/EfficiencyMap.cpp)
add_executable(Control
        src/Controller.cpp
        src/pid.cpp)
add_executable(Rotation src/Rotation.cpp)
add_executable(errorCheck src/checkError.cpp)
add_executable(Power src/powerCalculator.cpp)
add_executable(currentVoltageFlow
        src/currentVoltageFlow.cpp
        src/parseDewetron.cpp
        src/PhysicalValueTemplate.cpp)
add_executable(Torque src/torqueCalculator.cpp)
## Specify libraries to link a library or executable target against
#target_link_libraries(${PROJECT_NAME}_CCN

target_link_libraries(Efficiency
  ${catkin_LIBRARIES})
target_link_libraries(Rotation
  ${catkin_LIBRARIES})
target_link_libraries(Control
  ${catkin_LIBRARIES})
target_link_libraries(currentVoltageFlow
  ${catkin_LIBRARIES})
target_link_libraries(Power
  ${catkin_LIBRARIES})
target_link_libraries(errorCheck
  ${catkin_LIBRARIES})
target_link_libraries(Torque
  ${catkin_LIBRARIES})
