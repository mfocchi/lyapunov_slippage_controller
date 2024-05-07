cmake_minimum_required(VERSION 3.5)
project(lyapunov_slippage_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(optim_interfaces REQUIRED) 



install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

add_library(lyapunovController include/${PROJECT_NAME}/lyapunovController.cpp)
add_library(error_codes include/${PROJECT_NAME}/error_codes.cpp)
add_library(motionModels include/${PROJECT_NAME}/motionModels.cpp)
add_library(generalPurpose include/${PROJECT_NAME}/generalPurpose.cpp)
add_library(coppeliaSimNode  include/${PROJECT_NAME}/coppeliaSimNode.cpp)
add_library(differential_drive_model  include/${PROJECT_NAME}/differential_drive_model.cpp)






add_executable(differential_drive_ol_node src/differential_drive_ol_node.cpp)
add_executable(slippage_controller_node src/slippage_controller_node.cpp)
add_executable(feedbackTester src/feedbackTester.cpp)

ament_target_dependencies(coppeliaSimNode
  std_msgs
  rclcpp
)

target_link_libraries(motionModels generalPurpose)
target_link_libraries(lyapunovController motionModels)
target_link_libraries(lyapunovController error_codes)
target_link_libraries(differential_drive_ol_node motionModels)
target_link_libraries(differential_drive_ol_node differential_drive_model)
target_link_libraries(differential_drive_ol_node coppeliaSimNode)
target_link_libraries(slippage_controller_node lyapunovController)
target_link_libraries(slippage_controller_node differential_drive_model)
target_link_libraries(slippage_controller_node coppeliaSimNode)
target_link_libraries(feedbackTester motionModels)

target_include_directories(differential_drive_ol_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(slippage_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(feedbackTester PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


ament_target_dependencies(differential_drive_ol_node 
  Eigen3 
  rclcpp 
  geometry_msgs 
  sensor_msgs
  std_msgs
  tf2
  tf2_ros)

ament_target_dependencies(slippage_controller_node
  Eigen3 
  rclcpp 
  geometry_msgs 
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  optim_interfaces #this contains the header of the generated custom service call optim
  )

ament_target_dependencies(feedbackTester 
  Eigen3 
  rclcpp 
  geometry_msgs 
  tf2
  tf2_ros)

install(TARGETS 
  differential_drive_ol_node
  DESTINATION lib/${PROJECT_NAME}
  )
install(TARGETS 
  slippage_controller_node
  DESTINATION lib/${PROJECT_NAME}
  )
install(TARGETS 
  feedbackTester
  DESTINATION lib/${PROJECT_NAME}
  )


#test for service call
add_executable(client_service_call_optimizer
  src/test_service_call_optimizer.cpp)

ament_target_dependencies(client_service_call_optimizer
  rclcpp
  optim_interfaces
)

install(TARGETS  
client_service_call_optimizer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY  
   config
   DESTINATION share/${PROJECT_NAME}
)


ament_package()
