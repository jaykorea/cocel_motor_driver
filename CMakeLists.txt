cmake_minimum_required(VERSION 3.5)
project(cocel_driver)

# Default to C99 for C
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14 for C++
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
find_package(rosidl_default_generators REQUIRED)
find_package(Threads)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorDriverCmd.msg"
  "msg/CocelDriverCmd.msg"
  DEPENDENCIES std_msgs
)

# 라이브러리 소스 파일 정의
set(LIBRARY_SOURCE
    src/CANInterface.cpp
    src/MotorDriver.cpp
)

# 라이브러리 생성
add_library(cocel_driver_lib ${LIBRARY_SOURCE})

# Specify source files
add_executable(cocel_driver_node src/cocel_driver_node.cpp)
target_link_libraries(cocel_driver_node cocel_driver_lib)

# 생성된 메시지 인터페이스를 실행 가능한 타겟에 연결
rosidl_target_interfaces(cocel_driver_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

target_include_directories(cocel_driver_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  cocel_driver_node
  rclcpp
  std_msgs
)

# Installation instructions
install(TARGETS cocel_driver_node
  DESTINATION lib/${PROJECT_NAME})
  
install(TARGETS cocel_driver_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# Linting (optional, for code quality)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch config files.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Package configuration
ament_export_dependencies(rosidl_default_runtime)

ament_package()
