
cmake_minimum_required(VERSION 3.8)
project(robotcontrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(EXTERNAL_INCLUDE_DIR_1 "/home/lab/RM_1/auto_aid_node_1/auto_aid_node_/armor/install/interface/include/interface")
set(EXTERNAL_INCLUDE_DIR_2 "/home/lab/RM_1/auto_aid_node_1/auto_aid_node_/armor/src/Armor")
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interface REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 检测系统位数并设置相应的路径和库
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(BMCAN_INCLUDE_DIR "./lib/bin/unix64/release/")
    set(BMCAN_LIB_NAME "bmapi64")
else()
    set(BMCAN_INCLUDE_DIR "./lib/bin/unix32/release")
    set(BMCAN_LIB_NAME "bmapi")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
# 添加bmcan include和库搜索路径
# 添加bmcan include和库搜索路径
# 添加bmcan include和库搜索路径
include_directories(${EXTERNAL_INCLUDE_DIR_1})
include_directories(${EXTERNAL_INCLUDE_DIR_2})
include_directories("./lib/include")
link_directories(${BMCAN_INCLUDE_DIR})
include_directories("./include/robotcontrol")

add_executable(motor_pid_node 
  src/motor_pid_node.cpp
)
target_include_directories(motor_pid_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(motor_pid_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  motor_pid_node
  rclcpp
  interface
)
install(TARGETS 
  motor_pid_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(motor_test_node 
  src/motor_test_node.cpp
)
target_include_directories(motor_test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(motor_test_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  motor_test_node
  rclcpp
  interface
)
install(TARGETS 
  motor_test_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(motor_send_node 
  src/motor_send.cpp
  src/bmcan_bus.cpp
)

# 链接库
target_link_libraries(motor_send_node ${BMCAN_LIB_NAME} usb-1.0)

target_include_directories(motor_send_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(motor_send_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  motor_send_node
  "rclcpp"
  "interface"
)

install(TARGETS 
  motor_send_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装Python脚本
install(
  PROGRAMS
  scripts/motor_test.py
  DESTINATION lib/${PROJECT_NAME}
)


# 安装launch文件夹
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


