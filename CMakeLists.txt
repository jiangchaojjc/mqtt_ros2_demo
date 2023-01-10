cmake_minimum_required(VERSION 3.5)
project(mqtt_test)

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
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(charge_interface REQUIRED)
find_package(rcsbot_interface REQUIRED)

nav2_package()

link_directories(/usr/local/lib) #添加动态连接库的路径

include_directories(${PROJECT_SOURCE_DIR}/include /usr/local/include)

set(executable_name mqtt_navi)

set(library_name ${executable_name}_core)

add_library(${library_name} SHARED
 src/mqtt.cpp
 src/msg_work.cpp
 src/ros2_mqtt_bridge.cpp
 )

set(dependencies     
        rclcpp
        rclcpp_action
        rclcpp_lifecycle
        nav_msgs
        nav2_msgs
        nav2_util
        tf2_ros 
        charge_interface
        rcsbot_interface
        )

target_link_libraries(${library_name} paho-mqtt3a jsoncpp)

ament_target_dependencies(${library_name}
  ${dependencies}
)

add_executable(${executable_name} src/main.cpp)

ament_target_dependencies(${executable_name}
${dependencies}
)

target_link_libraries(${executable_name} ${library_name})


install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_package()
