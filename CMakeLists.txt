cmake_minimum_required(VERSION 3.8)
project(francor_hazemat_find_object_2d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(find_object_2d REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)


find_package(Qt5 COMPONENTS Core Gui REQUIRED)


set(dependencies rclcpp
sensor_msgs
image_transport
cv_bridge
find_object_2d) 



add_executable(object_overlay_node
    src/object_overlay_node.cpp
)

ament_target_dependencies(object_overlay_node ${dependencies})


target_link_libraries(object_overlay_node Qt5::Core Qt5::Gui)

target_include_directories(object_overlay_node PRIVATE ${Qt5Gui_INCLUDE_DIRS})

install(TARGETS object_overlay_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

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

ament_package()
