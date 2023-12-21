cmake_minimum_required(VERSION 3.8)
project(gmsl_ros2)

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
find_package(sensor_msgs REQUIRED)
find_package(Libwebsockets REQUIRED)


set(SOURCE ${CMAKE_SOURCE_DIR}/include)
set(SOURCES ${CMAKE_SOURCE_DIR}/include)

FILE(GLOB_RECURSE HeaderFiles ${CMAKE_SOURCE_DIR}/include/*.h)
file(GLOB SOURCES SourceFiles ${CMAKE_SOURCE_DIR}/src/*.cpp)

add_executable(gmsl_ros2 ${CMAKE_CURRENT_SOURCE_DIR}/src/gmsl_ros2_node.cpp ${SOURCES} ${HeaderFiles} ${SourceFiles})

target_include_directories(gmsl_ros2 PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_sources(gmsl_ros2 INTERFACE ${CMAKE_SOURCE_DIR}/include)

ament_target_dependencies(gmsl_ros2 rclcpp std_msgs sensor_msgs)

target_link_libraries(gmsl_ros2 websockets_shared)
target_include_directories(gmsl_ros2 PUBLIC "${LIBWEBSOCKETS_INCLUDE_DIRS}")

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
