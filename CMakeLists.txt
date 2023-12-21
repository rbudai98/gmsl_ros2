cmake_minimum_required(VERSION 3.8)
project(gmsl_ros2)

set(CMAKE_PREFIX_PATH "/opt/glog;/opt/protobuf;/opt/websockets")

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

find_package(Protobuf 3.9.0 REQUIRED)
find_package(Libwebsockets REQUIRED)
# set(LWS_CMAKE_DIR ${LWS_CMAKE_DIR} PARENT_SCOPE)

protobuf_generate_cpp(PROTO_SRCS PROTO_HRDS ${CMAKE_SOURCE_DIR}/buffer.proto)
get_filename_component(GENERATED_PROTO_FILES_DIR ${PROTO_HRDS} DIRECTORY)


set(SOURCE ${CMAKE_SOURCE_DIR}/include/gmsl_ros2)
set(SOURCES ${CMAKE_SOURCE_DIR}/include/gmsl_ros2)

FILE(GLOB_RECURSE HeaderFiles ${CMAKE_SOURCE_DIR}/include/gmsl_ros2/*.h)
file(GLOB SOURCES SourceFiles ${CMAKE_SOURCE_DIR}/src/*.cpp)

add_executable(gmsl_ros2 ${CMAKE_CURRENT_SOURCE_DIR}/src/gmsl_ros2_node.cpp ${SOURCES} ${HeaderFiles} ${SourceFiles} ${PROTO_SRCS} ${PROTO_HDRS})

target_include_directories(gmsl_ros2 PUBLIC ${CMAKE_SOURCE_DIR}/include/gmsl_ros2)
target_sources(gmsl_ros2 INTERFACE ${CMAKE_SOURCE_DIR}/include/gmsl_ros2)

ament_target_dependencies(gmsl_ros2 rclcpp std_msgs sensor_msgs)

target_include_directories(${PROJECT_NAME}
PRIVATE
        src/
        ${Protobuf_INCLUDE_DIRS}
        ${GENERATED_NET_PROTO_FILES_DIR}
        ${GENERATED_PROTO_FILES_DIR}
        ${GENERATED_NET_PROTO_FILES_DIR}
        ${LIBWEBSOCKETS_INCLUDE_DIRS}

${OPENSSL_INCLUDE_DIRS}
)

target_link_libraries( ${PROJECT_NAME}
        websockets_shared
        ${Protobuf_LIBRARIES}
)

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
