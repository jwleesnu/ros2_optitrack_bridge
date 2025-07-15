cmake_minimum_required(VERSION 3.5)
project(ros2_optitrack_bridge)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#     add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# find natnet lib
find_library(natnet_lib libNatNetLibShared.so PATHS ${PROJECT_SOURCE_DIR}/lib)

include_directories(include)

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

add_executable(pose
    src/optitrack_bridge_pose.cpp
    src/natnet_wrapper.cpp
)
ament_target_dependencies(pose rclcpp std_msgs geometry_msgs)

add_executable(odometry
    src/optitrack_bridge_odometry.cpp
    src/linear_kalman_filter.cpp
    src/natnet_wrapper.cpp
)
ament_target_dependencies(odometry rclcpp std_msgs geometry_msgs nav_msgs)
target_include_directories(odometry PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${EIGEN3_INCLUDE_DIR}
)

add_executable(px4
    src/optitrack_bridge_px4.cpp
    src/natnet_wrapper.cpp
)
ament_target_dependencies(px4 
    rclcpp std_msgs geometry_msgs px4_msgs)

add_executable(dummy
    src/optitrack_dummy.cpp
)
ament_target_dependencies(dummy 
    rclcpp std_msgs geometry_msgs px4_msgs)

add_executable(final
    src/optitrack_bridge_final.cpp
    src/linear_kalman_filter.cpp
    src/natnet_wrapper.cpp
)
ament_target_dependencies(final 
    rclcpp std_msgs geometry_msgs px4_msgs nav_msgs)
target_include_directories(final PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(pose ${natnet_lib})
target_link_libraries(odometry ${natnet_lib})
target_link_libraries(px4 ${natnet_lib})
target_link_libraries(final ${natnet_lib})

# install
ament_export_libraries(${natnet_lib})
install(
    TARGETS pose
    DESTINATION lib/${PROJECT_NAME}
)
install(
    TARGETS odometry
    DESTINATION lib/${PROJECT_NAME}
)
install(
    TARGETS px4
    DESTINATION lib/${PROJECT_NAME}
)
install(
    TARGETS dummy
    DESTINATION lib/${PROJECT_NAME}
)
install(
    TARGETS final
    DESTINATION lib/${PROJECT_NAME}
)
install(FILES
    ${natnet_lib}
    DESTINATION lib
)

ament_package()
