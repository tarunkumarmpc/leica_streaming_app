cmake_minimum_required(VERSION 3.5)
project(leica_streaming_app)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost REQUIRED )

# Include directories
include_directories(include ${Boost_INCLUDE_DIRS})

# Define the executable
add_executable(leica_streaming_app_node 
   src/main.cpp
   src/leica_streaming_app_node.cpp 
    src/total_station_interface.cpp 
    src/serial_total_station_interface.cpp 
    # src/tcp_total_station_interface.cpp
    src/leica_streaming_app_node.cpp 

)

# Specify dependencies
ament_target_dependencies(leica_streaming_app_node 
    rclcpp 
    geometry_msgs 
    nav_msgs 
    tf2_ros
    rclcpp_components
)

# Link libraries
target_link_libraries(leica_streaming_app_node ${Boost_LIBRARIES})

# Install targets
install(TARGETS
    leica_streaming_app_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.hpp"
)

ament_package()
