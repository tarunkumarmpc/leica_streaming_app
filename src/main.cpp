#include "rclcpp/rclcpp.hpp"
#include "leica_streaming_app/leica_streaming_app_node.h"
#include <rclcpp/executors/multi_threaded_executor.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Create an instance of your node
    auto node = std::make_shared<leica_streaming_app::LeicaStreamingAppNode>();

    // Create a multi-threaded executor
    rclcpp::executors::MultiThreadedExecutor executor;

    // Add your node to the executor
    executor.add_node(node);

    // Spin the executor
    executor.spin();

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}

