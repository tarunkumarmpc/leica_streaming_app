#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/component_manager.hpp"

int main(int argc, char **argv) {
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Create a component manager
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::NodeOptions options;
  auto component_manager = std::make_shared<rclcpp_components::ComponentManager>(&executor, options);

  // Load the component
  auto component = component_manager->load_component(
    "leica_streaming_app",
    "leica_streaming_app::LeicaStreamingAppSerialNode"
  );

  if (component == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("leica_streaming_app_serial_node"), "Failed to load component");
    return 1;
  }

  // Spin the executor
  executor.spin();

  // Shutdown ROS2
  rclcpp::shutdown();

  return 0;
}

