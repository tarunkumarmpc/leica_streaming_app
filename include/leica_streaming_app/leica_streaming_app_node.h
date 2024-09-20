#pragma once

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "leica_streaming_app/serial_total_station_interface.h" // Ensure this interface is defined

namespace leica_streaming_app {

class LeicaStreamingAppNode : public rclcpp::Node {
public:
    LeicaStreamingAppNode();
    ~LeicaStreamingAppNode() = default; // Default destructor

private:
    void positionCb(const nav_msgs::msg::Odometry::SharedPtr msg);
    void startStopCb(const std_msgs::msg::Bool::SharedPtr msg);
    void locationTSCallback(double x, double y, double z);

    std::string comport_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr prism_pos_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pos_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_stop_sub_;
    tf2_ros::TransformBroadcaster br_; // Declare here

    geometry_msgs::msg::TransformStamped transformStamped_;
    tf2::Quaternion q_;

    SerialTSInterface ts_; // Ensure this is declared as a member variable
};

} // namespace leica_streaming_app
