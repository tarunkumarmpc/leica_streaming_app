#include "leica_streaming_app/leica_streaming_app_node.h"

using namespace std::chrono_literals;

namespace leica_streaming_app {

LeicaStreamingAppNode::LeicaStreamingAppNode() 
    : Node("leica_streaming_app"),
      ts_(std::bind(&LeicaStreamingAppNode::locationTSCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)), 
      br_(this) { // Initialize TransformBroadcaster here

    this->declare_parameter<std::string>("comport", "/dev/ttyUSB0");
    this->get_parameter("comport", comport_);
    ts_.connect(comport_);

    prism_pos_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/leica/position", 10);
    
    pos_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/paintcopter/position", 10,
        std::bind(&LeicaStreamingAppNode::positionCb, this, std::placeholders::_1));

    start_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/leica/start_stop", 10,
        std::bind(&LeicaStreamingAppNode::startStopCb, this, std::placeholders::_1));
}

void LeicaStreamingAppNode::positionCb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    ts_.setPrismPosition(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
}

void LeicaStreamingAppNode::startStopCb(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
        ts_.start();
    } else {
        ts_.end();
    }
}

void LeicaStreamingAppNode::locationTSCallback(double x, double y, double z) {
    geometry_msgs::msg::PointStamped msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "world";
    msg.point.x = x;
    msg.point.y = y;
    msg.point.z = z;

    prism_pos_pub_->publish(msg);

    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = this->get_clock()->now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "leica_pos";
    transformStamped.transform.translation.x = x;
    transformStamped.transform.translation.y = y;
    transformStamped.transform.translation.z = z;

    q_.setRPY(0, 0, 0);
    transformStamped.transform.rotation.x = q_.x();
    transformStamped.transform.rotation.y = q_.y();
    transformStamped.transform.rotation.z = q_.z();
    transformStamped.transform.rotation.w = q_.w();

    br_.sendTransform(transformStamped);
}

} // namespace leica_streaming_app
