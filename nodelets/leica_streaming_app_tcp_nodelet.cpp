#include "leica_streaming_app/leica_streaming_app_tcp_nodelet.hpp"
#include <tf2/LinearMath/Quaternion.h>

namespace leica_streaming_app {

LeicaStreamingAppTCPNode::LeicaStreamingAppTCPNode(const rclcpp::NodeOptions& options)
  : Node("leica_streaming_app_tcp_node", options),
    ts_(std::bind(&LeicaStreamingAppTCPNode::locationTSCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3)) {
  
  this->declare_parameter<std::string>("ip", "10.2.86.54");
  this->declare_parameter<int>("port", 5001);

  std::string ip;
  int port;
  this->get_parameter("ip", ip);
  this->get_parameter("port", port);
  ts_.connect(ip, port);

  prism_pos_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/leica/position", 10);
  pos_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/paintcopter/position", 10, 
    std::bind(&LeicaStreamingAppTCPNode::positionCb, this, std::placeholders::_1));
  start_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/leica/start_stop", 10, 
    std::bind(&LeicaStreamingAppTCPNode::startStopCb, this, std::placeholders::_1));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
}

void LeicaStreamingAppTCPNode::positionCb(const nav_msgs::msg::Odometry::SharedPtr msg) {
  ts_.setPrismPosition(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
}

void LeicaStreamingAppTCPNode::startStopCb(const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data) {
    ts_.start();
  } else {
    ts_.end();
  }
}

void LeicaStreamingAppTCPNode::locationTSCallback(const double x, const double y, const double z) {
  auto msg = std::make_unique<geometry_msgs::msg::PointStamped>();
  msg->header.stamp = this->now();
  msg->header.frame_id = "world";
  msg->point.x = x;
  msg->point.y = y;
  msg->point.z = z;

  prism_pos_pub_->publish(std::move(msg));

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->now();
  t.header.frame_id = "world";
  t.child_frame_id = "leica_pos";
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.translation.z = z;

  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  tf_broadcaster_->sendTransform(t);
}

} // namespace leica_streaming_app

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(leica_streaming_app::LeicaStreamingAppTCPNode)

