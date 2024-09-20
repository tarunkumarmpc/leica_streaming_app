/*
g++ main.cpp -lboost_system -lboost_thread -lpthread -o leica_streaming_receiver
*/

#include <iostream>
#include <string>
#include <boost/asio.hpp>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

#include "leica_streaming_app/leica_streaming_app_serial_nodelet.hpp"

namespace leica_streaming_app {

LeicaStreamingAppSerialNode::LeicaStreamingAppSerialNode()
  : Node("leica_streaming_app_serial_node"),
    ts_(std::bind(&LeicaStreamingAppSerialNode::locationTSCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3)) {
  
  this->declare_parameter<std::string>("comport", "/dev/ttyUSB0");
  std::string comport;
  this->get_parameter("comport", comport);
  ts_.connect(comport);

  prism_pos_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/leica/position", 10);
  pos_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/paintcopter/position", 10, 
    std::bind(&LeicaStreamingAppSerialNode::positionCb, this, std::placeholders::_1));
  start_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/leica/start_stop", 10, 
    std::bind(&LeicaStreamingAppSerialNode::startStopCb, this, std::placeholders::_1));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
}

LeicaStreamingAppSerialNode::~LeicaStreamingAppSerialNode() {
}

void LeicaStreamingAppSerialNode::positionCb(const nav_msgs::msg::Odometry::SharedPtr msg) {
  ts_.setPrismPosition(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
}

void LeicaStreamingAppSerialNode::startStopCb(const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data) {
    ts_.start();
  } else {
    ts_.end();
  }
}

void LeicaStreamingAppSerialNode::locationTSCallback(const double x,
                                                     const double y,
                                                     const double z) {
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
RCLCPP_COMPONENTS_REGISTER_NODE(leica_streaming_app::LeicaStreamingAppSerialNode)

