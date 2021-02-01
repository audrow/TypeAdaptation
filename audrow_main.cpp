#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/String.hpp"


/*
 * Next steps:
 * * Make the type adapter
 */


int main() {

  // Setup
  rclcpp::init();
  auto node = rclcpp::Node::make_shared("my node");


  // Publish a std::string
  auto std_string_publisher = node->create_publisher<std::string>("my/std/string/publisher", 10);
  std_string_publisher->publish("My message");


  // Publish a std_msgs::msg::String
  // Shared pointer version
  auto msg = std::make_shared<std_msgs::msg::String>();
  msg->data = "String data by shared pointer";
  node->create_publisher<std_msgs::msg::String>("std_msgs/msg/string/ref/publisher", 10)->publish(msg);

  // Pass by value
  std_msgs::msg::String msg2;
  msg2.data = "String data by value";
  node->create_publisher<std_msgs::msg::String>("std_msgs/msg/string/value/publisher", 10)->publish(msg2);
}