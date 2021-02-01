#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/String.hpp"


int main() {
  // Setup
  rclcpp::init();
  auto node = rclcpp::Node::make_shared("my node");


  // Publish a std::string
  auto std_string_publisher = node->create_publisher<std::string>("my/std/string/publisher", 10);
  std_string_publisher->publish("My message");


  // Publish a std_msgs::msg::String
  auto msg = std::make_shared<std_msgs::msg::String>();
  msg->data = "String data";
  std::cout << msg->data << std::endl;

  auto std_msgs_string_publisher = node->create_publisher<std_msgs::msg::String>("my/std_msgs/msg/string/publisher", 10);
  std_msgs_string_publisher->publish(msg);
}