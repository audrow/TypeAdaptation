#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/String.hpp"

/*
 * Next steps
 * * Use a TypeAdapt object to create publisher
 */

int main() {

  rclcpp::init();

  auto node = rclcpp::Node::make_shared("my node");
  auto publisher = node->create_publisher<rclcpp::TypeAdapter<std::string, std_msgs::msg::String>>("my/publisher", 10);

  std::string my_string = "My string data";
  publisher->publish(my_string);
}