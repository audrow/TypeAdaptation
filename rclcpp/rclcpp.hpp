#ifndef RCLCPP 
#define RCLCPP

#include <iostream>
#include <memory>

#include "../std_msgs/msg/String.hpp"


namespace rclcpp {

void init() {
  std::cout << "From init\n";
}


template <typename T> 
struct TypeAdapter {
  static void serialize(T data) {
    std::cout << "Generic serialize: " << data << std::endl;
  }
  static void deserialize();
};

template <>
void TypeAdapter<std::string>::serialize(std::string data) {
  std::cout << "serialize std::string '" << data << "'\n";
}

template <>
void TypeAdapter<std_msgs::msg::String>::serialize(std_msgs::msg::String data) {
  std::cout << "serialize std_msgs::msg::String '" << data.data << "'\n";
}


template <typename T>
struct Publisher {
  std::string name;
  int queue_size;

  Publisher(std::string name_, int queue_size_) {
    name = name_;
    queue_size = queue_size_;
  }

  void publish(T msg) {
    return TypeAdapter<T>::serialize(msg);
  }
};

/*
template<>
void Publisher<std_msgs::msg::String>::publish(std_msgs::msg::String msg) {
    std::cout << "Publishing std_msgs::msg::String '" << msg.data << "' on topic '" << name << "'\n";
}

template<>
void Publisher<std_msgs::msg::String>::publish(std::shared_ptr<std_msgs::msg::String> msg) {
    std::cout << "Publishing shared pointer to std_msgs::msg::String '" << msg->data << "' on topic '" << name << "'\n";
}

template<>
void Publisher<std::string>::publish(std::string msg) {
    std::cout << "Publishing std::string '" << msg << "' on topic '" << name << "'\n";
}
*/


struct Node {
  std::string name;
  Node(std::string name_) {
    name = name_;
  }
  static std::shared_ptr<Node> make_shared(std::string name) {
    return std::make_shared<Node>(name);
  }
  template <typename T>
  std::shared_ptr<Publisher<T>> create_publisher(std::string name, int queue_size) {
    return std::make_shared<Publisher<T>>(name, queue_size);
  }
};
}

#endif