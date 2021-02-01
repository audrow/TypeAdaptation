#ifndef RCLCPP 
#define RCLCPP

#include <iostream>
#include <memory>

#include "../std_msgs/msg/String.hpp"


namespace rclcpp {

void init() {
  std::cout << "From init\n";
}


template <typename CustomType, typename ROSInterfaceType> 
struct TypeAdapter {
  static void serialize(CustomType customType, ROSInterfaceType & msg);
  static void deserialize();
};

template <>
void TypeAdapter<std::string, std_msgs::msg::String>::serialize(std::string data, std_msgs::msg::String & msg) {
  msg.data = data;
  std::cout << "Serialize std::string '" << data << "'\n";
}


template <typename T>
void send(T msg);

template <>
void send(std_msgs::msg::String msg) {
  std::cout << "Sending std_msgs::msg::String '" << msg.data << "'\n";
}


template <typename CustomType, typename ROSInterfaceType>
struct Publisher {
  std::string name;
  int queue_size;

  Publisher(std::string name_, int queue_size_) {
    name = name_;
    queue_size = queue_size_;
  }

  void publish(CustomType content) {
    ROSInterfaceType msg;
    TypeAdapter<CustomType, ROSInterfaceType>::serialize(content, msg);
    send<ROSInterfaceType>(msg);
  }
};


struct Node {
  std::string name;
  Node(std::string name_) {
    name = name_;
  }
  static std::shared_ptr<Node> make_shared(std::string name) {
    return std::make_shared<Node>(name);
  }
  template <typename CustomType, typename ROSInterfaceType>
  std::shared_ptr<Publisher<CustomType, ROSInterfaceType>> create_publisher(std::string name, int queue_size) {
    return std::make_shared<Publisher<CustomType, ROSInterfaceType>>(name, queue_size);
  }
};
}

#endif