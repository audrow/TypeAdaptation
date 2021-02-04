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

template<class CustomType, class ROSInterfaceType>
struct ROSMessage
{
  using ROSInterfaceType = ROSInterfaceType;
  using ActualType = CustomType;

  static
  const rosidl_typesupport_t *
  get_typesupport()
  {
    // from:
    //  https://github.com/ros2/rclcpp/blob/1c92e6d6091aae88422ad568761b736abf846e98/rclcpp/include/rclcpp/publisher.hpp#L81
    return rosidl_typesupport_cpp::get_message_type_support_handle<ROSInterfaceType>();
  }
};

template <typename T>
void send(T msg);

template <>
void send(std_msgs::msg::String msg) {
  std::cout << "Sending std_msgs::msg::String '" << msg.data << "'\n";
}

// https://github.com/ros2/rosidl/blob/ef9c561d639aa257b80e4788e6ef33503c9431d8/rosidl_runtime_cpp/include/rosidl_runtime_cpp/traits.hpp#L163-L164
template <
  typename MessageT
>
struct Publisher {
  std::string name;
  int queue_size;

  Publisher(std::string name_, int queue_size_) {
    name = name_;
    queue_size = queue_size_;
    rcl_publisher_t rcl_publisher = rcl_get_zero_initialized_publisher();
    rcl_ret_t ret = rcl_publisher_init(
      &rcl_publisher,
      ROSMessageT::get_typesupport(),
      /* ... */
    );
  }

  void publish(const typename ROSMessageT::ActualType & content) {
    // TODO figure out how to pass adapter in but keep this pattern
    ROSInterfaceType msg;
    AdapterType::serialize(content, msg);
    send<ROSInterfaceType>(msg);
  }
};

template<class NodeT, class MessageT>
std::shared_ptr<rclcpp::Publisher<MessageT>>
rclcpp::create_publisher<MessageT>(NodeT, "topic name", 10);

struct Node {
  std::string name;
  Node(std::string name_) {
    name = name_;
  }
  static std::shared_ptr<Node> make_shared(std::string name) {
    return std::make_shared<Node>(name);
  }
  template <typename AdapterType>
  std::shared_ptr<Publisher<AdapterType>> create_publisher(std::string name, int queue_size) {
    // TODO(wjwwood): should be rclcpp::create_publisher()
    return std::make_shared<Publisher<AdapterType>>(name, queue_size);
  }
};
}

#endif