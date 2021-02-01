#ifdef DONT_RUN

/*
 * I've put my code in audrow_main.cpp so that 
 * I can keep what we have here as a working example
 */

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/String.hpp"


template<class FacadeType, class ROSInterfaceType>
struct TypeFacadeDeclaration
{
  // Does the stuff, like serialize and deserialize and converts.
};

int main() {
  rclcpp::init();
  auto node = std::make_shared<rclcpp::Node>("my_node");

  // Most implicit, just facade type.
  auto pub1 = node->create_publisher<std::string>("/chatter", 10);

  // Facade type, but also underlying type.
  auto pub2 = node->create_publisher<std::string, std_msgs::msg::String>("/chatter", 10);

  // Facade type, but also underlying type, with helper class.
  auto pub3 = node->create_publisher<rclcpp::TypeMapping<std::string, std_msgs::msg::String>>("/chatter", 10);

  // Most explicit, explicit english style.
  using MsgT = rclcpp::TypeFacade<std::string>::instead_of<std_msgs::msg::String>;
  auto pub4 = node->create_publisher<MsgT>("/chatter", 10);

  std::string msg = "Hello World";
  pub1->publish(msg);
  pub2->publish(msg);
  pub3->publish(msg);
  pub4->publish(msg);
}

/*
template<class From, class To>
struct TypeMapping
{
  // ...
};

template<class FacadeType>
struct TypeFacade {
  template<class InterfaceType>
  using instead_of = TypeMapping<FacadeType, InterfaceType>;
};

// you can be explicit like this:
using MsgT = rclcpp::TypeFacade<std::string>::instead_of<std_msgs::msg::String>;
rclcpp::Publisher<MsgT>::SharedPtr publisher_;
// ^ you would use this if you want it to be clear what is happening
//   (maybe examples or depending on the preferences of the developer), or
//   you might use it when `std::string` could map to more than one ROS type
//   and you need to select which one to use.

// We could have a short form of this that takes two arguments:
rclcpp::Publisher<TypeMapping<std::string, std_msgs::msg::String>>::SharedPtr publisher_;
// Or this, but it might be prohibitively complicated to implement
// since there are multiple template args already for Publisher/Subscription:
rclcpp::Publisher<std::string, std_msgs::msg::String>::SharedPtr publisher_;

// or you can be implicit:
rclcpp::Publisher<std::string>::SharedPtr publisher_;
// ^ If std::string has multiple types it could represent then this has to error.
*/
#endif