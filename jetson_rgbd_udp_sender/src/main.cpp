#include "jetson_rgbd_udp_sender/sender_node.hpp"

#include <exception>
#include <iostream>
#include <memory>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int return_code = 0;
  try {
    auto node = std::make_shared<jetson_rgbd_udp_sender::SenderNode>();
    rclcpp::spin(node);
    node->shutdown();
  } catch (const std::exception & exception) {
    std::cerr << "rgbd_udp_sender fatal error: " << exception.what() << std::endl;
    return_code = 1;
  }
  rclcpp::shutdown();
  return return_code;
}
