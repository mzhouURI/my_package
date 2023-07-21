
#include "rclcpp/rclcpp.hpp"

#include "my_package/my_simple_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<MySimpleNode> node = std::make_shared<MySimpleNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
