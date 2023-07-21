#include "my_package/my_simple_node.hpp"


using namespace std::chrono_literals;
using SimpleSrv = my_interfaces::srv::SimpleSrv;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;


MySimpleNode::MySimpleNode(std::string name) : Node(name)
{
    //topic with namespace
    _publisher = this->create_publisher<std_msgs::msg::String>("~/output_topic", 10);
    //topci without namespace
    _subscriber = this->create_subscription<std_msgs::msg::String>("input_topic", 10, 
                                                                std::bind(&MySimpleNode::simple_callback, 
                                                                this, _1));

    _service = this->create_service<SimpleSrv>("simple_service",
        std::bind(&MySimpleNode::simple_service, this, _1, _2));

    _timer = this->create_wall_timer(
    500ms, std::bind(&MySimpleNode::publish, this));
    _count = 0;
    RCLCPP_INFO(this->get_logger(), "Publisher created!!");
}

void MySimpleNode::publish()
{
    _count++;

    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(_count);
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    _publisher->publish(message);
}

void MySimpleNode::simple_callback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}

void MySimpleNode::simple_service(const std::shared_ptr<my_interfaces::srv::SimpleSrv::Request> request,
                                  const std::shared_ptr<my_interfaces::srv::SimpleSrv::Response> response)
{

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming datat = %s", request->data.c_str());
    response->data = "good \r\n";
}