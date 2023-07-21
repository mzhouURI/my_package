#ifndef MY_SIMPLE_NODE_HPP_
#define MY_SIMPLE_NODE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "my_interfaces/srv/simple_srv.hpp"

class MySimpleNode : public rclcpp::Node
{

public:
    MySimpleNode(std::string name = "publisher");

    

    void publish();

private:
    //Pub topics
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher;

    //sub topics
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subscriber;

    //service 
    rclcpp::Service<my_interfaces::srv::SimpleSrv>::SharedPtr _service;

    //sub callback
    void simple_callback(const std_msgs::msg::String::SharedPtr msg);

    //service call
    void simple_service(
        const std::shared_ptr<my_interfaces::srv::SimpleSrv::Request> request,
        const std::shared_ptr<my_interfaces::srv::SimpleSrv::Response> response);


    rclcpp::TimerBase::SharedPtr _timer;
    int _count;
};

#endif  // MY_SIMPLE_NODE_HPP_





    