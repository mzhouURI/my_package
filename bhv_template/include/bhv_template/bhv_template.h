#ifndef MVP2_MISSION__BEHAVIOR_TEMPLATE_
#define MVP2_MISSION__BEHAVIOR_TEMPLATE_

#include <memory>
#include <string>
#include <cmath>
#include <chrono>
#include <ctime>
#include <thread>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "mvp2_util/robot_utils.hpp"
#include "behavior_interface/behavior_base.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

namespace mvp2_mission 
{

using namespace std::chrono_literals;  //NOLINT

class BehaviorTemplate : public BehaviorBase
{
private:
    /**
        * @brief This function is triggered when a behavior gets activated
        * A plugin may or may not override this function.
        */
    virtual void activated() override;

    /**
        * @brief This function is triggered when a behavior gets disabled
        * A plugin may or may not override this function.
        */
    virtual void disabled() override;

    /**
    * @param  parent pointer to user's node
    * @param  name The name of this planner
    * @param  tf A pointer to a TF buffer
    */
    virtual void initialize(
        const rclcpp::Node::WeakPtr &parent,
        const std::string &name) override;

    /**
    * @brief Callback function for velocity subscriber
    * @param msg received Twist message
    */
    void templateCallback(const std_msgs::msg::String::SharedPtr msg);

    // ros2 related
    rclcpp::Node::WeakPtr m_node;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_str_pub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_str_sub;
    rclcpp::Logger m_logger{rclcpp::get_logger("mvp2_mission")};

    // variables
    double m_test_double;

public:
    /**
        * @brief Trivial constructor
        */
    BehaviorTemplate();

    /**
        * @brief Request set point from the behavior. It is consumed by helm.
        *
        * @param msg Result value of the behavior. This value is written by the
        *            Behavior. Helm uses this variable to generate set_point
        *            for the controller.
        * @return true if you want helm to use the result.
        * @return false if you don't want helm to use the result.
        */
    bool request_set_point(mvp_msgs::msg::ControlProcess *msg) override;    

};

} // namespace mvp2_mission

#endif // MVP2_MISSION__BEHAVIOR_TEMPLATE_