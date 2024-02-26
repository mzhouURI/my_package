#include "bhv_template/bhv_template.h"
#include "mvp2_util/node_utils.hpp"

using namespace mvp2_helm;

void BehaviorTemplate::initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name) 
{
    m_node = parent;
    auto node = m_node.lock();

    m_logger = m_node->get_logger();

    RCLCPP_INFO(logger_, "Configuring %s", name.c_str());

    // set up parameters
    mvp2_util::declare_parameter_if_not_declared(
        node, "test_double", rclcpp::ParameterValue(1.0));

    node->get_parameter("test_double", m_test_double);

    // ros 
    // m_str_pub = node->template create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    m_str_pub = node->create_publisher<std_msgs::msg::String>("str_pub", 10);

    m_str_sub = node->create_subscription<std_msgs::msg::String>(
        "str_sub", rclcpp::SystemDefaultsQoS(),
        std::bind(&BehaviorTemplate::templateCallback,
        this, std::placeholders::_1));
}

BehaviorTemplate::BehaviorTemplate() {
    std::cout << "A message from the template behavior" << std::endl;
}

void BehaviorTemplate::activated() {
    /**
     * @brief This function is called when the behavior gets activated.
     *
     * @note This function is an inherited virtual function. It may left
     * unimplemented.
     *
     * @details This function is called when the behavior internal state
     * defined by #BehaviorBase::m_actived changes to true.
     */
    std::cout << "Template behavior is activated!" << std::endl;
}

void BehaviorTemplate::disabled() {
    /**
     * @brief This function is called when the behavior gets disabled.
     *
     * @note This function is an inherited virtual function. It may left
     * unimplemented.
     *
     * @details This function is called when the behavior internal state
     * defined by #BehaviorBase::m_actived changes to false.
     */
    std::cout << "Template behavior is disabled!" << std::endl;
}

bool BehaviorTemplate::request_set_point(
    mvp_msgs::msg::ControlProcess *set_point) {

    /**
     * @brief Read the controller values from #BehaviorBase::m_process_values.
     *
     * @details The object type is mvp_controller/ControlProcess. MVP Controller
     * is just a MIMO PID controller with quadratic programming optimizer.
     * Naming convention is just taken from the control theory: [Error, Set
     * Point, Process]. Process and error values are generated by the
     * controller. Vehicle's physical state is in that variable.

     * @code{.cpp}
     *      double surge = BehaviorBase::m_process_values.velocity.x;
     *      double pitch = BehaviorBase::m_process_values.orientation.y;
     * @endcode
     */
    double surge = BehaviorBase::m_process_values.velocity.x;

    /**
     * @brief Write the requested action to #set_point variable
     *
     * @details Helm calls the function to get commands generated by the
     * behavior. Behavior must modify this value with the desired action.
     * Behavior can only control the degrees of freedom defined by the
     * #BehaviorBase::m_dofs vector.
     */
    set_point->velocity.x = -surge;

    /**
     * @brief Return true if you want Helm to use the result of this behavior.
     *
     * @details Helm looks at the return value of this function and decides
     * whether or not to use the result from this function. If the function
     * returns false, helm will not use the result that is written to #set_point
     * variable.
     */
    return true;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mvp2_helm::BehaviorTemplate, mvp2_helm::BehaviorBase)