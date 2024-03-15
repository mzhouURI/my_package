#ifndef MVP2_MISSION__HELM_
#define MVP2_MISSION__HELM_

#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <thread>

#include "mvp2_util/node_thread.hpp"
#include "node_wrapper.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "behavior_interface/behavior_base.h"
#include "std_msgs/msg/string.hpp"

namespace mvp2_mission
{

class Helm : public mvp2_mission::NodeWrapper
{
public:
    /**
    * @brief A constructor for behavior_server::BehaviorServer
    * @param options Additional options to control creation of the node.
    */
    explicit Helm(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~Helm();

    /**
    * @brief Loads behavior plugins from parameter file
    * @return bool if successfully loaded the plugins
    */
    bool loadBehaviorPlugins();


    void initialize();

    void loop();

protected:
  //! Dedicated callback group and executor for behavior management
  //! in order to isolate each behavior sub/pub.
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::unique_ptr<mvp2_util::NodeThread> executor_thread_;
  rclcpp::TimerBase::SharedPtr helm_timer_;
  void timer_callback();

  std::thread m_controller_worker;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  void topic_callback(const std_msgs::msg::String & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
  }

  /**
   * @brief bhv container
   */
//   std::vector<std::shared_ptr<BehaviorContainer>> m_behavior_containers;
  pluginlib::ClassLoader<mvp2_mission::BehaviorBase> plugin_loader_;
  std::vector<pluginlib::UniquePtr<mvp2_mission::BehaviorBase>> behaviors_;

  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> behavior_ids_;
  std::vector<std::string> behavior_types_;  
};

} // namespace mvp2_mission

#endif // MVP2_MISSION__HELM_