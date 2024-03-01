#ifndef MVP2_MISSION__HELM_
#define MVP2_MISSION__HELM_

#include <chrono>
#include <string>
#include <memory>
#include <vector>

#include "mvp2_util/lifecycle_node.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "behavior_interface/behavior_base.h"

namespace mvp2_mission
{

class Helm : public mvp2_util::LifecycleNode
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

protected:
  /**
   * @brief Activate lifecycle server
   */
  mvp2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivate lifecycle server
   */
  mvp2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Cleanup lifecycle server
   */
  mvp2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  ////////////////////////////////////////////////////////////////////////////
  
  /**
   * @brief Configure lifecycle server
   */
  mvp2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Shutdown lifecycle server
   */
  mvp2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

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