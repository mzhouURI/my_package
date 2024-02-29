#include <memory>
#include <string>
#include <vector>
#include <utility>
#include "mvp2_util/node_utils.hpp"
#include "mvp2_helm/helm.h"

namespace mvp2_mission 
{

Helm::Helm(const rclcpp::NodeOptions & options)
: LifecycleNode("behavior_server_helm", "", options),
  plugin_loader_("behavior_interface", "mvp2_mission::BehaviorBase"),
  default_ids_{"template"},
  default_types_{"mvp2_mission/BehaviorTemplate"}
{
    declare_parameter("behavior_plugins", default_ids_);
    get_parameter("behavior_plugins", behavior_ids_);

    get_parameter("behavior_plugins", behavior_ids_);
    if (behavior_ids_ == default_ids_) {
        for (size_t i = 0; i < default_ids_.size(); ++i) {
        declare_parameter(default_ids_[i] + ".plugin", default_types_[i]);
        }
    }
}

Helm::~Helm()
{
  behaviors_.clear();
}


mvp2_util::CallbackReturn
Helm::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  // setup some pub & sub

  // load behaviors
  behavior_types_.resize(behavior_ids_.size());
  if (!loadBehaviorPlugins()) {
    return mvp2_util::CallbackReturn::FAILURE;
  }

  return mvp2_util::CallbackReturn::SUCCESS;
}

bool 
Helm::loadBehaviorPlugins()
{
  auto node = shared_from_this();

  for (size_t i = 0; i != behavior_ids_.size(); i++) {
    behavior_types_[i] = nav2_util::get_plugin_type_param(node, behavior_ids_[i]);
    try {
      RCLCPP_INFO(
        get_logger(), "Creating behavior plugin %s of type %s",
        behavior_ids_[i].c_str(), behavior_types_[i].c_str());
      behaviors_.push_back(plugin_loader_.createUniqueInstance(behavior_types_[i]));
      behaviors_.back()->initialize(node, behavior_ids_[i]);
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(), "Failed to create behavior %s of type %s."
        " Exception: %s", behavior_ids_[i].c_str(), behavior_types_[i].c_str(),
        ex.what());
      return false;
    }
  }

  return true;
}

} // namespace mvp2_mission