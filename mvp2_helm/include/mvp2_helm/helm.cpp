#include <memory>
#include <string>
#include <vector>
#include <utility>
#include "mvp2_util/node_utils.hpp"
#include "mvp2_helm/helm.h"

using namespace std::chrono_literals;

namespace mvp2_mission 
{

Helm::Helm(const rclcpp::NodeOptions & options)
: NodeWrapper("mvp2_helm", "", options),
  plugin_loader_("behavior_interface", "mvp2_mission::BehaviorBase"),
  default_ids_{"template"},
  default_types_{"mvp2_mission/BehaviorTemplate"}
{
    RCLCPP_INFO(get_logger(), "helm constructor");

    // parameters
    declare_parameter("behavior_plugins", default_ids_);
    get_parameter("behavior_plugins", behavior_ids_);

    get_parameter("behavior_plugins", behavior_ids_);
    if (behavior_ids_ == default_ids_) {
        for (size_t i = 0; i < default_ids_.size(); ++i) {
        declare_parameter(default_ids_[i] + ".plugin", default_types_[i]);
        }
    }

    // normal ros setup
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic_pub", 10);
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic_sub", 10, std::bind(&Helm::topic_callback, this, std::placeholders::_1));


    //! init a individual thread for behavior management
    callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);  

    auto cb_opt = rclcpp::SubscriptionOptions();
    cb_opt.callback_group = callback_group_;

    helm_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      //! this make one time callback ?
      // [this]() -> void { helm_timer_->cancel(); timer_callback();},
      [this]() -> void {timer_callback();},
      callback_group_);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_callback_group(callback_group_, get_node_base_interface());
    executor_thread_ = std::make_unique<mvp2_util::NodeThread>(executor_); 
}

Helm::~Helm()
{
  behaviors_.clear();
}


void Helm::initialize() {

    // configure plugins
    RCLCPP_INFO(get_logger(), "Configuring");
    behavior_types_.resize(behavior_ids_.size());
    if (!loadBehaviorPlugins()) {
      RCLCPP_INFO(get_logger(), "Plugins configured failed!");
      std::exit(EXIT_FAILURE);
    }          

    // new working thread
    m_controller_worker = std::thread([this] { loop(); });
    m_controller_worker.detach();    
}

void Helm::loop() {

  //! running rclcpp::shutdown() will shutdown the system ?
  // rclcpp::Rate r(2);
  // while(rclcpp::ok() && !rclcpp::shutdown()) {
  //     RCLCPP_INFO(get_logger(), "hi from helm loop");
  //     r.sleep();
  // }

  rclcpp::Rate r(1);
  while(rclcpp::ok()) {
    RCLCPP_INFO(get_logger(), "hi from helm loop");
    r.sleep();
    // std::chrono::milliseconds dura(500);
    // std::this_thread::sleep_for(dura);
  }
}

void Helm::timer_callback() {
  RCLCPP_INFO(get_logger(), "hi from helm additional timer callback!");
}

bool 
Helm::loadBehaviorPlugins()
{
  auto node = shared_from_this();

  for (size_t i = 0; i != behavior_ids_.size(); i++) {
    behavior_types_[i] = mvp2_util::get_plugin_type_param(node, behavior_ids_[i]);
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

#include "rclcpp_components/register_node_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(mvp2_mission::Helm)
