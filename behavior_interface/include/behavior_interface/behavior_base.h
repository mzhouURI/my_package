#ifndef MVP2_MISSION__BEHAVIOR_BASE_
#define MVP2_MISSION__BEHAVIOR_BASE_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "mvp2_util/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include <mvp_msgs/msg/control_process.hpp>

namespace mvp2_mission
{

class BehaviorBase {
private:
    friend class Helm;

    friend class BehaviorContainer;

    /**
        * @brief Unique name for the behavior.
        * This name will later be used as namespace for ros node handler.
        *
        * @todo A behavior may or may not use a parameter from ROS. However,
        *       this name is still a necessity.
        */
    std::string m_name;

    /**
        * @brief Frequency of the helm
        */
    double m_helm_frequency;

    /**
        * @brief Global link of helm
        */
    std::string m_global_link;

    /**
        * @brief Local link of helm
        */
    std::string m_local_link;

    /**
        * @brief A string holds the active state name
        */
    std::string m_active_state;

    /**
        * @brief A vector holds DOFs active at the moment by controller
        *
        */
    std::vector<int> m_active_dofs;

    /**
        * @brief Behaviors calls this function to request a state change from
        *        MVP-Helm.
        *
        * This function is set during the runtime to map one of the functions
        * from MVP-Helm.
        */
    std::function<bool(const std::string&)> f_change_state;

    void f_set_active_state(const std::string& state) {
        m_active_state = state;
        state_changed(state);
    }

    /**
        * @brief This function is called by the MVP-Helm everytime if a
        *        behavior is active in the given state.
        */
    void f_activate()
    {
        if(!m_activated) {
            activated();
        }
        m_activated = true;
    }

    /**
        * @brief This function is called by the MVP-Helm everytime if a
        *        behavior is *not* active in the given state.
        */
    void f_disable()
    {
        if(m_activated) {
            disabled();
        }
        m_activated = false;
    }

protected:
    /**
        * @brief A vector holds controlled DOFs by behavior
        * Each behavior must present the degrees of freedoms that they want
        * to control. Helm will be controlling this information during
        * execution.
        */
    std::vector<int> m_dofs;

    /**
        * @brief Registered state of the of the low level controller
        */
    mvp_msgs::msg::ControlProcess m_process_values;

    /**
        * @brief a member variable to hold activity state of the behavior
        */
    bool m_activated = false;

    /**
        * @brief This function is triggered when a behavior gets activated
        * A plugin may or may not override this function.
        */
    virtual void activated() = 0;

    /**
        * @brief This function is triggered when a behavior gets disabled
        * A plugin may or may not override this function.
        */
    virtual void disabled() = 0;

    /**
        * @brief
        *
        * @param state_name
    */
    virtual void state_changed(const std::string& state_name) {
        printf("name=%s\n", state_name.c_str());
    }

    virtual auto change_state(const std::string& state) -> bool final {
        return f_change_state(state);
    }

    virtual double get_helm_frequency() final { return m_helm_frequency; }

    virtual std::string get_helm_global_link() final { return m_global_link; }

    virtual std::string get_helm_local_link() final { return m_local_link; }

    virtual auto configure_dofs() -> decltype(m_dofs) {return decltype(m_dofs)();};

public:

    /**
        * @brief Construct a new Behavior Base object
        *
        */
    BehaviorBase() = default;

    virtual ~BehaviorBase() = default;    

    /**
        * @brief retrieve degrees of freedoms controlled by the behavior
        *
        * @return std::vector<ctrl::DOF::IDX>
        */
    virtual auto get_dofs() -> const decltype(m_dofs)& final
    {
        return m_dofs;
    }

    virtual auto get_name() -> std::string final { return m_name; }

    /**
        * @brief
        * @param set_point
        * @return true if Behavior wants helm to use its result
        * @return false if Behavior doesn't want helm to use its result
        */
    virtual bool request_set_point(mvp_msgs::msg::ControlProcess* set_point) = 0;

    /**
    * @param  parent pointer to user's node
    * @param  name The name of this planner
    * @param  tf A pointer to a TF buffer
    */
    virtual void initialize(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        const std::string &name) = 0;

    /**
     * @brief Trivial generic pointer type
     */
    typedef std::shared_ptr<BehaviorBase> Ptr;
};

} // namespace mvp2_mission

#endif  // MVP2_MISSION__BEHAVIOR_BASE_