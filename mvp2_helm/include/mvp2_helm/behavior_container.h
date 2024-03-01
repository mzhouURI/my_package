#ifndef MVP2_MISSION__BEHAVIOR_CONTAINER_
#define MVP2_MISSION__BEHAVIOR_CONTAINER_

#include "behavior_interface/behavior_base.h"

#include "memory"
#include "string"
#include "vector"
#include "map"

#include "pluginlib/class_loader.h"
#include "pluginlib/class_list_macros.hpp"
#include "dictionary.h"


namespace mvp2_mission {

    class BehaviorContainer {
    public:

        //! @brief Trivial constructor
        typedef std::shared_ptr<BehaviorContainer> Ptr;

    private:

        /**
         * @brief Behavior component settings
         *
         */
        behavior_component_t m_opts;

        /**
         * @brief Behavior defined by #m_class_name
         */
        std::shared_ptr<BehaviorBase> m_behavior;

        /**
         * @brief Pluginlib class loader
         */
        std::shared_ptr<pluginlib::ClassLoader<BehaviorBase>> m_class_loader;

    public:

        /**
         * @brief Default constructor
         */
        BehaviorContainer() = default;

        explicit BehaviorContainer(behavior_component_t opts);

        ~BehaviorContainer();

        void initialize();

        auto get_behavior() -> decltype(m_behavior) { return m_behavior; }

        auto get_opts() -> decltype(m_opts) { return m_opts; }

    };

}

#endif // MVP2_MISSION__BEHAVIOR_CONTAINER_