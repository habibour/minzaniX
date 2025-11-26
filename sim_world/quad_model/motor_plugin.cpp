/**
 * @file motor_plugin.cpp
 * @brief Gazebo Harmonic motor thrust plugin (OPTIONAL)
 * 
 * This plugin subscribes to motor command topics and applies forces/torques
 * to the quadcopter model.
 * 
 * To build: link against gz-sim and gz-transport libraries
 */

#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/double.pb.h>

namespace motor_plugin
{
    class MotorPlugin : public gz::sim::System,
                        public gz::sim::ISystemConfigure,
                        public gz::sim::ISystemPreUpdate
    {
    public:
        MotorPlugin() = default;
        ~MotorPlugin() override = default;

        void Configure(const gz::sim::Entity &_entity,
                      const std::shared_ptr<const sdf::Element> &_sdf,
                      gz::sim::EntityComponentManager &_ecm,
                      gz::sim::EventManager &_eventMgr) override
        {
            // Parse SDF for motor topics
            // Subscribe to motor command topics
            // Store entity reference
        }

        void PreUpdate(const gz::sim::UpdateInfo &_info,
                      gz::sim::EntityComponentManager &_ecm) override
        {
            // Apply thrust forces based on latest motor commands
            // Calculate torques from motor positions
            // Apply to base_link
        }

    private:
        gz::transport::Node node_;
        double motor_thrust_[4] = {0.0};
    };
}

// Register plugin with Gazebo
GZ_ADD_PLUGIN(motor_plugin::MotorPlugin,
              gz::sim::System,
              motor_plugin::MotorPlugin::ISystemConfigure,
              motor_plugin::MotorPlugin::ISystemPreUpdate)
